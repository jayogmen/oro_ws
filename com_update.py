import os
import json
import time
import logging
import requests
import git
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple, Set
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
import sys

# Previous dataclass definitions remain the same
@dataclass
class ArtifactMetadata:
    buildTime: str
    gitCommit: str
    branch: str
    workflow: str

@dataclass
class Version:
    version: str
    metadata: Optional[ArtifactMetadata] = None

@dataclass
class UpdateData:
    latestSHA: str
    artifactUrl: str
    updateType: str
    metadata: Optional[ArtifactMetadata] = None

@dataclass
class UpdateResponse:
    status: bool
    message: str
    data: Optional[UpdateData]

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'UpdateResponse':
        if 'data' in data and data['data']:
            metadata = None
            if 'metadata' in data['data']:
                metadata = ArtifactMetadata(
                    buildTime=data['data']['metadata'].get('buildTime', ''),
                    gitCommit=data['data']['metadata'].get('gitCommit', ''),
                    branch=data['data']['metadata'].get('branch', ''),
                    workflow=data['data']['metadata'].get('workflow', '')
                )
            return cls(
                status=data['status'],
                message=data['message'],
                data=UpdateData(
                    latestSHA=data['data']['latestSHA'],
                    artifactUrl=data['data']['artifactUrl'],
                    updateType=data['data']['updateType'],
                    metadata=metadata
                )
            )
        return cls(
            status=data['status'],
            message=data['message'],
            data=None
        )

class ComponentUpdateClient:
    VERSION_FILE = "/opt/ota-client/current_version.json"
    COMPONENT_PATH = "/root"
    UPDATE_CHECK_INTERVAL = 300  # 5 minutes

    def __init__(self):
        self.device_id = os.getenv("DEVICE_ID", "device-10")
        self.project_name = os.getenv("PROJECT_NAME", "ota_update")
        self.api_base_url = os.getenv("API_URL", "http://13.232.234.162:5000/api")
        self.component_name = os.getenv("COMPONENT_NAME", "oro_ws")
        
        # Ensure required directories exist
        os.makedirs(self.COMPONENT_PATH, exist_ok=True)
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler("/var/log/ota-client.log"),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger("ComponentUpdateClient")
        
        # Setup requests session with retry strategy
        self.session = requests.Session()
        retry_strategy = Retry(
            total=3,
            backoff_factor=1,
            status_forcelist=[429, 500, 502, 503, 504],
        )
        adapter = HTTPAdapter(max_retries=retry_strategy)
        self.session.mount("http://", adapter)
        self.session.mount("https://", adapter)

        # Initialize version
        self.current_version = self._load_current_version()

    def _load_current_version(self) -> Version:
        # Previous implementation remains the same
        try:
            if os.path.exists(self.VERSION_FILE):
                with open(self.VERSION_FILE, 'r') as f:
                    data = json.load(f)
                    metadata = None
                    if 'metadata' in data:
                        metadata = ArtifactMetadata(
                            buildTime=data['metadata'].get('buildTime', ''),
                            gitCommit=data['metadata'].get('gitCommit', ''),
                            branch=data['metadata'].get('branch', ''),
                            workflow=data['metadata'].get('workflow', '')
                        )
                    return Version(version=data.get('version', 'unknown'), metadata=metadata)
            else:
                default_version = Version(version='unknown')
                self._save_current_version(default_version)
                return default_version
        except Exception as e:
            self.logger.error(f"Error loading version file: {e}")
            return Version(version='unknown')

    def _save_current_version(self, version: Version) -> None:
        try:
            data = {
                'version': version.version,
                'metadata': {
                    'buildTime': version.metadata.buildTime,
                    'gitCommit': version.metadata.gitCommit,
                    'branch': version.metadata.branch,
                    'workflow': version.metadata.workflow
                } if version.metadata else None
            }
            with open(self.VERSION_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.logger.error(f"Error saving version file: {e}")
    def _send_build_status(self, status: bool, message: str, commit_sha: str) -> bool:
        """Send build status to the server"""
        try:
            url = f"{self.api_base_url}/updateBuildStatus"
            payload = {
                "deviceId": self.device_id,
                "projectName": self.project_name,
                "componentName": self.component_name,
                "status": status,
                "message": message,
                "commitSHA": commit_sha
            }
            
            response = self.session.post(url, json=payload, timeout=(5, 15))
            response.raise_for_status()
            
            self.logger.info(f"Build status sent successfully: {status}, {message}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send build status: {e}")
            return False

    

    def _find_ros_packages(self, repo_path: str) -> Dict[str, str]:
        """
        Find all ROS packages in the repository and return a mapping of
        package paths to package names
        """
        packages = {}
        for root, _, files in os.walk(repo_path):
            if 'package.xml' in files:
                # Get the package name from package.xml
                import xml.etree.ElementTree as ET
                package_xml = os.path.join(root, 'package.xml')
                try:
                    tree = ET.parse(package_xml)
                    package_name = tree.getroot().find('name').text
                    # Store relative path from repo root
                    rel_path = os.path.relpath(root, repo_path)
                    packages[rel_path] = package_name
                except Exception as e:
                    self.logger.warning(f"Error parsing package.xml at {root}: {e}")
        return packages

    def _get_changed_packages(self, repo: git.Repo, old_commit: str, new_commit: str) -> Set[str]:
        """Get the list of ROS packages that have changed between commits"""
        try:
            # Get all packages in the repository
            packages = self._find_ros_packages(repo.working_dir)
            
            # Get the diff between commits
            diff = repo.git.diff(f"{old_commit}..{new_commit}", name_only=True).split('\n')
            
            # Check which packages have changes
            changed_packages = set()
            for changed_file in diff:
                if changed_file:  # Skip empty lines
                    changed_path = os.path.dirname(changed_file)
                    # Check if this file is under any package directory
                    for package_path, package_name in packages.items():
                        if changed_path.startswith(package_path):
                            changed_packages.add(package_name)
                            break
            
            return changed_packages
        except Exception as e:
            self.logger.error(f"Error determining changed packages: {e}")
            return set()

    def _rollback_to_commit(self, repo: git.Repo, commit_sha: str) -> bool:
        """Rollback to a specific commit"""
        try:
            self.logger.info(f"Rolling back to commit {commit_sha}")
            
            # Hard reset to the specified commit
            repo.git.reset('--hard', commit_sha)
            
            # Clean untracked files
            repo.git.clean('-fd')
            
            self.logger.info(f"Successfully rolled back to commit {commit_sha}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to rollback to commit {commit_sha}: {e}")
            return False

    def _run_colcon_build(self, repo_path: str, packages: Set[str] = None):
        """Run colcon build for specific packages"""
        try:
            build_cmd = ["colcon", "build"]
            if packages:
                # Build only specific packages using package names
                packages_str = " ".join(f"--packages-select {pkg}" for pkg in packages)
                build_cmd.extend(packages_str.split())
            
            # Change to repository directory
            original_dir = os.getcwd()
            os.chdir(repo_path)
            
            self.logger.info(f"Running colcon build command: {' '.join(build_cmd)}")
            
            # Create build and install directories if they don't exist
            os.makedirs("build", exist_ok=True)
            os.makedirs("install", exist_ok=True)
            
            # Execute colcon build
            result = os.system(" ".join(build_cmd))
            
            if result != 0:
                raise Exception(f"Colcon build failed with exit code {result}")
            
            self.logger.info("Colcon build completed successfully")
            
            # Source the setup file after successful build
            setup_file = os.path.join(repo_path, "install", "setup.bash")
            if os.path.exists(setup_file):
                os.system(f"source {setup_file}")
            
            return True, "Build completed successfully"
                
        except Exception as e:
            error_msg = f"Error during colcon build: {e}"
            self.logger.error(error_msg)
            return False, error_msg
        finally:
            # Always return to original directory
            os.chdir(original_dir)

    def _handle_git_conflicts(self, repo: git.Repo) -> bool:
        """
        Handle Git conflicts during pull operations
        Returns True if conflicts were resolved successfully
        """
        try:
            # Check if there are any local changes
            if repo.is_dirty():
                self.logger.warning("Local changes detected. Stashing changes...")
                repo.git.stash()
            
            # Get current branch name
            current_branch = repo.active_branch.name
            
            # Fetch all changes from remote
            self.logger.info("Fetching all changes from remote...")
            repo.git.fetch('--all')
            
            # Try reset to remote branch state
            try:
                self.logger.info(f"Resetting local branch to origin/{current_branch}...")
                repo.git.reset('--hard', f'origin/{current_branch}')
            except git.GitCommandError as e:
                self.logger.warning(f"Hard reset failed: {e}, trying alternative approach...")
                # If reset fails, try to merge with remote changes
                repo.git.merge(f'origin/{current_branch}', '--allow-unrelated-histories')
            
            # Clean untracked files and directories
            repo.git.clean('-fd')
            
            # Reapply stashed changes if any
            if repo.git.stash('list'):
                try:
                    self.logger.info("Reapplying stashed changes...")
                    repo.git.stash('pop')
                except git.GitCommandError as e:
                    self.logger.warning(f"Conflicts occurred while reapplying stashed changes: {e}")
                    # If pop fails, keep changes in stash
                    self.logger.info("Keeping original changes in stash. Manual intervention may be required.")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error handling git conflicts: {e}")
            return False

    def _safe_pull(self, repo: git.Repo) -> Tuple[bool, str]:
        """
        Safely pull changes from remote repository
        Returns (success: bool, message: str)
        """
        try:
            # Store current branch name
            current_branch = repo.active_branch.name
            
            # Configure git to allow unrelated histories
            repo.git.config('pull.rebase', 'false')
            repo.git.config('pull.ff', 'false')  # Changed from 'only' to 'false'
            
            try:
                # Try regular pull first
                self.logger.info("Attempting regular pull...")
                repo.git.pull()
                return True, "Pull completed successfully"
                
            except git.GitCommandError as e:
                self.logger.warning(f"Regular pull failed: {e}")
                
                if "Not possible to fast-forward" in str(e) or "refusing to merge unrelated histories" in str(e):
                    self.logger.info("Fast-forward failed, attempting conflict resolution...")
                    if self._handle_git_conflicts(repo):
                        # Verify we're up to date
                        local_commit = repo.head.commit
                        remote_commit = repo.remotes.origin.refs[current_branch].commit
                        
                        if local_commit == remote_commit:
                            return True, "Repository updated successfully after resolving conflicts"
                        else:
                            # Final attempt with merge
                            self.logger.info("Attempting merge with unrelated histories...")
                            repo.git.pull('--allow-unrelated-histories')
                            return True, "Repository updated successfully with unrelated histories merge"
                    else:
                        return False, "Failed to resolve git conflicts"
                else:
                    raise e
                    
        except git.GitCommandError as e:
            return False, f"Git command error: {str(e)}"
        except Exception as e:
            return False, f"Unexpected error during pull: {str(e)}"

    def _check_and_update_repository(self, update_info: UpdateResponse) -> None:
        try:
            if not update_info.data:
                self.logger.error("Update info contains no data")
                return

            component_dir = Path(self.COMPONENT_PATH) / self.component_name
            repo_path = str(component_dir)

            try:
                # Try to open existing repository
                repo = git.Repo(repo_path)
                self.logger.info("Found existing repository")
                
                # Store the current commit before update
                old_commit = repo.head.commit.hexsha
                
                # Update remote URL if needed
                if repo.remotes.origin.url != update_info.data.artifactUrl:
                    self.logger.info("Updating remote URL")
                    repo.remotes.origin.set_url(update_info.data.artifactUrl)
                
                # Fetch latest changes
                self.logger.info("Fetching updates...")
                origin = repo.remotes.origin
                origin.fetch()

                # Check if we're behind the remote
                local_commit = repo.head.commit
                remote_commit = origin.refs[repo.active_branch.name].commit

                if local_commit != remote_commit:
                    self.logger.info("Updates found. Pulling changes...")
                    success, message = self._safe_pull(repo)
                    
                    if success:
                        self.logger.info(message)
                        new_commit = repo.head.commit.hexsha
                        
                        # Get list of changed packages
                        changed_packages = self._get_changed_packages(repo, old_commit, new_commit)
                        if changed_packages:
                            self.logger.info(f"Changed packages detected: {changed_packages}")
                            # Run colcon build only for changed packages
                            build_success, build_message = self._run_colcon_build(repo_path, changed_packages)
                            
                            # Send build status to server
                            self._send_build_status(build_success, build_message, new_commit)
                            
                            if not build_success:
                                self.logger.warning("Build failed, rolling back to previous commit")
                                if self._rollback_to_commit(repo, old_commit):
                                    # Run build again with previous commit
                                    rollback_build_success, rollback_build_message = self._run_colcon_build(repo_path)
                                    if not rollback_build_success:
                                        self.logger.error("Build failed even after rollback!")
                                    return  # Exit without updating version
                        else:
                            self.logger.info("No ROS packages were modified in this update")
                            self._send_build_status(True, "No packages required rebuilding", new_commit)
                    else:
                        raise Exception(f"Failed to pull updates: {message}")
                else:
                    self.logger.info("Repository is up to date")

            except git.exc.InvalidGitRepositoryError:
                self.logger.info(f"No valid repository found at {repo_path}. Cloning fresh...")
                if component_dir.exists():
                    import shutil
                    shutil.rmtree(component_dir)
                repo = git.Repo.clone_from(
                    update_info.data.artifactUrl,
                    repo_path,
                    branch='main'
                )
                self.logger.info("Repository cloned successfully")
                # For fresh clone, build all packages
                build_success, build_message = self._run_colcon_build(repo_path)
                self._send_build_status(build_success, build_message, repo.head.commit.hexsha)
                
                if not build_success:
                    raise Exception("Initial build failed after fresh clone")

            # Update version information
            if update_info.data.metadata:
                new_version = Version(
                    version=update_info.data.latestSHA,
                    metadata=update_info.data.metadata
                )
                self._save_current_version(new_version)
                self.current_version = new_version

        except Exception as e:
            self.logger.error(f"Failed to update repository: {e}")
            self._send_build_status(False, str(e), update_info.data.latestSHA if update_info.data else "unknown")
            raise

    def check_for_updates(self) -> None:
        try:
            url = f"{self.api_base_url}/checkForUpdate/{self.device_id}/{self.project_name}/{self.current_version.version}"
            self.logger.info(f"Checking for updates: {url}")
            
            response = self.session.get(url, timeout=(5, 15))
            response.raise_for_status()
            
            update_info = UpdateResponse.from_dict(response.json())
            self.logger.info(f"Received update info: {update_info}")
            
            if update_info.status and update_info.data and update_info.data.updateType == "component-update":
                self.logger.info("Component update available, proceeding to check and update...")
                self._check_and_update_repository(update_info)
            else:
                self.logger.info(f"No component update available: {update_info.message}")
                
        except Exception as e:
            self.logger.error(f"Error checking for updates: {e}")
        finally:
            time.sleep(self.UPDATE_CHECK_INTERVAL)

    def run(self) -> None:
        self.logger.info("Starting OTA component update client...")
        
        import signal
        import sys
        
        # Flag to control the main loop
        self._running = True
        
        def signal_handler(signum, frame):
            """Handle shutdown signals gracefully"""
            self.logger.info(f"Received signal {signum}. Starting graceful shutdown...")
            self._running = False
        
        # Register signal handlers
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        try:
            while self._running:
                try:
                    self.check_for_updates()
                except KeyboardInterrupt:
                    self.logger.info("Received keyboard interrupt")
                    break
                except Exception as e:
                    self.logger.error(f"Error in main update loop: {e}")
                    # Wait before retrying to avoid rapid failure loops
                    time.sleep(60)
        finally:
            self._cleanup()
    
    def _cleanup(self) -> None:
        """Perform cleanup operations before shutdown"""
        try:
            self.logger.info("Performing cleanup operations...")
            
            # Close the requests session
            try:
                self.session.close()
            except Exception as e:
                self.logger.error(f"Error closing requests session: {e}")
            
            # Save final version state
            try:
                self._save_current_version(self.current_version)
            except Exception as e:
                self.logger.error(f"Error saving final version state: {e}")
            
            self.logger.info("Cleanup completed successfully")
            
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")

def main():
    """Main entry point for the OTA component update client"""
    try:
        client = ComponentUpdateClient()
        client.run()
    except Exception as e:
        logging.error(f"Fatal error in main: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()