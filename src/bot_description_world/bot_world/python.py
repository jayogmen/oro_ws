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
import subprocess
import traceback

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
    UPDATE_CHECK_INTERVAL = 300 

    def __init__(self):
        self.device_id = os.getenv("DEVICE_ID", "device-10")
        self.project_name = os.getenv("PROJECT_NAME", "ota_update")
        self.api_base_url = os.getenv("API_URL", "http://13.232.234.162:5000/api")
        self.component_name = os.getenv("COMPONENT_NAME", "oro_git_ws")
        
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
        try:
            if os.path.exists(self.VERSION_FILE):
                with open(self.VERSION_FILE, 'r') as f:
                    data = json.load(f)
                    metadata = None
                    if data.get('metadata'):
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
                    'buildTime': version.metadata.buildTime if version.metadata else '',
                    'gitCommit': version.metadata.gitCommit if version.metadata else '',
                    'branch': version.metadata.branch if version.metadata else '',
                    'workflow': version.metadata.workflow if version.metadata else ''
                }
            }
            with open(self.VERSION_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.logger.error(f"Error saving version file: {e}")

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

    def _send_build_status(self, status: bool, message: str, commit_sha: str) -> bool:
        """
        Send build status to the server with enhanced logging and error handling
        """
        try:
            url = f"{self.api_base_url}/updateBuildStatus"
            
            # Construct payload
            payload = {
                "deviceId": self.device_id,
                "projectName": self.project_name,
                "componentName": self.component_name,
                "status": status,
                "message": message,
                "commitSHA": commit_sha
            }
            
            # Detailed logging before sending request
            self.logger.info(f"Attempting to send build status to {url}")
            self.logger.debug(f"Build status payload: {payload}")
            
            # Log connection attempt
            self.logger.debug("Initiating POST request to server...")
            
            # Send POST request with timeout
            response = self.session.post(url, json=payload, timeout=(5, 15))
            
            # Log response details
            self.logger.debug(f"Server Response - Status Code: {response.status_code}")
            self.logger.debug(f"Server Response - Content: {response.text}")
            
            # Check response status
            response.raise_for_status()
            
            self.logger.info(f"✓ Build status successfully sent to server")
            self.logger.debug(f"Status: {status}, Message: {message}, Commit: {commit_sha}")
            return True
            
        except requests.exceptions.ConnectionError as e:
            self.logger.error(f"❌ Connection error while sending build status: {e}")
            self.logger.debug(f"Connection details - URL: {url}, Timeout: (5, 15)")
            return False
            
        except requests.exceptions.Timeout as e:
            self.logger.error(f"❌ Timeout while sending build status: {e}")
            self.logger.debug("Request timed out after 15 seconds")
            return False
            
        except requests.exceptions.RequestException as e:
            self.logger.error(f"❌ Failed to send build status: {e}")
            self.logger.debug(f"Full error traceback: {traceback.format_exc()}")
            return False


    def _run_colcon_build(self, repo_path: str, packages: Set[str] = None, commit_sha: str = None) -> Tuple[bool, str]:
        """
        Run colcon build for specific packages with enhanced logging and error handling
        """
        original_dir = os.getcwd()
        build_process = None
        
        try:
            # Validate inputs
            if not os.path.exists(repo_path):
                raise ValueError(f"Repository path does not exist: {repo_path}")
                
            if packages and not all(isinstance(pkg, str) for pkg in packages):
                raise ValueError("All package names must be strings")
            
            # Construct build command
            build_cmd = ["colcon", "build"]
            if packages:
                build_cmd.append("--packages-select")
                build_cmd.extend(packages)
            
            cmd_str = " ".join(build_cmd)
            
            # Log build initiation
            self.logger.info(f"🔧 Initiating colcon build in: {repo_path}")
            self.logger.info(f"Build command: {cmd_str}")
            
            # Change directory and create necessary folders
            os.chdir(repo_path)
            
            # Create directories with logging
            for dir_name in ["build", "install"]:
                dir_path = os.path.join(repo_path, dir_name)
                if not os.path.exists(dir_path):
                    self.logger.debug(f"Creating directory: {dir_path}")
                    os.makedirs(dir_path)
            
            # Execute build with detailed output capture
            self.logger.info("Starting build process...")
            build_process = subprocess.run(
                cmd_str,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding='utf-8'
            )
            
            # Process build result
            if build_process.returncode != 0:
                error_msg = (
                    f"❌ Colcon build failed with exit code {build_process.returncode}\n"
                    f"Command output:\n{build_process.stdout}\n"
                    f"Error output:\n{build_process.stderr}"
                )
                self.logger.error(error_msg)
                
                if commit_sha:
                    self.logger.info("Sending build failure status to server...")
                    status_sent = self._send_build_status(False, error_msg[:500], commit_sha)  # Truncate message if too long
                    self.logger.debug(f"Build failure status sent: {status_sent}")
                
                return False, error_msg
            
            # Handle successful build
            self.logger.info("✓ Colcon build completed successfully")
            
            # Source setup file
            setup_file = os.path.join(repo_path, "install", "setup.bash")
            if os.path.exists(setup_file):
                self.logger.debug(f"Sourcing setup file: {setup_file}")
                try:
                    subprocess.run(
                        f"bash -c 'source {setup_file}'",
                        shell=True,
                        check=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    self.logger.debug("Setup file sourced successfully")
                except subprocess.CalledProcessError as e:
                    self.logger.warning(f"Failed to source setup file: {e}")
            
            # Send success status
            if commit_sha:
                self.logger.info("Sending build success status to server...")
                success_msg = "Build completed successfully"
                status_sent = self._send_build_status(True, success_msg, commit_sha)
                self.logger.debug(f"Build success status sent: {status_sent}")
            
            return True, "Build completed successfully"
            
        except Exception as e:
            error_msg = f"❌ Error during colcon build: {str(e)}\n{traceback.format_exc()}"
            self.logger.error(error_msg)
            
            if commit_sha:
                self.logger.info("Sending build error status to server...")
                status_sent = self._send_build_status(False, str(e), commit_sha)
                self.logger.debug(f"Build error status sent: {status_sent}")
            
            return False, error_msg
            
        finally:
            # Cleanup
            if original_dir:
                self.logger.debug(f"Returning to original directory: {original_dir}")
                os.chdir(original_dir)

    def _find_ros_packages(self, repo_path: str) -> Dict[str, str]:
        """Find all ROS packages in the repository"""
        packages = {}
        for root, _, files in os.walk(repo_path):
            if 'package.xml' in files:
                import xml.etree.ElementTree as ET
                package_xml = os.path.join(root, 'package.xml')
                try:
                    tree = ET.parse(package_xml)
                    package_name = tree.getroot().find('name').text
                    rel_path = os.path.relpath(root, repo_path)
                    packages[rel_path] = package_name
                except Exception as e:
                    self.logger.warning(f"Error parsing package.xml at {root}: {e}")
        return packages

    def _get_changed_packages(self, repo: git.Repo, old_commit: str, new_commit: str) -> Set[str]:
        """Get the list of ROS packages that have changed between commits"""
        try:
            packages = self._find_ros_packages(repo.working_dir)
            diff = repo.git.diff(f"{old_commit}..{new_commit}", name_only=True).split('\n')
            
            changed_packages = set()
            for changed_file in diff:
                if changed_file:
                    changed_path = os.path.dirname(changed_file)
                    for package_path, package_name in packages.items():
                        if changed_path.startswith(package_path):
                            changed_packages.add(package_name)
                            break
            
            return changed_packages
        except Exception as e:
            self.logger.error(f"Error determining changed packages: {e}")
            return set()

    def _handle_git_conflicts(self, repo: git.Repo) -> bool:
        """Handle Git conflicts during pull operations"""
        try:
            if repo.is_dirty():
                self.logger.warning("Local changes detected. Stashing changes...")
                repo.git.stash()
            
            current_branch = repo.active_branch.name
            
            self.logger.info("Fetching all changes from remote...")
            repo.git.fetch('--all')
            
            try:
                self.logger.info(f"Resetting local branch to origin/{current_branch}...")
                repo.git.reset('--hard', f'origin/{current_branch}')
            except git.GitCommandError as e:
                self.logger.warning(f"Hard reset failed: {e}, trying alternative approach...")
                repo.git.merge(f'origin/{current_branch}', '--allow-unrelated-histories')
            
            repo.git.clean('-fd')
            
            if repo.git.stash('list'):
                try:
                    self.logger.info("Reapplying stashed changes...")
                    repo.git.stash('pop')
                except git.GitCommandError as e:
                    self.logger.warning(f"Conflicts occurred while reapplying stashed changes: {e}")
                    self.logger.info("Keeping original changes in stash. Manual intervention may be required.")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error handling git conflicts: {e}")
            return False

    def _safe_pull(self, repo: git.Repo) -> Tuple[bool, str]:
        """Safely pull changes from remote repository"""
        try:
            current_branch = repo.active_branch.name
            
            repo.git.config('pull.rebase', 'false')
            repo.git.config('pull.ff', 'false')
            
            try:
                self.logger.info("Attempting regular pull...")
                repo.git.pull()
                return True, "Pull completed successfully"
                
            except git.GitCommandError as e:
                self.logger.warning(f"Regular pull failed: {e}")
                
                if "Not possible to fast-forward" in str(e) or "refusing to merge unrelated histories" in str(e):
                    self.logger.info("Fast-forward failed, attempting conflict resolution...")
                    if self._handle_git_conflicts(repo):
                        local_commit = repo.head.commit
                        remote_commit = repo.remotes.origin.refs[current_branch].commit
                        
                        if local_commit == remote_commit:
                            return True, "Repository updated successfully after resolving conflicts"
                        else:
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
        
        self._running = True
        
        def signal_handler(signum, frame):
            # Signal handler to stop the running loop
            self._running = False
        
        # Register signal handlers for graceful shutdown on SIGTERM and SIGINT
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        
        try:
            while self._running:
                try:
                    self.check_for_updates()
                except KeyboardInterrupt:
                    # Explicitly handle Control + C
                    self.logger.info("Received keyboard interrupt. Shutting down.")
                    break
                except Exception as e:
                    self.logger.error(f"Error in main update loop: {e}")
                    time.sleep(60)
        finally:
            self._cleanup()


    
    def _cleanup(self) -> None:
        try:
            self.logger.info("Performing cleanup operations...")
            
            try:
                self.session.close()
            except Exception as e:
                self.logger.error(f"Error closing requests session: {e}")
            
            try:
                self._save_current_version(self.current_version)
            except Exception as e:
                self.logger.error(f"Error saving current version during cleanup: {e}")
            
            self.logger.info("Cleanup completed")
            
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")

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
                old_commit = repo.head.commit
                
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
                    
                    # First, stash any local changes
                    if repo.is_dirty():
                        self.logger.info("Stashing local changes...")
                        repo.git.stash()

                    try:
                        # Configure merge strategy to favor remote changes
                        with repo.config_writer() as git_config:
                            git_config.set_value('pull', 'rebase', 'false')
                            git_config.set_value('merge', 'ff', 'only')

                        # Pull with explicit merge strategy
                        repo.git.pull('origin', repo.active_branch.name, '--strategy=recursive', '--strategy-option=theirs')
                        
                        self.logger.info("Updates downloaded successfully")
                        
                        # Try to apply stashed changes if any were stashed
                        if repo.git.stash('list'):
                            try:
                                self.logger.info("Attempting to reapply local changes...")
                                repo.git.stash('pop')
                            except git.GitCommandError as e:
                                self.logger.warning(f"Could not reapply local changes: {e}")
                                # Create a backup branch with the stashed changes
                                backup_branch = f"backup-{int(time.time())}"
                                repo.git.stash('branch', backup_branch)
                                self.logger.info(f"Local changes saved in branch: {backup_branch}")
                        
                        # Get list of changed packages
                        changed_packages = self._get_changed_packages(repo, old_commit, repo.head.commit)
                        if changed_packages:
                            self.logger.info(f"Changed packages detected: {changed_packages}")
                            # Run colcon build only for changed packages
                            self._run_colcon_build(repo_path, changed_packages)
                        else:
                            self.logger.info("No ROS packages were modified in this update")
                    
                    except git.GitCommandError as e:
                        self.logger.error(f"Git pull failed: {e}")
                        # If pull fails, try to reset to a clean state
                        repo.git.reset('--hard', 'origin/' + repo.active_branch.name)
                        self.logger.info("Reset to remote branch state")
                        
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
                self._run_colcon_build(repo_path)

            # Update version information
            if update_info.data.metadata:
                new_version = Version(
                    version=update_info.data.latestSHA,
                    metadata=update_info.data.metadata
                )
                self._save_current_version(new_version)
                self.current_version = new_version

            # Run post-update script if it exists
            post_update_script = component_dir / "scripts" / "post_update.sh"
            if post_update_script.exists():
                self.logger.info("Running post-update script")
                os.chmod(post_update_script, 0o755)
                result = os.system(str(post_update_script))
                if result != 0:
                    raise Exception(f"Post-update script failed with exit code {result}")

        except Exception as e:
            self.logger.error(f"Failed to update repository: {e}")
            raise

if __name__ == "__main__":
    client = ComponentUpdateClient()
    client.run()
