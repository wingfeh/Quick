# GitHub Management Scripts

This directory contains scripts to help you manage your GitHub repository with Personal Access Token authentication.

## Scripts Available

### 1. `quick_push.sh` - Fast Git Push
- Quick one-command push to GitHub
- Automatic timestamped commit messages
- Usage: `./quick_push.sh [commit_message]`

### 2. `push_to_github.sh` - Interactive Push
- Full-featured interactive git push
- Custom commit messages with confirmation
- Colorized output and error handling
- Usage: `./push_to_github.sh [commit_message]`

### 3. `git_manager.sh` - Complete Git Management
- Multiple git operations (push, pull, status, log, diff)
- Command-line interface
- Usage: `./git_manager.sh [command] [options]`

### 4. `setup_github_token.sh` - Token Configuration
- Securely set up your GitHub Personal Access Token
- Creates `~/.github_token` file with proper permissions
- Usage: `./setup_github_token.sh`

## Setup Instructions

1. **Set up your token** (choose one method):
   ```bash
   # Method 1: Use setup script
   ./setup_github_token.sh
   
   # Method 2: Environment variable
   export GITHUB_TOKEN='your_token_here'
   
   # Method 3: Create token file manually
   echo 'your_token_here' > ~/.github_token
   chmod 600 ~/.github_token
   ```

2. **Start using the scripts**:
   ```bash
   # Quick push with auto message
   ./quick_push.sh
   
   # Quick push with custom message
   ./quick_push.sh "Fix sensor issues"
   
   # Interactive push
   ./push_to_github.sh
   
   # Check repository status
   ./git_manager.sh status
   ```

## Security

- Personal Access Token is stored securely in `~/.github_token`
- File permissions set to 600 (user read/write only)
- Token is never committed to the repository
- Scripts read token from environment or file dynamically

## Requirements

- Git installed and configured
- Valid GitHub Personal Access Token with repository access
- Bash shell environment
