#!/bin/bash
set -euo pipefail

validate_branch() {
  local branch=$1
  # Only allow letters, numbers, dots, dashes, underscores, slashes
  if [[ ! $branch =~ ^[a-zA-Z0-9._/-]+$ ]]; then
    echo "Invalid characters in branch name: $branch"
    exit 1
  fi
}

# Returns git commands as a string to be run locally or remotely
git_sync_cmds() {
  local branch=$1
  local force=$2 # "true" or "false"

  if [[ "$force" == "true" ]]; then
    cat <<EOF
    #In workspace/src
    PROJECT_NAME="post"
    PROJECT_DIR="\${HOME}/Desktop/test_ws/src"
    REMOTE="https://github.com/UofI-CDACS/rome_py.git"
    cd "\${PROJECT_DIR}"
    # In project directory
    if [ ! -d "\${PROJECT_NAME}" ]; then
      git clone "\${REMOTE}" "\${PROJECT_NAME}"
    fi
    
    cd "\${PROJECT_NAME}"
    git fetch --prune origin
    
    # Switch to the branch (track if it doesn't exist)
    if git show-ref --verify --quiet "refs/heads/${branch}"; then
      git switch "${branch}"
    else
      git switch --track "origin/${branch}"
    fi
    
    # Make sure the local branch exactly matches the remote
    git reset --hard "origin/${branch}"
    git clean -fd
EOF
  else
    cat <<EOF
set -e
if [ ! -d post ]; then
  git clone https://github.com/UofI-CDACS/rome_py.git post
fi
cd post
git fetch --all
current_branch=\$(git rev-parse --abbrev-ref HEAD)
if [ "\$current_branch" != "$branch" ]; then
  git checkout "$branch"
fi
git pull origin "$branch"
EOF
  fi
}

sync_git_repo() {
  local path=$1
  local branch=$2
  local force=$3 # "true" or "false"

  validate_branch "$branch"

  cd "$path"
  eval "$(git_sync_cmds "$branch" "$force")"
}

sync_git_repo_ssh() {
  local label=$1
  local ip=$2
  local user=$3
  local path=$4
  local branch=$5
  local force=$6 # "true" or "false"

  validate_branch "$branch"

  if ! declare -f ssh_run >/dev/null; then
    echo "Error: ssh_run function not defined. Please source ssh_common.sh first."
    exit 1
  fi

  echo "[$label] Syncing repo..."
  ssh_run "$ip" "$user" "$(git_sync_cmds "$branch" "$force")"
}
