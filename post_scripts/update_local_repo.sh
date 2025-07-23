#!/bin/bash
#-e Exit if there is a non-zero exit status
#-u All un-defined variables are errors
#-o pipefail Pipe exit status from the failing line
set -euo pipefail

# Defaults
BARE_REPO_DIR="/\$HOME/Desktop/rome_py_bare_repo"
GITHUB_REPO="git@github.com:UofI-CDACS/rome_py.git"
BRANCH="main"
COMMIT=""
FORCE_REINIT=false

usage() {
  cat <<EOF
Usage: $0 [-b bare_repo_dir] [-r github_repo] [-B branch] [-c commit]

  -b  Path to bare repo directory (default: $BARE_REPO_DIR)
  -r  GitHub repo URL (default: $GITHUB_REPO)
  -B  Branch to fetch (default: $BRANCH)
  -c  Specific commit hash (optional)
  -f  Force reinitialization of the bare repo if it already exists (deletes the existing repo)

Example:
  $0 -b $BARE_REPO_DIR -r $GITHUB_REPO -B post-develop -c abc123
EOF
  exit 1
}

# Parse arguments
while getopts ":b:r:B:c:h" opt; do
  case $opt in
  b) BARE_REPO_DIR="$OPTARG" ;;
  r) GITHUB_REPO="$OPTARG" ;;
  B) BRANCH="$OPTARG" ;;
  c) COMMIT="$OPTARG" ;;
  f) FORCE_REINIT=true ;;
  h) usage ;;
  *) usage ;;
  esac
done

# Step 1: Check and create bare repo if missing or force reinit
if [ -d "$BARE_REPO_DIR" ]; then
  if ! git --git-dir="$BARE_REPO_DIR" rev-parse --is-bare-repository &>/dev/null; then
    if [ "$FORCE_REINIT" = true ]; then
      echo "Force reinitializing bare repo at $BARE_REPO_DIR"
      rm -rf "$BARE_REPO_DIR"
      git init --bare "$BARE_REPO_DIR"
    else
      echo "Error: '$BARE_REPO_DIR' exists but is not a valid bare Git repository."
      echo "       Use -f to force reinitialization."
      exit 1
    fi
  fi
else
  echo "Creating bare repo at $BARE_REPO_DIR"
  git init --bare "$BARE_REPO_DIR"
fi

cd "$BARE_REPO_DIR"

# Step 2: Add remote if missing
if ! git remote get-url origin &>/dev/null; then
  git remote add origin "$GITHUB_REPO"
fi

# Step 3: Fetch branch
echo "Fetching branch '$BRANCH' from origin..."
git fetch origin "$BRANCH"

# Step 4: Determine target commit
TARGET_COMMIT="$COMMIT"
if [ -z "$TARGET_COMMIT" ]; then
  TARGET_COMMIT="$(git rev-parse origin/$BRANCH)"
  echo "No commit specified. Using latest on origin/$BRANCH: $TARGET_COMMIT"
else
  if ! git rev-parse --verify "$TARGET_COMMIT" &>/dev/null; then
    echo "Error: commit '$TARGET_COMMIT' not found after fetch." >&2
    exit 1
  fi
  echo "Using specified commit: $TARGET_COMMIT"
fi

# Step 5: Update HEAD
echo "Updating HEAD to $TARGET_COMMIT"
echo "$TARGET_COMMIT" >HEAD

echo "Bare repo updated successfully."
