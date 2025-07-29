#!/bin/bash
set -euo pipefail

# Defaults
BARE_REPO_DIR="/home/rospi/Desktop/rome_py_bare_repo"
WORKING_REPO_DIR="/home/rospi/Desktop/rome_py"
#HOSTS=("rospi-0-desktop" "rospi-0-desktop" "rospi-2-desktop" "rospi-3-desktop" "rospi-4-desktop")
HOSTS=("172.23.254.20" "172.23.254.24" "172.23.254.22" "172.23.254.23" "172.23.254.18")
BRANCH="main"
FORCE_CLONE=false

usage() {
  cat <<EOF
Usage: $0 [-b bare_repo_dir] [-w working_repo_dir] [-h hosts_csv] [-B branch] [-f]

  -b  Path to bare repo directory on this machine (default: $BARE_REPO_DIR)
  -w  Path to working repo directory on target machines (default: $WORKING_REPO_DIR)
  -h  Comma-separated list of hosts (default: ${HOSTS[*]})
  -B  Branch to checkout/update (default: $BRANCH)
  -f  Force reclone if existing working repo is invalid

Example:
  $0 -b $BARE_REPO_DIR -w $WORKING_REPO_DIR -h rospi-0,rospi-1 -B post-develop -f
EOF
  exit 1
}

while getopts ":b:w:h:B:f" opt; do
  case $opt in
  b) BARE_REPO_DIR="$OPTARG" ;;
  w) WORKING_REPO_DIR="$OPTARG" ;;
  h) IFS=',' read -r -a HOSTS <<<"$OPTARG" ;;
  B) BRANCH="$OPTARG" ;;
  f) FORCE_CLONE=true ;;
  *) usage ;;
  esac
done

for HOST in "${HOSTS[@]}"; do
  echo "==> Processing $HOST..."

  ssh -A "rospi@$HOST" bash -c "'
    set -e
    REPO=\"$WORKING_REPO_DIR\"
    BARE_REPO_SSH=\"ssh://rospi@172.23.254.20$BARE_REPO_DIR\"
    BRANCH=\"$BRANCH\"
    FORCE_CLONE=$FORCE_CLONE

    if [ ! -d \"\$REPO\" ]; then
      echo \"Cloning repo on $HOST...\"
      git clone \"\$BARE_REPO_SSH\" \"\$REPO\"
    else
      if git -C \"\$REPO\" rev-parse --is-inside-work-tree &>/dev/null && \
         [ \"\$(git -C \"\$REPO\" remote get-url origin)\" = \"\$BARE_REPO_SSH\" ]; then
        echo \"Updating existing repo on $HOST...\"
        cd \"\$REPO\"
        git fetch origin
        git reset --hard origin/\$BRANCH
      else
        if [ \"\$FORCE_CLONE\" = true ]; then
          echo \"Existing directory invalid; removing and recloning on $HOST...\"
          rm -rf \"\$REPO\"
          git clone \"\$BARE_REPO_SSH\" \"\$REPO\"
        else
          echo \"Warning: Existing directory on $HOST is not a valid clone of bare repo.\"
          echo \"Skipping clone/update for $HOST. Use -f to force reclone.\"
        fi
      fi
    fi
  '"
done

echo "All done."
