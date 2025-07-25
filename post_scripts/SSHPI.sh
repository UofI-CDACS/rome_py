#!/bin/bash

# Users and hosts (passwords no longer needed)
declare -A pi_credentials=(
    [172.23.254.18]="rospi"
    [172.23.254.22]="rospi"
    [172.23.254.23]="rospi"
    [172.23.254.24]="rospi"
)
declare -A pi_node_names=(
    [172.23.254.18]="rospi_1"
    [172.23.254.22]="rospi_2"
    [172.23.254.23]="rospi_3"
    [172.23.254.24]="rospi_4"
)
WORKSPACE_FOLDER="${WORKSPACE_FOLDER:-$HOME/Desktop/test_ws}"
BRANCH_NAME="${BRANCH_NAME:-post-develop-feature-preprocessing}"
DDS_CONFIG="${DDS_CONFIG:-cyclonedx.xml}"
PULL_GITHUB="${PULL_GITHUB:-TRUE}"
BUILD_WORKSPACE="${BUILD_WORKSPACE:-TRUE}"
SSH_PIS="${SSH_PIS:-TRUE}"
FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" --text="Enter the Stations Parameters" \
    --field="Workspace Folder":TXT "$WORKSPACE_FOLDER" \
    --field="Branch Name":TXT "$BRANCH_NAME" \
    --field="DDS Config:CB" "cyclone_source.sh!fast_source.sh" "$DDS_CONFIG" \
    --field="Pull from GitHub":CHK "$PULL_GITHUB" \
    --field="Build workspace":CHK "$BUILD_WORKSPACE" \
    --field="SSH into PIs":CHK "$SSH_PIS" \
    --button="Install:0" --button="Cancel:1" \
    --separator=","
)
YAD_EXIT_CODE=$?

if [ $YAD_EXIT_CODE -ne 0 ]; then
    exit 0
fi

if [ -z "$FORM_OUTPUT" ]; then
    yad --error --text="All required fields must be filled."
    exit 1
fi

IFS=',' read -r WORKSPACE_FOLDER BRANCH_NAME DDS_CONFIG PULL_GITHUB BUILD_WORKSPACE SSH_PIS <<< "$FORM_OUTPUT"

# Option 1: Pull from GitHub
if [ "$PULL_GITHUB" = "TRUE" ]; then
    echo "=== Pulling from GitHub ==="
    
    # Run locally
    cd "$WORKSPACE_FOLDER/src"
    if [ ! -d post ]; then
        git clone https://github.com/UofI-CDACS/rome_py.git post
    fi
    cd post
    git fetch --all

    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    if [ "$CURRENT_BRANCH" != "$BRANCH_NAME" ]; then
        git checkout $BRANCH_NAME
    fi
    if [ ! git diff --quiet origin/$BRANCH_NAME ]; then
        git reset --hard origin/$BRANCH_NAME
    fi
    
    # Run on remote PIs
    for ip in "${!pi_credentials[@]}"; do
        creds="${pi_credentials[$ip]}"
        username="${creds%%:*}"
        ssh -A -tt -o StrictHostKeyChecking=no "${username}@${ip}" bash -c "'
            cd $WORKSPACE_FOLDER/src
            if [ ! -d post ]; then
                git clone https://github.com/UofI-CDACS/rome_py.git post
            fi
            cd post
            git fetch --all
            CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
            if [ "$CURRENT_BRANCH" != "$BRANCH_NAME" ]; then
                git checkout $BRANCH_NAME
            fi
            if [ ! git diff --quiet origin/$BRANCH_NAME ]; then
                git reset --hard origin/$BRANCH_NAME
            fi
        '"
        echo "Git pull completed on $ip"
    done
fi

# Option 2: Build workspace
if [ "$BUILD_WORKSPACE" = "TRUE" ]; then
    echo "=== Building workspace ==="
    
    # Build locally
    cd $WORKSPACE_FOLDER
    rm -rf build install log
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    
    # Build on remote PIs
    for ip in "${!pi_credentials[@]}"; do
        creds="${pi_credentials[$ip]}"
        username="${creds%%:*}"
        ssh -A -tt -o StrictHostKeyChecking=no "${username}@${ip}" bash -c "'
            cd $WORKSPACE_FOLDER
            source /opt/ros/jazzy/setup.bash
            rm -rf build install log
            colcon build --symlink-install
        '"
        echo "Build completed on $ip"
    done
fi

# Option 3: SSH into PIs
if [ "$SSH_PIS" = "TRUE" ]; then
    echo "=== SSHing into PIs ==="
    
    # Loop through pi_credentials associative array
    for HOST in "${!pi_credentials[@]}"; do
        USER_PASS="${pi_credentials[$HOST]}"
        USER="${USER_PASS%%:*}"
        PASSWORD="${USER_PASS##*:}"
        USER_AT_HOST="${USER}@${HOST}"
        NODE_NAME="${pi_node_names[$HOST]}"
        REMOTE_COMMAND="
            cd $WORKSPACE_FOLDER
            chmod -R +rwx .
            source \"./src/post/post_scripts/$DDS_CONFIG\"
            source \"$WORKSPACE_FOLDER/install/setup.bash\"
            ros2 run post_core station --name ${NODE_NAME} --type default
        "
        gnome-terminal --tab -- bash -c "echo 'Connecting to $USER_AT_HOST...'; ssh -tt -o StrictHostKeyChecking=no $USER_AT_HOST '$REMOTE_COMMAND; bash'; exec bash"
    done
fi