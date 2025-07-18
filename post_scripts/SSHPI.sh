#!/bin/bash
if ! command -v yad &> /dev/null; then
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [[ "$ID" == "ubuntu" || "$ID_LIKE" == *"debian"* ]]; then
            sudo apt-get update && sudo apt-get install -y yad
        elif [[ "$ID" == "cachyos" || "$ID_LIKE" == *"arch"* ]]; then
            sudo pacman -Sy --noconfirm yad
        else
            echo "Unsupported OS. Please install 'yad' manually."
            exit 1
        fi
    else
        echo "Cannot detect OS. Please install 'yad' manually."
        exit 1
    fi
fi
# Users, hosts, and passwords
declare -A pi_credentials=(
    [172.23.254.18]="rospi:rospi"
    [172.23.254.22]="rospi:rospi"
    [172.23.254.23]="rospi:rospi"
    [172.23.254.24]="rospi:rospi"
)
declare -A pi_node_names=(
    [172.23.254.18]="rospi_1"
    [172.23.254.22]="rospi_2"
    [172.23.254.23]="rospi_3"
    [172.23.254.24]="rospi_4"
)
WORKSPACE_FOLDER="${WORKSPACE_FOLDER:-$HOME/Desktop/test_ws}"
BRANCH_NAME="${BRANCH_NAME:-main}"
DDS_CONFIG="${DDS_CONFIG:-cyclonedds.xml}"
FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" --text="Enter the Stations Parameters" \
    --field="Workspace Folder":TXT "$WORKSPACE_FOLDER" \
    --field="Branch Name":TXT "$BRANCH_NAME" \
    --field="DDS Config:CB" "cyclonedds.xml!fastdds.xml" "$DDS_CONFIG" \
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
IFS=',' read -r WORKSPACE_FOLDER BRANCH_NAME DDS_CONFIG <<< "$FORM_OUTPUT"
# Run the same steps locally
cd "$WORKSPACE_FOLDER/src"
if [ ! -d post ]; then
    git clone https://github.com/UofI-CDACS/rome_py.git post
fi
cd post
git checkout $BRANCH_NAME
echo "BRANCH_NAME is: '$BRANCH_NAME'"

GIT_OUTPUT=$(git pull)
if [[ "$GIT_OUTPUT" != "Already up to date." ]]; then
    echo \"$password\" | sudo rm -rf build install log
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
else
    echo "No changes to pull. Skipping colcon build."
fi
# Then run the steps remotely
for ip in "${!pi_credentials[@]}"; do
    creds="${pi_credentials[$ip]}"
    username="${creds%%:*}"
    password="${creds#*:}"

    sshpass -p "$password" ssh -tt -o StrictHostKeyChecking=no "${username}@${ip}" bash -c "'
        cd $WORKSPACE_FOLDER/src
        if [ ! -d post ]; then
            git clone https://github.com/UofI-CDACS/rome_py.git post
        fi
        cd post
        git checkout $BRANCH_NAME
        GIT_OUTPUT=\$(git pull)
        cd $WORKSPACE_FOLDER
        if [[ \"\$GIT_OUTPUT\" != \"Already up to date.\" ]]; then
            source /opt/ros/jazzy/setup.bash
            echo \"$password\" | sudo -S rm -rf build install log
            colcon build --symlink-install
        else
            echo \"No changes to pull. Skipping colcon build.\"
        fi
    '"
    echo "Copied and built on $ip"
done

# Loop through pi_credentials associative array
for HOST in "${!pi_credentials[@]}"; do
    USER_PASS="${pi_credentials[$HOST]}"
    USER="${USER_PASS%%:*}"
    PASSWORD="${USER_PASS##*:}"
    USER_AT_HOST="${USER}@${HOST}"
    NODE_NAME="${pi_node_names[$HOST]}"
    REMOTE_COMMAND="
        cd $WORKSPACE_FOLDER
        source "$WORKSPACE_FOLDER/src/post/post_scripts/$DDS_CONFIG" < <(echo "$WORKSPACE_FOLDER/src/post/post_scripts/")
        source "$WORKSPACE_FOLDER/src/install/setup.bash"
        ros2 run post_station station --ros-args -r __node:=${NODE_NAME}
    "
    gnome-terminal --tab -- bash -c "echo 'Connecting to $USER_AT_HOST...'; sshpass -p '$PASSWORD' ssh -tt -o StrictHostKeyChecking=no $USER_AT_HOST '$REMOTE_COMMAND; bash'; exec bash"
done