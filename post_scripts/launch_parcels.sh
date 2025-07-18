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
WORKSPACE_FOLDER="${WORKSPACE_FOLDER:-~/Desktop/test_ws}"
DDS_CONFIG="${DDS_CONFIG:-cyclonedds.xml}"
PARCEL_COUNT="${PARCEL_COUNT:-1}"
OWNER="${OWNER:-Owner}"
INSTRUCTION_SET="${INSTRUCTION_SET:-default}"
CUSTOM_PARAMS="${CUSTOM_PARAMS:-FALSE}"
LOOP_INFINITELY="${LOOP_INFINITELY:-FALSE}"
NEXT_LOCATION="${NEXT_LOCATION:-rospi_1}"
FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" --text="Enter the Parcels Parameters" \
    --field="Workspace Folder":TXT "$WORKSPACE_FOLDER" \
    --field="DDS Config":CB "cyclonedds.xml!fastdds.xml" "$DDS_CONFIG" \
    --field="Parcel Count":NUM \
    --field="Owner":TXT "$OWNER" \
    --field="Instruction Set":TXT "$INSTRUCTION_SET" \
    --field="Custom Parameters (optional)":CHK "$CUSTOM_PARAMS" \
    --field="Next Location":TXT "rospi_1" "$NEXT_LOCATION" \
    --field="Loop Infinitely":CHK "$LOOP_INFINITELY" \
    --button="Launch:0" --button="Cancel:1" \
    --separator=",")

YAD_EXIT_CODE=$?
if [ $YAD_EXIT_CODE -ne 0 ]; then
    exit 0
fi

if [ -z "$FORM_OUTPUT" ]; then
    yad --error --text="All required fields must be filled."
    exit 1
fi


IFS=',' read -r WORKSPACE_FOLDER DDS_CONFIG PARCEL_COUNT OWNER INSTRUCTION_SET CUSTOM_PARAMS NEXT_LOCATION LOOP_INFINITELY <<< "$FORM_OUTPUT"
echo "Parameters received:"
echo "WORKSPACE_FOLDER: $WORKSPACE_FOLDER"
echo "DDS_CONFIG: $DDS_CONFIG"
echo "PARCEL_COUNT: $PARCEL_COUNT"
echo "OWNER: $OWNER"
echo "INSTRUCTION_SET: $INSTRUCTION_SET"
echo "CUSTOM_PARAMS: $CUSTOM_PARAMS"
echo "NEXT_LOCATION: $NEXT_LOCATION"
echo "LOOP_INFINITELY: $LOOP_INFINITELY"
if [ "$CUSTOM_PARAMS" = "TRUE" ]; then
    echo "Parsing logs enabled."
    PARAMS=$(yad --entry --title="Custom Parameters" --text="Enter custom parameters (comma-separated):" --entry-text "" --button="OK:0" --button="Cancel:1" --separator=",")
    IFS=',' read -ra PARAMS <<< "$PARAMS"
    echo "PARAMS: ${PARAMS[@]}"
else
    CUSTOM_PARAMS=""
fi
yad --question --title="Parse Logs" --text="Do you want to parse the logs?" --button=Yes:0 --button=No:1


if [ $? -eq 0 ]; then
    PARSE_LOGS=true
else
    PARSE_LOGS=false
fi

cd $WORKSPACE_FOLDER
source "$WORKSPACE_FOLDER/src/rome_py/POST_SCRIPTS/$DDS_CONFIG"
source install/setup.bash
ros2 launch post_station send_parcel_launch.py --ros-args \
    -p parcel_count:=$PARCEL_COUNT \
    -p owner:="$OWNER" \
    -p INSTRUCTION_SET:="$INSTRUCTION_SET" \
    -p data:="$PARAMS"
    sleep 10 # This should be adjusted to be more accurate on when the logging is done
    if [ "$PARSE_LOGS" = true ]; then
        python3 "$WORKSPACE_FOLDER/src/post_logs/Parse_logs.py"
    fi

if [ "$LOOP_INFINITELY" = "TRUE" ]; then
    ros2 launch post_station send_parcel_launch.py --ros-args \
        -p parcel_count:=$PARCEL_COUNT \
        -p owner:="$OWNER" \
        -p INSTRUCTION_SET:="$INSTRUCTION_SET" \
        -p data:="$PARAMS"
    sleep 5 # This should be adjusted to be more accurate on when the logging is done
    if [ "$PARSE_LOGS" = true ]; then
        python3 "$WORKSPACE_FOLDER/src/post_logs/Parse_logs.py"
    fi