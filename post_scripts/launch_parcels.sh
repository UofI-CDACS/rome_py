#!/bin/bash

WORKSPACE_FOLDER="${WORKSPACE_FOLDER:-$HOME/Desktop/test_ws}"
# Default values for the new parameters
STATION_NAME="${STATION_NAME:-owner_station}"
MODE="${MODE:-round_robin}"
INTERVAL_MS="${INTERVAL_MS:-100}"
TTL_VALUE="${TTL_VALUE:-10}"
DDS_CONFIG="${DDS_CONFIG:-cyclonedds_source.sh}"
PARCEL_COUNT="${PARCEL_COUNT:-100}"
OWNER="${OWNER:-Owner}"
INSTRUCTION_SET="${INSTRUCTION_SET:-loop}"
LOOP_INFINITELY="${LOOP_INFINITELY:-FALSE}"
LOSS_MODE="${LOSS_MODE:-lossless}"  # 'lossy' or 'lossless'
CUSTOM_PARAMS="${CUSTOM_PARAMS:-FALSE}"
QOS_DEPTH="${QOS_DEPTH:-10}"
NEXT_LOCATION="${NEXT_LOCATION:-[rospi_1]}"
FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" --text="Enter the Parcels Parameters" \
    --field="Workspace Folder":TXT "$WORKSPACE_FOLDER" \
    --field="Station Name":TXT "$STATION_NAME" \
    --field="Mode":CB "round_robin!random!once" \
    --field="DDS Config":CB "cyclonedds_source.sh!fastrtps_source.sh" \
    --field="Interval (ms)":NUM "$INTERVAL_MS" \
    --field="TTL Value":NUM "$TTL_VALUE" \
    --field="Parcel Count":NUM "$PARCEL_COUNT" \
    --field="Owner":TXT "$OWNER" \
    --field="Instruction Set":TXT "$INSTRUCTION_SET" \
    --field="Next Location":TXT "$NEXT_LOCATION" \
    --field="Loop Infinitely":CHK "$LOOP_INFINITELY" \
    --field="Custom Parameters":CHK "$CUSTOM_PARAMS" \
    --field="Loss Mode":CB "lossless!lossy" \
    --field="QOS Depth":NUM "$QOS_DEPTH" \
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

IFS=',' read -r WORKSPACE_FOLDER STATION_NAME MODE DDS_CONFIG INTERVAL_MS TTL_VALUE PARCEL_COUNT OWNER INSTRUCTION_SET NEXT_LOCATION LOOP_INFINITELY CUSTOM_PARAMS LOSS_MODE QOS_DEPTH <<< "$FORM_OUTPUT"

# Convert INTERVAL_MS to INTERVAL_SEC
INTERVAL_SEC=$(echo "scale=3; $INTERVAL_MS / 1000" | bc)

echo "Parameters received:"
echo "WORKSPACE_FOLDER: $WORKSPACE_FOLDER"
echo "STATION_NAME: $STATION_NAME"
echo "MODE: $MODE"
echo "COUNT: $COUNT"
echo "INTERVAL_SEC: $INTERVAL_SEC"
echo "TTL_VALUE: $TTL_VALUE"
echo "DDS_CONFIG: $DDS_CONFIG"
echo "PARCEL_COUNT: $PARCEL_COUNT"
echo "OWNER: $OWNER"
echo "INSTRUCTION_SET: $INSTRUCTION_SET"
echo "NEXT_LOCATION: $NEXT_LOCATION"
echo "LOOP_INFINITELY: $LOOP_INFINITELY"
echo "CUSTOM_PARAMS: $CUSTOM_PARAMS"
echo "LOSS_MODE: $LOSS_MODE"
echo "QOS_DEPTH: $QOS_DEPTH"
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
source "$WORKSPACE_FOLDER/src/post/post_scripts/$DDS_CONFIG" \""$WORKSPACE_FOLDER"\"
source "$WORKSPACE_FOLDER/install/setup.bash"

if [ -n "$PARAMS" ]; then
    PARAMS_JSON=$(printf '%s\n' "${PARAMS[@]}" | jq -R . | jq -s .)
else
    PARAMS_JSON="{}"
fi

if [ -n "$PARAMS" ]; then
    PARAMS_JSON=$(printf '%s\n' "${PARAMS[@]}" | jq -R . | jq -s . | jq '. + [{"key": "ttl", "val": "'$TTL_VALUE'"}]')
else
PARAMS_JSON="['ttl:"$TTL_VALUE"']"
fi
echo "PARAMS_JSON: $PARAMS_JSON"
#gnome-terminal -- bash -c "cd $WORKSPACE_FOLDER && source $WORKSPACE_FOLDER/src/post/post_scripts/$DDS_CONFIG && source $WORKSPACE_FOLDER/install/setup.bash && ros2 run post_core station --type graveyard --name default_graveyard --lossmode $LOSS_MODE --depth $QOS_DEPTH; exec bash"
sleep 3
ros2 run post_core station --type sender --name $STATION_NAME --lossmode $LOSS_MODE --depth $QOS_DEPTH --ros-args \
    -p destinations:="$NEXT_LOCATION" \
    -p count:=$PARCEL_COUNT \
    -p mode:="$MODE" \
    -p loss_mode:="$LOSS_MODE" \
    -p interval_sec:=$INTERVAL_SEC \
    -p owner_id:="$OWNER" \
    -p instruction_set:="$INSTRUCTION_SET" \
    -p data:="$PARAMS_JSON"

sleep 10
if [ "$PARSE_LOGS" = true ]; then
    python3 "$WORKSPACE_FOLDER/src/post/post_scripts/logParser.py"
fi

if [ "$LOOP_INFINITELY" = "TRUE" ]; then
    ros2 run post_core station --type sender --name $STATION_NAME --ros-args \
        -p destinations:="$NEXT_LOCATION" \
        -p count:=$PARCEL_COUNT \
        -p mode:="$MODE" \
        -p loss_mode:="$LOSS_MODE" \
        -p interval_sec:=$INTERVAL_SEC \
        -p owner_id:="$OWNER" \
        -p instruction_set:="$INSTRUCTION_SET" \
        -p data:="$PARAMS_JSON"
    sleep 5 # This should be adjusted to be more accurate on when the logging is done
    if [ "$PARSE_LOGS" = true ]; then
        python3 "$WORKSPACE_FOLDER/src/post/post_scripts/logParser.py"
    fi
