#!/bin/bash

WORKSPACE_FOLDER="${WORKSPACE_FOLDER:-$HOME/Desktop/test_ws}"
# Default values for the new parameters
STATION_NAME="${STATION_NAME:-owner_station}"
MODE="${MODE:-round_robin}"
COUNT="${COUNT:-20}"
INTERVAL_SEC="${INTERVAL_SEC:-0.1}"
TTL_VALUE="${TTL_VALUE:-10}"
DDS_CONFIG="${DDS_CONFIG:-cyclone_source.sh}"
PARCEL_COUNT="${PARCEL_COUNT:-1}"
OWNER="${OWNER:-Owner}"
INSTRUCTION_SET="${INSTRUCTION_SET:-loop}"
LOOP_INFINITELY="${LOOP_INFINITELY:-FALSE}"
CUSTOM_PARAMS="${CUSTOM_PARAMS:-FALSE}"
NEXT_LOCATION="${NEXT_LOCATION:-['rospi_1','rospi_2','rospi_3','rospi_4']}"
FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" --text="Enter the Parcels Parameters" \
    --field="Workspace Folder":TXT "$WORKSPACE_FOLDER" \
    --field="Station Name":TXT "$STATION_NAME" \
    --field="Mode":CB "round_robin!broadcast!unicast" "$MODE" \
    --field="Count":NUM "$COUNT" \
    --field="Interval (sec)":NUM "$INTERVAL_SEC" \
    --field="TTL Value":NUM "$TTL_VALUE" \
    --field="DDS Config":CB "cyclone_source.sh!fast_source.sh" "$DDS_CONFIG" \
    --field="Parcel Count":NUM "$PARCEL_COUNT" \
    --field="Owner":TXT "$OWNER" \
    --field="Instruction Set":TXT "$INSTRUCTION_SET" \
    --field="Next Location":TXT "$NEXT_LOCATION" \
    --field="Loop Infinitely":CHK "$LOOP_INFINITELY" \
    --field="Custom Parameters":CHK "$CUSTOM_PARAMS" \
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


IFS=',' read -r WORKSPACE_FOLDER STATION_NAME MODE COUNT INTERVAL_SEC TTL_VALUE DDS_CONFIG PARCEL_COUNT OWNER INSTRUCTION_SET NEXT_LOCATION LOOP_INFINITELY CUSTOM_PARAMS <<< "$FORM_OUTPUT"
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
source "$WORKSPACE_FOLDER/src/post/post_scripts/$DDS_CONFIG"
source "$WORKSPACE_FOLDER/install/setup.bash"

if [ -n "$PARAMS" ]; then
    PARAMS_JSON=$(printf '%s\n' "${PARAMS[@]}" | jq -R . | jq -s .)
else
    PARAMS_JSON="{}"
fi

if [ -n "$PARAMS" ]; then
    PARAMS_JSON=$(printf '%s\n' "${PARAMS[@]}" | jq -R . | jq -s . | jq '. + [{"key": "ttl", "val": "'$TTL_VALUE'"}]')
else
    PARAMS_JSON='[{"key": "ttl", "val": "'$TTL_VALUE'"}]'
fi
echo "PARAMS_JSON: $PARAMS_JSON"
ros2 run post_core station --type sender --name $STATION_NAME --ros-args \
    -p destinations:="$NEXT_LOCATION" \
    -p count:=$PARCEL_COUNT \
    -p mode:="$MODE" \
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
        -p interval_sec:=$INTERVAL_SEC \
        -p owner_id:="$OWNER" \
        -p instruction_set:="$INSTRUCTION_SET" \
        -p data:="$PARAMS_JSON"
    sleep 5 # This should be adjusted to be more accurate on when the logging is done
    if [ "$PARSE_LOGS" = true ]; then
        python3 "$WORKSPACE_FOLDER/src/post/post_scripts/logParser.py"
    fi
