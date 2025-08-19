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
INSTRUCTION_SET="${INSTRUCTION_SET:-loop_dynamic}"
LOOP_INFINITELY="${LOOP_INFINITELY:-FALSE}"
LOSS_MODE="${LOSS_MODE:-lossless}"  # 'lossy' or 'lossless'
CUSTOM_PARAMS="${CUSTOM_PARAMS:-FALSE}"
QOS_DEPTH="${QOS_DEPTH:-10}"
NEXT_LOCATION="${NEXT_LOCATION:-[rospi_1]}"
FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" --text="Enter the Parcels Parameters" \
    --field="Workspace Folder":TXT "$WORKSPACE_FOLDER" \
    --field="Station Name":TXT "$STATION_NAME" \
    --field="Mode":CB "round_robin!random!once" \
    --field="DDS Config":CB "cyclonedds_source.sh!zenohdds_source.sh!fastrtps_source.sh" \
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
    --separator="|")

YAD_EXIT_CODE=$?
if [ $YAD_EXIT_CODE -ne 0 ]; then
    exit 0
fi

if [ -z "$FORM_OUTPUT" ]; then
    yad --error --text="All required fields must be filled."
    exit 1
fi

IFS='|' read -r WORKSPACE_FOLDER STATION_NAME MODE DDS_CONFIG INTERVAL_MS TTL_VALUE PARCEL_COUNT OWNER INSTRUCTION_SET NEXT_LOCATION LOOP_INFINITELY CUSTOM_PARAMS LOSS_MODE QOS_DEPTH <<< "$FORM_OUTPUT"

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
    PARAMS=""
fi
ROUTE_DEFAULT='{"rospi_1": "rospi_2", "rospi_2": "rospi_3", "rospi_3": "rospi_4", "rospi_4": "rospi_1"}'
if [ $INSTRUCTION_SET = "loop_dynamic" ]; then
    ROUTE_MAP=$(yad --entry --title="Dynamic Route Configuration" --text="Enter route mapping (JSON format):\nExample: {\"rospi_1\": \"rospi_2\", \"rospi_2\": \"rospi_4\", \"rospi_3\": \"rospi_1\", \"rospi_4\": \"rospi_3\"}" --entry-text "$ROUTE_DEFAULT" --button="OK:0" --button="Cancel:1" --width=600 --height=200)
    if [ $? -ne 0 ] || [ -z "$ROUTE_MAP" ]; then
        echo "Route configuration cancelled or empty"
        exit 1
    fi
    echo "ROUTE_MAP: $ROUTE_MAP"
fi
#yad --question --title="Parse Logs" --text="Do you want to parse the logs?" --button=Yes:0 --button=No:1
#if [ $? -eq 0 ]; then
#    PARSE_LOGS=true
#else
#    PARSE_LOGS=false
#fi

cd $WORKSPACE_FOLDER
source "$WORKSPACE_FOLDER/install/setup.bash"
source "$WORKSPACE_FOLDER/src/post/post_scripts/$DDS_CONFIG" "$WORKSPACE_FOLDER"

if [ -n "$PARAMS" ]; then
    if [ -n "$ROUTE_MAP" ]; then
        # Escape the route JSON properly
        ROUTE_ESCAPED=$(printf '%s' "$ROUTE_MAP" | sed 's/"/\\"/g')
        PARAMS_JSON=$(printf '%s\n' "${PARAMS[@]}" | jq -R . | jq -s . | jq --arg route "$ROUTE_ESCAPED" '. + ["ttl:'$TTL_VALUE'", "route:\($route)"]')
    else
        PARAMS_JSON=$(printf '%s\n' "${PARAMS[@]}" | jq -R . | jq -s . | jq '. + ["ttl:'$TTL_VALUE'"]')
    fi
else
    if [ -n "$ROUTE_MAP" ]; then
        # Escape the route JSON properly
        ROUTE_ESCAPED=$(printf '%s' "$ROUTE_MAP" | sed 's/"/\\"/g')
        PARAMS_JSON='["ttl:'$TTL_VALUE'", "route:'$ROUTE_ESCAPED'"]'
    else
        PARAMS_JSON='["ttl:'$TTL_VALUE'"]'
    fi
fi

echo "PARAMS_JSON: $PARAMS_JSON"
python3 -c "
import pymongo
import json
from datetime import datetime

# MongoDB connection
database = pymongo.MongoClient('mongodb://root:example@172.23.254.20:27017/')
collection = database['logs']['launchSettings']

# Prepare document
doc = {
    'timestamp': datetime.now(),
    'station_name': '$STATION_NAME',
    'mode': '$MODE',
    'interval_sec': float('$INTERVAL_SEC'),
    'ttl_value': int('$TTL_VALUE'),
    'dds_config': '$DDS_CONFIG',
    'parcel_count': int('$PARCEL_COUNT'),
    'owner': '$OWNER',
    'instruction_set': '$INSTRUCTION_SET',
    'send_locations': '$NEXT_LOCATION',
    'loss_mode': '$LOSS_MODE',
    'qos_depth': int('$QOS_DEPTH'),
    }

# Insert document
collection.insert_one(doc)
print('Data sent to MongoDB successfully')
"
#gnome-terminal -- bash -c "cd $WORKSPACE_FOLDER && source $WORKSPACE_FOLDER/src/post/post_scripts/$DDS_CONFIG && source $WORKSPACE_FOLDER/install/setup.bash && ros2 run post_core station --type graveyard --name default_graveyard --lossmode $LOSS_MODE --depth $QOS_DEPTH; exec bash"
#sleep 3
ros2 run post_core station --type sender --name $STATION_NAME --lossmode $LOSS_MODE --depth $QOS_DEPTH --ros-args \
    -p destinations:="$NEXT_LOCATION" \
    -p count:=$PARCEL_COUNT \
    -p mode:="$MODE" \
    -p interval_sec:=$INTERVAL_SEC \
    -p owner_id:="$OWNER" \
    -p instruction_set:="$INSTRUCTION_SET" \
    -p data:="$PARAMS_JSON"

#if [ "$PARSE_LOGS" = true ]; then
#    HOSTNAME=$(hostname)
#    graveyard_linecount=$(wc -l < "${WORKSPACE_FOLDER}/graveyard/default_graveyard/log-${OWNER}-${HOSTNAME}.txt")
#    prev_graveyard_linecount=0
#    while true; do
#        graveyard_linecount=$(wc -l < "${WORKSPACE_FOLDER}/graveyard/default_graveyard/log-${OWNER}-${HOSTNAME}.txt")
#        if [ "$graveyard_linecount" -gt "$prev_graveyard_linecount" ]; then
#            sleep 1
#           prev_graveyard_linecount=$graveyard_linecount
#            echo "Waiting for logs to be written..."
#        else
#            break
#        fi
#   done
#    python3 "$WORKSPACE_FOLDER/src/post/post_scripts/logParser.py" --owner ${OWNER} --instruction_set ${INSTRUCTION_SET}
#fi

if [ "$LOOP_INFINITELY" = "TRUE" ]; then
    ros2 run post_core station --type sender --name $STATION_NAME --lossmode $LOSS_MODE --depth $QOS_DEPTH --ros-args \
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
    fi
