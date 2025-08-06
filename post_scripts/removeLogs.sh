#!/bin/bash

PI_IPS=("172.23.254.18" "172.23.254.22" "172.23.254.23" "172.23.254.24")
PI_USER="rospi"
FOLDER_TO_CLEAN="~/Desktop/test_ws/loop"

remove_files_from_pi() {
    local pi_ip=$1
    echo "Connecting to Pi at $pi_ip..."
    
    ssh "$PI_USER@$pi_ip" "rm -rf $FOLDER_TO_CLEAN/*"
    
    if [ $? -eq 0 ]; then
        echo "Successfully cleaned $FOLDER_TO_CLEAN on $pi_ip"
    else
        echo "Failed to clean files on $pi_ip"
    fi
}
MASTER_PI_IP=("172.23.254.20")
echo "Starting log cleanup on all Pi devices..."

for pi_ip in "${PI_IPS[@]}"; do
    remove_files_from_pi "$pi_ip"
done
FOLDER_TO_CLEAN="~/Desktop/test_ws/graveyard"
remove_files_from_pi "${MASTER_PI_IP[0]}"
echo "Log cleanup completed."