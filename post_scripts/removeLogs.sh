#!/bin/bash

PI_IPS=("172.23.254.20")
PI_USER="rospi"
FOLDER_TO_CLEAN="/var/lib/Logsforgrafana"

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

echo "Starting log cleanup on all Pi devices..."

for pi_ip in "${PI_IPS[@]}"; do
    remove_files_from_pi "$pi_ip"
done

echo "Log cleanup completed."