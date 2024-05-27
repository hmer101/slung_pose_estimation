#!/bin/bash
# Script to copy results files from the physical system (drones and load) to the local machine

# Author: Harvey Merton
# Date: 03/27/24

NUM_DRONES=3 # Set the maximum number of drones to launch
NUM_LOAD=1  # Set the number of loads to be used

START_DRONE_NUM=1  # Set the starting drone number
START_LOAD_NUM=1  # Set the starting load number

REMOTE_DATA_FOLDER="/home/ws_ros2/install/slung_pose_estimation/share/slung_pose_estimation/data"  # Set the remote data folder to copy from
LOCAL_DATA_FOLDER="/home/harvey/px4_ros_com_ros2/src/slung_pose_estimation/data"  # Set the local data folder to copy to

# Find the files present in the remote file list that are not present in the local file list
find_missing_local_files(){
    local remote_file_list=$1
    local local_file_list=$2
    local missing_file_list=$3
    local device_type=$4
    local device_num=$5

    while read -r file; do
        if ! grep -q "^$file$" $local_file_list; then
            echo "$file"
        fi
    done < $remote_file_list > $missing_file_list   
}


# Function to SSH into devices and view logs
ssh_and_copy_files() {
    local device_type=$1
    local num_devices=$2
    local start_num=$3
    local uuid_file=$4
    local local_file_list=$5

    local count=0  # Initialize a counter for launched devices
    local line_num=0  # Initialize a line counter

    while IFS=' ' read -r uuid ip_addr; do #line
        # Start at start_num and end at num_devices
        line_num=$((line_num + 1))

        if [ "$line_num" -lt "$start_num" ]; then
            continue  # Skip lines until start_num is reached
        fi

        if [ "$count" -ge "$num_devices" ]; then
            echo "Maximum number of ${device_type}s ($num_devices) reached. Exiting..."
            break
        fi

        # Extract info from txt file to ssh into devices
        device_uuid=${uuid} #${line}
        device_ip=${ip_addr}
        device_number=$((count + 1))

        # 0. Generate a local file list (each loop incase drones have same files - don't need to recopy)
        ls $LOCAL_DATA_FOLDER > $local_file_list

        # 1. Extract list of files present on the remote device in the data folder
        remote_file_list="$LOCAL_DATA_FOLDER/0_remote_files_${device_type}_$device_number.txt"
        echo "ls $REMOTE_DATA_FOLDER" | balena ssh $device_ip main > $remote_file_list #$device_uuid.local

        # 2. Find files missing from the local machine
        missing_file_list="$LOCAL_DATA_FOLDER/0_missing_files_${device_type}_$device_number.txt"
        find_missing_local_files $remote_file_list $local_file_list $missing_file_list $device_type $device_number

        # 3. Copy the missing files from the remote device to the local machine
        while read -r file; do
            echo "cat $REMOTE_DATA_FOLDER/$file" | balena ssh $device_ip main > "$LOCAL_DATA_FOLDER/$file" #$device_uuid.local
        done < $missing_file_list

        # Increment the device count
        count=$((count + 1))
    done < "$uuid_file"
}

# Generate list of files in local data folder currently (used to compare with remote files to see what to copy)
# Do this in the loop above to avoid recopying files that are already present
local_file_list="$LOCAL_DATA_FOLDER/0_local_files.txt"
# ls $LOCAL_DATA_FOLDER > $local_file_list

# Balena SSH into drones and copy the results
ssh_and_copy_files "drone" $NUM_DRONES $START_DRONE_NUM "../../swarm_load_carry/config/phys_drones_uuid.txt" $local_file_list

# Balena SSH into loads and copy the results
ssh_and_copy_files "load" $NUM_LOAD $START_LOAD_NUM "../../swarm_load_carry/config/phys_load_uuid.txt" $local_file_list
