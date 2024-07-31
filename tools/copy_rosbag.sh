#!/bin/bash

# Define the SSH details
UUID_FILE="/home/harvey/px4_ros_com_ros2/src/multi_drone_slung_load/config/phys_load_uuid.txt" #"phys_drones_uuid.txt"
DEVICE_INDEX=1

REMOTE_DIR="/home/ws_ros2/data" #ws_ros2
LOCAL_DIR="/home/harvey/px4_ros_com_ros2/data"

# Extract the remote host IP address
get_remote_ip() {
    local uuid_file=$1
    local device_index=$2

    local line_num=0  # Initialize a line counter

    # Read the UUID and IP address from the file
    while IFS=' ' read -r uuid ip_addr; do #line
        # Skip lines until device_index is reached
        line_num=$((line_num + 1))

        if [ "$line_num" -lt "$device_index" ]; then
            continue  
        fi

        # Extract info from txt file to ssh into devices
        device_uuid=${uuid}
        device_ip=${ip_addr}

        # Return the IP address of the remote host
        echo "$device_ip"
        return 0
        
    done < "$uuid_file"
}

# Get the remote IP address
remote_ip=$(get_remote_ip $UUID_FILE $DEVICE_INDEX)
#echo "The remote IP address is: $remote_ip"

# Find the latest file in the remote directory (to account for the new rosbag that has started)
LATEST_FILE=$(echo "find $REMOTE_DIR -type f -printf '%p\n' | sort | tail -n 1" | balena ssh $remote_ip main) #tail -n 1
LATEST_DIR=$(dirname "$LATEST_FILE")
LATEST_FOLDER=$(basename "$LATEST_DIR")

# Kill ros2 on the host such that the metadata is saved 
echo "pkill -f 'opt/ros/humble'" | balena ssh $remote_ip main

# Wait until new rosbag starts on remote host
sleep 10

# Step 1: Copy the entire folder from the container to the remote host
CONTAINER_NAME=$(echo "balena-engine ps --format '{{.Names}}'" | balena ssh $remote_ip | head -n 1)
echo "balena-engine cp $CONTAINER_NAME:$LATEST_DIR /tmp/$LATEST_FOLDER" | balena ssh $remote_ip

# Step 2: Copy the folder from the remote host to the local machine
scp -r -P 22222 root@$remote_ip:/tmp/$LATEST_FOLDER $LOCAL_DIR/

# Clean up the temporary folder on the remote host
echo "rm -rf /tmp/$LATEST_FOLDER" | balena ssh $remote_ip

# Display the copied folder
echo "The latest data folder has been copied to: $LOCAL_DIR/$LATEST_FOLDER"
