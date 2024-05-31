#!/bin/bash

# Define the SSH details
REMOTE_USER="root"
REMOTE_HOST="remote_host"
REMOTE_DIR="/path/to/dir"
LOCAL_DIR="/path/to/local/dir"

# SSH into the remote device and find the latest file
LATEST_FILE=$(ssh $REMOTE_USER@$REMOTE_HOST "find $REMOTE_DIR -type f -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2")

# Check if the LATEST_FILE variable is not empty
if [ -n "$LATEST_FILE" ]; then
  # Copy the latest file to the local directory
  scp $REMOTE_USER@$REMOTE_HOST:"$LATEST_FILE" "$LOCAL_DIR"
  echo "Latest file copied to $LOCAL_DIR"
else
  echo "No file found in the specified directory."
fi

# export TMP_FILE=frames_2024-05-30_17.39.51.pdf
# echo "balena-engine cp c6c68d7f44c9:/home/$TMP_FILE /tmp/rosbag_0.db3" | balena ssh 192.168.3.1
# scp -P 22222 root@192.168.3.1:/tmp/$TMP_FILE ~/$TMP_FILE

# Balena SSH into loads and copy the results
#ssh_and_copy_files "load" $NUM_LOAD $START_LOAD_NUM "../../swarm_load_carry/config/phys_load_uuid.txt" $local_file_list