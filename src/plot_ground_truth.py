import matplotlib.pyplot as plt
import math
import numpy as np

def read_log_file(filename):
    time, pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_load_rel_world_gt, rpy_load_rel_world_gt = [], [], [], [], []
    pos_err_load, att_err_load, distTransLoad, distAngGeoLoad = [], [], [], []
    pos_drones_rel_world_desired, rpy_drones_rel_world_desired = [], []
    pos_drones_rel_world_gt, rpy_drones_rel_world_gt = [], []
    pos_err_drones, att_err_drones, distTransDrones, distAngGeoDrones = [], [], [], []

    with open(filename, 'r') as file:
        for line in file:
            data = line.split()
            if len(data) == 81: # Adjusted length check for 3 drones
                time.append(float(data[0]))
                pos_load_rel_world_desired.append([float(data[1]), float(data[2]), float(data[3])])
                rpy_load_rel_world_desired.append([float(data[4]), float(data[5]), float(data[6])])
                pos_load_rel_world_gt.append([float(data[7]), float(data[8]), float(data[9])])
                rpy_load_rel_world_gt.append([float(data[10]), float(data[11]), float(data[12])])
                pos_err_load.append([float(data[13]), float(data[14]), float(data[15])])
                att_err_load.append([float(data[16]), float(data[17]), float(data[18])])
                distTransLoad.append(float(data[19]))
                distAngGeoLoad.append(float(data[20]))

                # Loop for drone data
                for i in range(3): # 3 drones
                    base_index = 21 + i * 20
                    pos_drones_rel_world_desired.append([float(data[base_index]), float(data[base_index + 1]), float(data[base_index + 2])])
                    rpy_drones_rel_world_desired.append([float(data[base_index + 3]), float(data[base_index + 4]), float(data[base_index + 5])])
                    pos_drones_rel_world_gt.append([float(data[base_index + 6]), float(data[base_index + 7]), float(data[base_index + 8])])
                    rpy_drones_rel_world_gt.append([float(data[base_index + 9]), float(data[base_index + 10]), float(data[base_index + 11])])
                    pos_err_drones.append([float(data[base_index + 12]), float(data[base_index + 13]), float(data[base_index + 14])])
                    att_err_drones.append([float(data[base_index + 15]), float(data[base_index + 16]), float(data[base_index + 17])])
                    distTransDrones.append(float(data[base_index + 18]))
                    distAngGeoDrones.append(float(data[base_index + 19]))

    # Convert to np
    time = np.array(time)
    pos_load_rel_world_desired = np.array(pos_load_rel_world_desired)
    rpy_load_rel_world_desired = np.array(rpy_load_rel_world_desired)
    pos_load_rel_world_gt = np.array(pos_load_rel_world_gt)
    rpy_load_rel_world_gt = np.array(rpy_load_rel_world_gt)
    pos_err_load = np.array(pos_err_load)
    att_err_load = np.array(att_err_load)
    distTransLoad = np.array(distTransLoad)
    distAngGeoLoad = np.array(distAngGeoLoad)

    pos_drones_rel_world_desired = np.array(pos_drones_rel_world_desired).reshape(-1, 3)
    rpy_drones_rel_world_desired = np.array(rpy_drones_rel_world_desired).reshape(-1, 3)
    pos_drones_rel_world_gt = np.array(pos_drones_rel_world_gt).reshape(-1, 3)
    rpy_drones_rel_world_gt = np.array(rpy_drones_rel_world_gt).reshape(-1, 3)
    pos_err_drones = np.array(pos_err_drones).reshape(-1, 3)
    att_err_drones = np.array(att_err_drones).reshape(-1, 3)
    distTransDrones = np.array(distTransDrones).reshape(-1, 3)
    distAngGeoDrones = np.array(distAngGeoDrones).reshape(-1, 3)

    return (time, pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_load_rel_world_gt, rpy_load_rel_world_gt,
            pos_err_load, att_err_load, distTransLoad, distAngGeoLoad,
            pos_drones_rel_world_desired, rpy_drones_rel_world_desired, pos_drones_rel_world_gt, rpy_drones_rel_world_gt,
            pos_err_drones, att_err_drones, distTransDrones, distAngGeoDrones)

# def plot_data(time, pos_load_rel_world_gt, rpy_load_rel_world_gt, pos_drones_rel_world_gt, rpy_drones_rel_world_gt,
#               pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_drones_rel_world_desired, rpy_drones_rel_world_desired,
#               title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
#     plt.figure(figsize=(15, 15))

#     # Position and Ground Truth Position
#     plt.subplot(3, 1, 1)
#     pos_load_rel_world_gt = list(zip(*pos_load_rel_world_gt))
#     pos_load_rel_world_desired = list(zip(*pos_load_rel_world_desired))
#     plt.plot(time, pos_load_rel_world_gt[0], label='pos_load_gt_x', color='blue')
#     plt.plot(time, pos_load_rel_world_gt[1], label='pos_load_gt_y', color='green')
#     plt.plot(time, pos_load_rel_world_gt[2], label='pos_load_gt_z', color='red')
#     plt.plot(time, pos_load_rel_world_desired[0], label='pos_load_desired_x', linestyle='dashed', color='blue')
#     plt.plot(time, pos_load_rel_world_desired[1], label='pos_load_desired_y', linestyle='dashed', color='green')
#     plt.plot(time, pos_load_rel_world_desired[2], label='pos_load_desired_z', linestyle='dashed', color='red')
#     plt.xlabel('Time (s)', fontsize=axes_label_font_size)
#     plt.ylabel('Position (m)', fontsize=axes_label_font_size)
#     plt.title('Load Position - GT vs Desired', fontsize=title_font_size)
#     plt.legend(fontsize=legend_font_size)
#     plt.xticks(fontsize=ticks_font_size)
#     plt.yticks(fontsize=ticks_font_size)

#     # RPY and Ground Truth RPY
#     plt.subplot(3, 1, 2)
#     rpy_load_rel_world_gt = np.array(rpy_load_rel_world_gt).T
#     rpy_load_rel_world_desired = np.array(rpy_load_rel_world_desired).T
#     for i in range(3):
#         plt.plot(time, unwrap_and_convert_to_degrees(rpy_load_rel_world_gt[i]), label=f'rpy_load_gt_{["roll", "pitch", "yaw"][i]}', color=f'C{i}')
#         plt.plot(time, unwrap_and_convert_to_degrees(rpy_load_rel_world_desired[i]), label=f'rpy_load_desired_{["roll", "pitch", "yaw"][i]}', linestyle='dashed', color=f'C{i}')
#     plt.xlabel('Time (s)', fontsize=axes_label_font_size)
#     plt.ylabel('Orientation (Degrees)', fontsize=axes_label_font_size)
#     plt.title('Load Orientation - GT vs Desired', fontsize=title_font_size)
#     plt.legend(fontsize=legend_font_size)
#     plt.xticks(fontsize=ticks_font_size)
#     plt.yticks(fontsize=ticks_font_size)

#     # Drone Position and Orientation
#     plt.subplot(3, 1, 3)
#     pos_drones_rel_world_gt = np.array(pos_drones_rel_world_gt).T
#     for i in range(3):
#         plt.plot(time, pos_drones_rel_world_gt[i], label=f'drone_{i+1}_pos_x', linestyle='dashed', color=f'C{i}')
#     plt.xlabel('Time (s)', fontsize=axes_label_font_size)
#     plt.ylabel('Position (m)', fontsize=axes_label_font_size)
#     plt.title('Drone Positions Relative to the World', fontsize=title_font_size)
#     plt.legend(fontsize=legend_font_size)
#     plt.xticks(fontsize=ticks_font_size)
#     plt.yticks(fontsize=ticks_font_size)

#     plt.tight_layout()
#     plt.show()

def plot_load_data(time, pos_load_rel_world_gt, rpy_load_rel_world_gt, pos_load_rel_world_desired, rpy_load_rel_world_desired,
                   title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(15, 10))

    # Position and Ground Truth Position
    plt.subplot(2, 1, 1)
    pos_load_rel_world_gt = list(zip(*pos_load_rel_world_gt))
    pos_load_rel_world_desired = list(zip(*pos_load_rel_world_desired))
    plt.plot(time, pos_load_rel_world_gt[0], label='pos_load_gt_x', color='blue')
    plt.plot(time, pos_load_rel_world_gt[1], label='pos_load_gt_y', color='green')
    plt.plot(time, pos_load_rel_world_gt[2], label='pos_load_gt_z', color='red')
    plt.plot(time, pos_load_rel_world_desired[0], label='pos_load_desired_x', linestyle='dashed', color='blue')
    plt.plot(time, pos_load_rel_world_desired[1], label='pos_load_desired_y', linestyle='dashed', color='green')
    plt.plot(time, pos_load_rel_world_desired[2], label='pos_load_desired_z', linestyle='dashed', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position (m)', fontsize=axes_label_font_size)
    plt.title('Load Position - GT vs Desired', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # RPY and Ground Truth RPY
    plt.subplot(2, 1, 2)
    rpy_load_rel_world_gt = np.array(rpy_load_rel_world_gt).T
    rpy_load_rel_world_desired = np.array(rpy_load_rel_world_desired).T
    for i in range(3):
        plt.plot(time, unwrap_and_convert_to_degrees(rpy_load_rel_world_gt[i]), label=f'rpy_load_gt_{["roll", "pitch", "yaw"][i]}', color=f'C{i}')
        plt.plot(time, unwrap_and_convert_to_degrees(rpy_load_rel_world_desired[i]), label=f'rpy_load_desired_{["roll", "pitch", "yaw"][i]}', linestyle='dashed', color=f'C{i}')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation (Degrees)', fontsize=axes_label_font_size)
    plt.title('Load Orientation - GT vs Desired', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_drones_data(time, pos_drones_rel_world_gt, rpy_drones_rel_world_gt, pos_drones_rel_world_desired, rpy_drones_rel_world_desired,
                     title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(15, 10))

    # Drone Position and Orientation
    plt.subplot(2, 1, 1)
    pos_drones_rel_world_gt = np.array(pos_drones_rel_world_gt).T
    for i in range(3):
        plt.plot(time, pos_drones_rel_world_gt[i], label=f'drone_{i+1}_pos_x', linestyle='dashed', color=f'C{i}')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position (m)', fontsize=axes_label_font_size)
    plt.title('Drone Positions Relative to the World', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # Add similar plots for orientation if needed
    plt.subplot(2, 1, 2)
    rpy_drones_rel_world_gt = np.array(rpy_drones_rel_world_gt).T
    for i in range(3):
        plt.plot(time, unwrap_and_convert_to_degrees(rpy_drones_rel_world_gt[i]), label=f'drone_{i+1}_rpy_{["roll", "pitch", "yaw"][i]}', color=f'C{i}')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation (Degrees)', fontsize=axes_label_font_size)
    plt.title('Drone Orientations Relative to the World', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_errors(time, pos_err_load, att_err_load, pos_err_drones, att_err_drones,
                title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(15, 10))

    # Load Position Errors
    plt.subplot(2, 1, 1)
    pos_err_load = list(zip(*pos_err_load))
    plt.plot(time, pos_err_load[0], label='pos_err_load_x', color='blue')
    plt.plot(time, pos_err_load[1], label='pos_err_load_y', color='green')
    plt.plot(time, pos_err_load[2], label='pos_err_load_z', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position Error (m)', fontsize=axes_label_font_size)
    plt.title('Load Position Error', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # Load Attitude Errors
    plt.subplot(2, 1, 2)
    att_err_load = list(zip(*att_err_load))
    plt.plot(time, unwrap_and_convert_to_degrees(att_err_load[0]), label='att_err_load_roll', color='blue')
    plt.plot(time, unwrap_and_convert_to_degrees(att_err_load[1]), label='att_err_load_pitch', color='green')
    plt.plot(time, unwrap_and_convert_to_degrees(att_err_load[2]), label='att_err_load_yaw', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Attitude Error (Degrees)', fontsize=axes_label_font_size)
    plt.title('Load Attitude Error', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_trans_geo(time, distTransLoad, distAngGeoLoad, distTransDrones, distAngGeoDrones,
                   title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(15, 10))

    # Translational and Angular Distances
    plt.subplot(2, 1, 1)
    plt.plot(time, distTransLoad, label='distTransLoad', color='blue')
    plt.plot(time, distAngGeoLoad, label='distAngGeoLoad', color='green')
    for i in range(3):
        plt.plot(time, distTransDrones[:, i], label=f'distTransDrone_{i+1}', linestyle='dashed', color=f'C{i}')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Distance (m)', fontsize=axes_label_font_size)
    plt.title('Translational Distances', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.subplot(2, 1, 2)
    plt.plot(time, distAngGeoLoad, label='distAngGeoLoad', color='blue')
    for i in range(3):
        plt.plot(time, distAngGeoDrones[:, i], label=f'distAngGeoDrone_{i+1}', linestyle='dashed', color=f'C{i}')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Angle Distance (Degrees)', fontsize=axes_label_font_size)
    plt.title('Angular Distances', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def unwrap_and_convert_to_degrees(data):
    data_unwrapped = np.unwrap(np.radians(data)) # Unwrap phases in radians
    return np.degrees(data_unwrapped) # Convert radians to degrees

def main():
    # Read data
    # Retrieve data from log file
    path = '/home/harvey/ws_ros2/src/slung_pose_estimation/src/' #'/home/harvey/px4_ros_com_ros2/install/slung_pose_estimation/share/slung_pose_estimation/data/'
    filename = path + '2024_08_29_17_04_40_logger1.txt' # replace with your log file path

    (time, pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_load_rel_world_gt, rpy_load_rel_world_gt,
    pos_err_load, att_err_load, distTransLoad, distAngGeoLoad,
    pos_drones_rel_world_desired, rpy_drones_rel_world_desired, pos_drones_rel_world_gt, rpy_drones_rel_world_gt,
    pos_err_drones, att_err_drones, distTransDrones, distAngGeoDrones) = read_log_file(filename)

    # Plot settings
    title_font_size = 14
    axes_label_font_size = 12
    legend_font_size = 10
    ticks_font_size = 10

    # Plot Data
    # plot_data(time, pos_load_rel_world_gt, rpy_load_rel_world_gt, pos_drones_rel_world_gt, rpy_drones_rel_world_gt,
    #         pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_drones_rel_world_desired, rpy_drones_rel_world_desired,
    #         title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)

    # Plot load data
    plot_load_data(
        time, 
        pos_load_rel_world_gt, 
        rpy_load_rel_world_gt, 
        pos_load_rel_world_desired, 
        rpy_load_rel_world_desired,
        title_font_size=title_font_size, 
        axes_label_font_size=axes_label_font_size, 
        legend_font_size=legend_font_size, 
        ticks_font_size=ticks_font_size
    )

    # Plot drones data
    plot_drones_data(
        time, 
        pos_drones_rel_world_gt, 
        rpy_drones_rel_world_gt, 
        pos_drones_rel_world_desired, 
        rpy_drones_rel_world_desired,
        title_font_size=title_font_size, 
        axes_label_font_size=axes_label_font_size, 
        legend_font_size=legend_font_size, 
        ticks_font_size=ticks_font_size
    )

    # Plot Errors
    # plot_errors(time, pos_err_load, att_err_load, pos_err_drones, att_err_drones,
                # title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)
    

if __name__ == '__main__':
    main()