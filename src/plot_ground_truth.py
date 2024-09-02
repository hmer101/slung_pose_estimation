import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np

def read_log_file(filename, only_read_mission=False):
    time, pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_load_rel_world_gt, rpy_load_rel_world_gt = [], [], [], [], []
    pos_err_load, att_err_load, distTransLoad, distAngGeoLoad = [], [], [], []
    
    pos_load_rel_world_qs, rpy_load_rel_world_qs = [], []
    qs_pos_diff_load, qs_att_diff_load, distTransLoadQs, distAngGeoLoadQs = [], [], [], []  
    
    pos_drones_rel_world_desired, rpy_drones_rel_world_desired = [], []
    pos_drones_rel_world_gt, rpy_drones_rel_world_gt = [], []
    pos_err_drones, att_err_drones, distTransDrones, distAngGeoDrones = [], [], [], []
    
    # State variables
    in_mission_block = False

    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            
            # Check for blank line to toggle the in_mission_block state
            if not line:
                in_mission_block = not in_mission_block
                continue  # Skip the blank line

            # Only process lines if we're in the mission block
            if only_read_mission and not in_mission_block:
                continue
            
            data = line.split()
            if len(data) == 95: # Adjusted length check for 3 drones
                time.append(float(data[0]))
                pos_load_rel_world_desired.append([float(data[1]), float(data[2]), float(data[3])])
                rpy_load_rel_world_desired.append([float(data[4]), float(data[5]), float(data[6])])
                pos_load_rel_world_gt.append([float(data[7]), float(data[8]), float(data[9])])
                rpy_load_rel_world_gt.append([float(data[10]), float(data[11]), float(data[12])])
                pos_err_load.append([float(data[13]), float(data[14]), float(data[15])])
                att_err_load.append([float(data[16]), float(data[17]), float(data[18])])
                distTransLoad.append(float(data[19]))
                distAngGeoLoad.append(float(data[20]))

                # Quasi-static load pose data
                pos_load_rel_world_qs.append([float(data[21]), float(data[22]), float(data[23])])
                rpy_load_rel_world_qs.append([float(data[24]), float(data[25]), float(data[26])])
                qs_pos_diff_load.append([float(data[27]), float(data[28]), float(data[29])])
                qs_att_diff_load.append([float(data[30]), float(data[31]), float(data[32])])
                distTransLoadQs.append(float(data[33]))
                distAngGeoLoadQs.append(float(data[34]))

                # Loop for drone data
                # Added nested list to store all drones
                pos_drones_rel_world_desired.append([])
                rpy_drones_rel_world_desired.append([])
                pos_drones_rel_world_gt.append([])
                rpy_drones_rel_world_gt.append([])
                pos_err_drones.append([])
                att_err_drones.append([])
                distTransDrones.append([])
                distAngGeoDrones.append([])
                

                for i in range(3): # 3 drones
                    base_index = 35 + 20*i
                    
                    pos_drones_rel_world_desired[-1].append([float(data[base_index]), float(data[base_index + 1]), float(data[base_index + 2])])
                    rpy_drones_rel_world_desired[-1].append([float(data[base_index + 3]), float(data[base_index + 4]), float(data[base_index + 5])])
                    pos_drones_rel_world_gt[-1].append([float(data[base_index + 6]), float(data[base_index + 7]), float(data[base_index + 8])])
                    rpy_drones_rel_world_gt[-1].append([float(data[base_index + 9]), float(data[base_index + 10]), float(data[base_index + 11])])
                    pos_err_drones[-1].append([float(data[base_index + 12]), float(data[base_index + 13]), float(data[base_index + 14])])
                    att_err_drones[-1].append([float(data[base_index + 15]), float(data[base_index + 16]), float(data[base_index + 17])])
                    distTransDrones[-1].append(float(data[base_index + 18]))
                    distAngGeoDrones[-1].append(float(data[base_index + 19]))
                    

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

    pos_load_rel_world_qs = np.array(pos_load_rel_world_qs)
    rpy_load_rel_world_qs = np.array(rpy_load_rel_world_qs)
    qs_pos_diff_load = np.array(qs_pos_diff_load)
    qs_att_diff_load = np.array(qs_att_diff_load)
    distTransLoadQs = np.array(distTransLoadQs)
    distAngGeoLoadQs = np.array(distAngGeoLoadQs)

    pos_drones_rel_world_desired = np.array(pos_drones_rel_world_desired) #.reshape(-1, 3, 3) # TODO: Confirm data is resized in correct way
    rpy_drones_rel_world_desired = np.array(rpy_drones_rel_world_desired) #.reshape(-1, 3, 3)
    pos_drones_rel_world_gt = np.array(pos_drones_rel_world_gt) #.reshape(-1, 3, 3)
    rpy_drones_rel_world_gt = np.array(rpy_drones_rel_world_gt) #.reshape(-1, 3, 3)
    pos_err_drones = np.array(pos_err_drones) #.reshape(-1, 3, 3)
    att_err_drones = np.array(att_err_drones) #.reshape(-1, 3, 3)
    distTransDrones = np.array(distTransDrones) #.reshape(-1, 3, 1)
    distAngGeoDrones = np.array(distAngGeoDrones) #.reshape(-1, 3, 1)

    return (time, pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_load_rel_world_gt, rpy_load_rel_world_gt,
            pos_err_load, att_err_load, distTransLoad, distAngGeoLoad,
            pos_load_rel_world_qs, rpy_load_rel_world_qs, qs_pos_diff_load, qs_att_diff_load, distTransLoadQs, distAngGeoLoadQs,
            pos_drones_rel_world_desired, rpy_drones_rel_world_desired, pos_drones_rel_world_gt, rpy_drones_rel_world_gt,
            pos_err_drones, att_err_drones, distTransDrones, distAngGeoDrones)


def plot_load_data(time, pos_load_rel_world_gt, rpy_load_rel_world_gt, pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_load_rel_world_qs, rpy_load_rel_world_qs,
                   title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(15, 10))

    # Position and Ground Truth Position
    plt.subplot(2, 1, 1)
    pos_load_rel_world_gt = list(zip(*pos_load_rel_world_gt))
    pos_load_rel_world_desired = list(zip(*pos_load_rel_world_desired))
    plt.plot(time, pos_load_rel_world_gt[0], label='pos_load_gt_x', color='red')
    plt.plot(time, pos_load_rel_world_gt[1], label='pos_load_gt_y', color='green')
    plt.plot(time, pos_load_rel_world_gt[2], label='pos_load_gt_z', color='blue')

    plt.plot(time, pos_load_rel_world_desired[0], label='pos_load_desired_x', linestyle='dashed', color='red')
    plt.plot(time, pos_load_rel_world_desired[1], label='pos_load_desired_y', linestyle='dashed', color='green')
    plt.plot(time, pos_load_rel_world_desired[2], label='pos_load_desired_z', linestyle='dashed', color='blue')

    plt.plot(time, pos_load_rel_world_qs[0], label='pos_load_quasi-static_x', linestyle='dotted', color='red')
    plt.plot(time, pos_load_rel_world_qs[1], label='pos_load_quasi-static_y', linestyle='dotted', color='green')
    plt.plot(time, pos_load_rel_world_qs[2], label='pos_load_quasi-static_z', linestyle='dotted', color='blue')

    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position (m)', fontsize=axes_label_font_size)
    plt.title('Load Position - Ground Truth vs Desired', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # RPY and Ground Truth RPY
    plt.subplot(2, 1, 2)
    rpy_load_rel_world_gt = np.array(rpy_load_rel_world_gt).T
    rpy_load_rel_world_desired = np.array(rpy_load_rel_world_desired).T
    for i in range(3):
        plt.plot(time, unwrap_and_convert_to_degrees(rpy_load_rel_world_gt[i]), label=f'rpy_load_gt_{["roll", "pitch", "yaw"][i]}', color=f'{["red", "green", "blue"][i]}')
        plt.plot(time, unwrap_and_convert_to_degrees(rpy_load_rel_world_desired[i]), label=f'rpy_load_desired_{["roll", "pitch", "yaw"][i]}', linestyle='dashed', color=f'{["red", "green", "blue"][i]}')
        plt.plot(time, unwrap_and_convert_to_degrees(rpy_load_rel_world_qs[i]), label=f'rpy_load_quasi-static_{["roll", "pitch", "yaw"][i]}', linestyle='dotted', color=f'{["red", "green", "blue"][i]}')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation (Degrees)', fontsize=axes_label_font_size)
    plt.title('Load Orientation - Ground Truth vs Desired', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_drones_data(time, pos_drones_rel_world_gt, rpy_drones_rel_world_gt, pos_drones_rel_world_desired, rpy_drones_rel_world_desired,
                     title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(15, 10))

    # Drone Position 
    plt.subplot(2, 1, 1)
    #pos_drones_rel_world_gt = np.array(pos_drones_rel_world_gt).T
    for i in range(3):
        # Position gt
        plt.plot(time, [pos_drone_i_fixed_t[i][0] for pos_drone_i_fixed_t in pos_drones_rel_world_gt], label=f'pos_drone_{i+1}_gt_x', color=f'C{3*i}') #TODO: Perhaps not in correct order??
        plt.plot(time, [pos_drone_i_fixed_t[i][1] for pos_drone_i_fixed_t in pos_drones_rel_world_gt], label=f'pos_drone_{i+1}_gt_y', color=f'C{3*i+1}')
        plt.plot(time, [pos_drone_i_fixed_t[i][2] for pos_drone_i_fixed_t in pos_drones_rel_world_gt], label=f'pos_drone_{i+1}_gt_z', color=f'C{3*i+2}')
        
        # Position desired
        plt.plot(time, [pos_drone_i_fixed_t[i][0] for pos_drone_i_fixed_t in pos_drones_rel_world_desired], label='_nolegend_', linestyle='dashed', color=f'C{3*i}') #label=f'pos_drone_{i+1}_d_x', linestyle='dashed', color=f'C{i}')
        plt.plot(time, [pos_drone_i_fixed_t[i][1] for pos_drone_i_fixed_t in pos_drones_rel_world_desired], label='_nolegend_', linestyle='dashed', color=f'C{3*i+1}') #label=f'pos_drone_{i+1}_d_y', linestyle='dashed', color=f'C{i+1}')
        plt.plot(time, [pos_drone_i_fixed_t[i][2] for pos_drone_i_fixed_t in pos_drones_rel_world_desired], label='_nolegend_', linestyle='dashed', color=f'C{3*i+2}') #label=f'pos_drone_{i+1}_d_z', linestyle='dashed', color=f'C{i+2}')
    
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position (m)', fontsize=axes_label_font_size)
    plt.title('Drone Positions Relative to the World', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # Drone orientation
    plt.subplot(2, 1, 2)
    
    for i in range(3):
        # Att gt 
        plt.plot(time, unwrap_and_convert_to_degrees([drone_i_fixed_t[i][0] for drone_i_fixed_t in rpy_drones_rel_world_gt]), label=f'att_drone_{i+1}_gt_r', color=f'C{3*i}') #TODO: Perhaps not in correct order??
        plt.plot(time, unwrap_and_convert_to_degrees([drone_i_fixed_t[i][1] for drone_i_fixed_t in rpy_drones_rel_world_gt]), label=f'att_drone_{i+1}_gt_p', color=f'C{3*i+1}')
        plt.plot(time, unwrap_and_convert_to_degrees([drone_i_fixed_t[i][2] for drone_i_fixed_t in rpy_drones_rel_world_gt]), label=f'att_drone_{i+1}_gt_y', color=f'C{3*i+2}')
        
        # Att desired
        plt.plot(time, unwrap_and_convert_to_degrees([drone_i_fixed_t[i][0] for drone_i_fixed_t in rpy_drones_rel_world_desired]), label='_nolegend_', linestyle='dashed', color=f'C{3*i}') #label=f'att_drone_{i+1}_d_r', linestyle='dashed', color=f'C{i}')
        plt.plot(time, unwrap_and_convert_to_degrees([drone_i_fixed_t[i][1] for drone_i_fixed_t in rpy_drones_rel_world_desired]), label='_nolegend_', linestyle='dashed', color=f'C{3*i+1}') #label=f'att_drone_{i+1}_d_p', linestyle='dashed', color=f'C{i+1}')
        plt.plot(time, unwrap_and_convert_to_degrees([drone_i_fixed_t[i][2] for drone_i_fixed_t in rpy_drones_rel_world_desired]), label='_nolegend_', linestyle='dashed', color=f'C{3*i+2}') #label=f'att_drone_{i+1}_d_y', linestyle='dashed', color=f'C{i+2}')
    

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
    plt.plot(time, pos_err_load[0], label='pos_err_load_x', color='red')
    plt.plot(time, pos_err_load[1], label='pos_err_load_y', color='green')
    plt.plot(time, pos_err_load[2], label='pos_err_load_z', color='blu')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position Error (m)', fontsize=axes_label_font_size)
    plt.title('Load Position Error', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # Load Attitude Errors
    plt.subplot(2, 1, 2)
    att_err_load = list(zip(*att_err_load))
    plt.plot(time, unwrap_and_convert_to_degrees(att_err_load[0]), label='att_err_load_roll', color='red')
    plt.plot(time, unwrap_and_convert_to_degrees(att_err_load[1]), label='att_err_load_pitch', color='green')
    plt.plot(time, unwrap_and_convert_to_degrees(att_err_load[2]), label='att_err_load_yaw', color='blue')
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


def plot_3d_trajectories(time, pos_load_rel_world_gt, pos_load_rel_world_desired, pos_load_rel_world_qs,
                                          pos_drones_rel_world_gt, pos_drones_rel_world_desired,
                                          title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    fig = plt.figure(figsize=(18, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_title('Trajectories of Load and Drones', fontsize=title_font_size)
    ax.set_xlabel('X (m)', fontsize=axes_label_font_size)
    ax.set_ylabel('Y (m)', fontsize=axes_label_font_size)
    ax.set_zlabel('Z (m)', fontsize=axes_label_font_size)
    
    # Plot Load Trajectories
    ax.plot(pos_load_rel_world_gt[:, 0], pos_load_rel_world_gt[:, 1], pos_load_rel_world_gt[:, 2], #pos_load_rel_world_gt[:, 0], pos_load_rel_world_gt[:, 1], pos_load_rel_world_gt[:, 2],
            label='Load ground truth', color='r')
    ax.plot(pos_load_rel_world_desired[:, 0], pos_load_rel_world_desired[:, 1], pos_load_rel_world_desired[:, 2],
            label='Load desired', color='r', linestyle='dashed')
    ax.plot(pos_load_rel_world_qs[:, 0], pos_load_rel_world_qs[:, 1], pos_load_rel_world_qs[:, 2],
            label='Load quasi-static', color='r', linestyle='dotted')

    # Plot Drone Trajectories
    colors = ['green', 'blue', 'purple']
    for i in range(3):
        ax.plot(pos_drones_rel_world_gt[:, i, 0], pos_drones_rel_world_gt[:, i, 1], pos_drones_rel_world_gt[:, i, 2],
                label=f'Drone {i+1} ground truth', color=colors[i])
        ax.plot(pos_drones_rel_world_desired[:, i, 0], pos_drones_rel_world_desired[:, i, 1], pos_drones_rel_world_desired[:, i, 2],
                label=f'Drone {i+1} desired', color=colors[i], linestyle='dashed')

    ax.legend(fontsize=legend_font_size)
    
    plt.tight_layout()
    plt.show()



def unwrap_and_convert_to_degrees(data): 
    data_unwrapped = np.unwrap(data) # Unwrap phases in radians np.radians(data)
    return np.degrees(data_unwrapped) # Convert radians to degrees

def main():
    # Read data
    # Retrieve data from log file
    path = '/home/harvey/ws_ros2/src/slung_pose_estimation/src/' #'/home/harvey/px4_ros_com_ros2/install/slung_pose_estimation/share/slung_pose_estimation/data/'
    filename = path + '2024_09_02_14_55_56_logger1.txt' #'2024_09_02_12_57_29_logger1.txt' #'2024_09_02_12_39_56_logger1.txt' # replace with your log file path
    only_plot_mission_phase = False #True


    (time, pos_load_rel_world_desired, rpy_load_rel_world_desired, pos_load_rel_world_gt, rpy_load_rel_world_gt,
    pos_err_load, att_err_load, distTransLoad, distAngGeoLoad,
    pos_load_rel_world_qs, rpy_load_rel_world_qs, qs_pos_diff_load, qs_att_diff_load, distTransLoadQs, distAngGeoLoadQs,
    pos_drones_rel_world_desired, rpy_drones_rel_world_desired, pos_drones_rel_world_gt, rpy_drones_rel_world_gt,
    pos_err_drones, att_err_drones, distTransDrones, distAngGeoDrones) = read_log_file(filename, only_read_mission=only_plot_mission_phase)

    # Plot settings
    title_font_size = 18 #14
    axes_label_font_size = 18 #12
    legend_font_size = 14 #10
    ticks_font_size = 18 #10

    # Plot Data

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
    # plot_drones_data(
    #     time, 
    #     pos_drones_rel_world_gt, 
    #     rpy_drones_rel_world_gt, 
    #     pos_drones_rel_world_desired, 
    #     rpy_drones_rel_world_desired,
    #     title_font_size=title_font_size, 
    #     axes_label_font_size=axes_label_font_size, 
    #     legend_font_size=legend_font_size, 
    #     ticks_font_size=ticks_font_size
    # )

    # Plot Errors
    # plot_errors(time, pos_err_load, att_err_load, pos_err_drones, att_err_drones,
                # title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)

    plot_3d_trajectories(time, pos_load_rel_world_gt, pos_load_rel_world_desired, pos_load_rel_world_qs,
                                        pos_drones_rel_world_gt, pos_drones_rel_world_desired,
                                        title_font_size=16, axes_label_font_size=12, legend_font_size=10, ticks_font_size=8)
    

if __name__ == '__main__':
    main()