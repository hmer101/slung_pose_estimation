import matplotlib.pyplot as plt
import math
import numpy as np

def read_log_file(filename, only_read_mission=False):
    time, pos_gt, rpy_gt, pos, rpy, pos_err, att_err, distTrans, distAngGeo = [], [], [], [], [], [], [], [], []
    pos_marker_rel_cam_qs, rpy_marker_rel_cam_qs, pos_err_qs, att_err_qs, distTransQs, distAngGeoQs = [], [], [], [], [], []
    pos_drone_rel_world, rpy_drone_rel_world = [], []
    pos_load_rel_world, rpy_load_rel_world = [], []
    
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
            if len(data) == 47:  # Updated length for new columns
                time.append(float(data[0]))
                pos_gt.append([float(data[1]), float(data[2]), float(data[3])])
                rpy_gt.append([float(data[4]), float(data[5]), float(data[6])])
                pos.append([float(data[7]), float(data[8]), float(data[9])])
                rpy.append([float(data[10]), float(data[11]), float(data[12])])
                pos_err.append([float(data[13]), float(data[14]), float(data[15])])
                att_err.append([float(data[16]), float(data[17]), float(data[18])])
                distTrans.append(float(data[19]))
                distAngGeo.append(float(data[20]))

                pos_marker_rel_cam_qs.append([float(data[21]), float(data[22]), float(data[23])])
                rpy_marker_rel_cam_qs.append([float(data[24]), float(data[25]), float(data[26])])
                pos_err_qs.append([float(data[27]), float(data[28]), float(data[29])])
                att_err_qs.append([float(data[30]), float(data[31]), float(data[32])])
                distTransQs.append(float(data[33]))
                distAngGeoQs.append(float(data[34]))

                pos_load_rel_world.append([float(data[35]), float(data[36]), float(data[37])])
                rpy_load_rel_world.append([float(data[38]), float(data[39]), float(data[40])])
                pos_drone_rel_world.append([float(data[41]), float(data[42]), float(data[43])])
                rpy_drone_rel_world.append([float(data[44]), float(data[45]), float(data[46])])


    # Convert to np
    time = np.array(time) #[t - time[0] for t in time]
    pos_gt = np.array(pos_gt)
    rpy_gt = np.array(rpy_gt)
    pos = np.array(pos)
    rpy = np.array(rpy)
    pos_err = np.array(pos_err)
    att_err = np.array(att_err)
    distTrans = np.array(distTrans)
    distAngGeo = np.array(distAngGeo)

    pos_marker_rel_cam_qs = np.array(pos_marker_rel_cam_qs) 
    rpy_marker_rel_cam_qs = np.array(rpy_marker_rel_cam_qs) 
    pos_err_qs = np.array(pos_err_qs)
    att_err_qs = np.array(att_err_qs) 
    distTransQs = np.array(distTransQs) 
    distAngGeoQs = np.array(distAngGeoQs)

    pos_drone_rel_world = np.array(pos_drone_rel_world)
    rpy_drone_rel_world = np.array(rpy_drone_rel_world)
    pos_load_rel_world = np.array(pos_load_rel_world)
    rpy_load_rel_world = np.array(rpy_load_rel_world) 

    return time, pos_gt, rpy_gt, pos, rpy, pos_err, att_err, distTrans, distAngGeo, pos_marker_rel_cam_qs, rpy_marker_rel_cam_qs, pos_err_qs, att_err_qs, distTransQs, distAngGeoQs, pos_drone_rel_world, rpy_drone_rel_world, pos_load_rel_world, rpy_load_rel_world


def plot_data(time, pos_gt, rpy_gt, pos, rpy, pos_qs, rpy_qs, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size): #, pos_err, att_err, distTrans, distAngGeo):
    plt.figure(figsize=(15, 10))

    # Position and Ground Truth Position
    plt.subplot(2, 1, 1)
    pos_gt = list(zip(*pos_gt))
    pos = list(zip(*pos))
    pos_qs = list(zip(*pos_qs))

    plt.plot(time, pos_gt[0], label='pos_gt_x', color='red')
    plt.plot(time, pos_gt[1], label='pos_gt_y', color='green')
    plt.plot(time, pos_gt[2], label='pos_gt_z', color='blue')

    plt.plot(time, pos[0], label='pos_x', linestyle='dashed', color='red')
    plt.plot(time, pos[1], label='pos_y', linestyle='dashed', color='green')
    plt.plot(time, pos[2], label='pos_z', linestyle='dashed', color='blue')

    plt.plot(time, pos_qs[0], label='pos_qs_x', linestyle='dotted', color='red')
    plt.plot(time, pos_qs[1], label='pos_qs_y', linestyle='dotted', color='green')
    plt.plot(time, pos_qs[2], label='pos_qs_z', linestyle='dotted', color='blue')

    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position (m)', fontsize=axes_label_font_size)
    plt.title('Measured vs. Ground Truth Position of the Load Relative to the Camera Frame', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # RPY and Ground Truth RPY
    plt.subplot(2, 1, 2)
    rpy_gt = list(zip(*rpy_gt))
    rpy = list(zip(*rpy))
    rpy_qs = list(zip(*rpy_qs))

    plt.plot(time, rpy_gt[0], label='rpy_gt_roll', color='red')
    plt.plot(time, rpy_gt[1], label='rpy_gt_pitch', color='green')
    plt.plot(time, rpy_gt[2], label='rpy_gt_yaw', color='blue')

    plt.plot(time, rpy[0], label='rpy_roll', linestyle='dashed', color='red')
    plt.plot(time, rpy[1], label='rpy_pitch', linestyle='dashed', color='green')
    plt.plot(time, rpy[2], label='rpy_yaw', linestyle='dashed', color='blue')

    plt.plot(time, rpy_qs[0], label='rpy_qs_roll', linestyle='dotted', color='red')
    plt.plot(time, rpy_qs[1], label='rpy_qs_pitch', linestyle='dotted', color='green')
    plt.plot(time, rpy_qs[2], label='rpy_qs_yaw', linestyle='dotted', color='blue')

    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation (Degrees)', fontsize=axes_label_font_size)
    plt.title('Measured vs. Ground Truth Orientation of the Load Relative to the Camera Frame', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_errors(time, pos_err, att_err, pos_err_qs, att_err_qs, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(10, 5))

    # Position Errors
    plt.subplot(2, 1, 1)
    pos_err = list(zip(*pos_err))
    plt.plot(time, pos_err[0], label='pos_err_x', color='red')
    plt.plot(time, pos_err[1], label='pos_err_y', color='green')
    plt.plot(time, pos_err[2], label='pos_err_z', color='blue')

    pos_err_qs = list(zip(*pos_err_qs))
    plt.plot(time, pos_err_qs[0], label='pos_err_qs_x', linestyle='dotted', color='red')
    plt.plot(time, pos_err_qs[1], label='pos_err_qs_y', linestyle='dotted', color='green')
    plt.plot(time, pos_err_qs[2], label='pos_err_qs_z', linestyle='dotted', color='blue')

    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position Error (m)', fontsize=axes_label_font_size)
    plt.title('Measurement Position Error Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # Orientation Errors
    plt.subplot(2, 1, 2)
    att_err = list(zip(*att_err))
    plt.plot(time, att_err[0], label='att_err_roll', color='red')
    plt.plot(time, att_err[1], label='att_err_pitch', color='green')
    plt.plot(time, att_err[2], label='att_err_yaw', color='blue')

    att_err_qs = list(zip(*att_err_qs))
    plt.plot(time, att_err_qs[0], label='att_err_qs_roll', linestyle='dotted', color='red')
    plt.plot(time, att_err_qs[1], label='att_err_qs_pitch', linestyle='dotted', color='green')
    plt.plot(time, att_err_qs[2], label='att_err_qs_yaw', linestyle='dotted', color='blue')

    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation Error (Degrees)', fontsize=axes_label_font_size)
    plt.title('Measurement Orientation Error Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_trans_geo(time, distTrans, distAngGeo, distTransQs, distAngGeoQs, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(10, 5))

    # distTrans
    plt.subplot(2, 1, 1)
    plt.plot(time, distTrans, color='red') #label='distTrans',
    plt.plot(time, distTransQs, linestyle='dotted', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Translation Distance (m)', fontsize=axes_label_font_size)
    plt.title('Distance Measurement Error Magnitude Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # distAngGeo
    plt.subplot(2, 1, 2)
    plt.plot(time, distAngGeo, color='red') #label='distAngGeo',
    plt.plot(time, distAngGeoQs, linestyle='dotted', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Geodesic Distance (Degrees)', fontsize=axes_label_font_size)
    plt.title('Geodesic Distance Attitude Measurement Error Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_trajectory(time, pos_drone_rel_world, rpy_drone_rel_world, pos_load_rel_world, rpy_load_rel_world, 
                    title_font_size=16, axes_label_font_size=14, legend_font_size=12, ticks_font_size=10):
    plt.figure(figsize=(15, 10))

    # Position of Drone and Load Relative to the World
    plt.subplot(2, 1, 1)
    pos_drone_rel_world = list(zip(*pos_drone_rel_world))
    pos_load_rel_world = list(zip(*pos_load_rel_world))
    plt.plot(time, pos_drone_rel_world[0], label='drone_pos_x', color='blue')
    plt.plot(time, pos_drone_rel_world[1], label='drone_pos_y', color='green')
    plt.plot(time, pos_drone_rel_world[2], label='drone_pos_z', color='red')
    plt.plot(time, pos_load_rel_world[0], label='load_pos_x', linestyle='dashed', color='blue')
    plt.plot(time, pos_load_rel_world[1], label='load_pos_y', linestyle='dashed', color='green')
    plt.plot(time, pos_load_rel_world[2], label='load_pos_z', linestyle='dashed', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position (m)', fontsize=axes_label_font_size)
    plt.title('Position of Drone and Load Relative to the World', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # Orientation (RPY) of Drone and Load Relative to the World
    plt.subplot(2, 1, 2)
    rpy_drone_rel_world = list(zip(*rpy_drone_rel_world))
    rpy_load_rel_world = list(zip(*rpy_load_rel_world))
    plt.plot(time, rpy_drone_rel_world[0], label='drone_rpy_roll', color='blue')
    plt.plot(time, rpy_drone_rel_world[1], label='drone_rpy_pitch', color='green')
    plt.plot(time, rpy_drone_rel_world[2], label='drone_rpy_yaw', color='red')
    plt.plot(time, rpy_load_rel_world[0], label='load_rpy_roll', linestyle='dashed', color='blue')
    plt.plot(time, rpy_load_rel_world[1], label='load_rpy_pitch', linestyle='dashed', color='green')
    plt.plot(time, rpy_load_rel_world[2], label='load_rpy_yaw', linestyle='dashed', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation (Degrees)', fontsize=axes_label_font_size)
    plt.title('Orientation of Drone and Load Relative to the World', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()


# Convert all angles to degrees from a list of RPY lists
def rad_2_deg_rpy(rpy_rad):
    return [[r * 180 / math.pi for r in rpy] for rpy in rpy_rad]

# Unwrap phases and convert to degrees
def process_data(time, rpy_gt, rpy, rpy_qs):
    ## Perform data transformations
    # Set first time as 0 (note this may not be good when comparing between multiple drones' measurements)
    #time = time - time[0] 

    # Phase unwrap
    rpy_gt = np.unwrap(rpy_gt, axis=0)
    rpy = np.unwrap(rpy, axis=0)
    rpy_qs = np.unwrap(rpy_qs, axis=0)

     # Ensure rpy and rpy_gt are in the same phase for each column independently
    for i in range(rpy.shape[1]):
        phase_diff = np.mean(rpy_gt[:, i] - rpy[:, i])
        phase_diff_qs = np.mean(rpy_gt[:, i] - rpy_qs[:, i])
        
        phase_adjustment = round(phase_diff / (2 * np.pi)) * (2 * np.pi)
        phase_adjustment_qs = round(phase_diff_qs / (2 * np.pi)) * (2 * np.pi)
        
        rpy[:, i] += phase_adjustment
        rpy_qs[:, i] += phase_adjustment_qs
    
    # Calculate attitude error
    att_err = rpy - rpy_gt
    att_err_qs = rpy - rpy_qs
    
    # Convert to deg
    rpy_gt = rpy_gt*180 / math.pi
    rpy = rpy*180 / math.pi
    rpy_qs = rpy_qs*180 / math.pi

    att_err = att_err*180 / math.pi
    att_err_qs = att_err_qs*180 / math.pi

    return rpy_gt, rpy, rpy_qs, att_err, att_err_qs #time

def main():
    # Set parameters
    title_font_size = 16
    axes_label_font_size = 14
    legend_font_size = 12
    ticks_font_size = 10

    #num_drones = 1
    #start_drone_num = 1
    plot_data_for = [1] #, 2, 3] #[1, 2, 3] # Drones to plot data for
    start_drone_num = min(plot_data_for)
    num_drones = len(plot_data_for)

    # Retrieve data from log file
    path = '/home/harvey/ws_ros2/src/slung_pose_estimation/src/' #'/home/harvey/ws_ros2/install/slung_pose_estimation/share/slung_pose_estimation/data/'
    filename_common = path + '2024_09_05_19_54_54_measurement_drone' # replace with your log file path
    only_plot_mission_phase = True

    # Store data from all drones
    time = [None]*num_drones # Get starting times from all drones to align the data
    pos_gt = [None]*num_drones
    rpy_gt = [None]*num_drones
    pos = [None]*num_drones
    rpy = [None]*num_drones
    pos_err = [None]*num_drones
    att_err = [None]*num_drones
    distTrans = [None]*num_drones
    distAngGeo = [None]*num_drones
    
    pos_marker_rel_cam_qs = [None]*num_drones 
    rpy_marker_rel_cam_qs = [None]*num_drones
    pos_err_qs = [None]*num_drones 
    att_err_qs = [None]*num_drones
    distTransQs = [None]*num_drones 
    distAngGeoQs = [None]*num_drones

    pos_drone_rel_world = [None]*num_drones
    rpy_drone_rel_world = [None]*num_drones
    pos_load_rel_world = [None]*num_drones
    rpy_load_rel_world = [None]*num_drones

    for i in range(num_drones):
        filename = filename_common + str(i+start_drone_num) + '.txt'
        time[i], pos_gt[i], rpy_gt[i], pos[i], rpy[i], pos_err[i], att_err[i], distTrans[i], distAngGeo[i], pos_marker_rel_cam_qs[i], rpy_marker_rel_cam_qs[i], pos_err_qs[i], att_err_qs[i], distTransQs[i], distAngGeoQs[i], pos_drone_rel_world[i], rpy_drone_rel_world[i], pos_load_rel_world[i], rpy_load_rel_world[i] = read_log_file(filename, only_read_mission=only_plot_mission_phase)
    
    # Find start time
    time_start = min([times[0] for times in time])
    
    # Plot data for all drones selected to do so, relative to start time 
    plot_drone_indicies = [drone_num - start_drone_num for drone_num in plot_data_for] 
    for i in plot_drone_indicies:
        time[i] = time[i] - time_start
        rpy_gt_proc, rpy_proc, rpy_qs_proc, att_err_proc, att_err_qs_proc = process_data(time[i], rpy_gt[i], rpy[i], rpy_marker_rel_cam_qs[i])
        plot_data(time[i], pos_gt[i], rpy_gt_proc, pos[i], rpy_proc, pos_marker_rel_cam_qs[i], rpy_qs_proc, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)
        plot_errors(time[i], pos_err[i], att_err_proc, pos_err_qs[i], att_err_qs_proc, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)
        plot_trans_geo(time[i], distTrans[i], distAngGeo[i], distTransQs[i], distAngGeoQs[i], title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)
        # plot_trajectory(time[i], pos_drone_rel_world[i], rpy_drone_rel_world[i], pos_load_rel_world[i], rpy_load_rel_world[i], 
        #             title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)

        # Drone number
        print(f'Drone {i+start_drone_num}')
              
        # Get conglomorate metrics for distance from quasi-static
        print(f'Mean translational measurement error magnitude from ground truth: {np.mean(distTrans[i])} m')
        print(f'Mean translational distance magnitude of quasi-static from ground truth: {np.mean(distTransQs[i])} m')
        
        print(f'Mean geodesic attitude measurement error from ground truth: {np.mean(distAngGeo[i])} deg')
        print(f'Mean geodesic attitude distance of quasi-static from ground truth: {np.mean(distAngGeoQs[i])} deg')


if __name__ == '__main__':
    main()