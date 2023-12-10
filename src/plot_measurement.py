import matplotlib.pyplot as plt
import math

def read_log_file(filename):
    time, pos_gt, rpy_gt, pos, rpy, pos_err, att_err, distTrans, distAngGeo = [], [], [], [], [], [], [], [], []
    
    with open(filename, 'r') as file:
        for line in file:
            data = line.split()
            if len(data) == 21:
                time.append(float(data[0]))
                pos_gt.append([float(data[1]), float(data[2]), float(data[3])])
                rpy_gt.append([float(data[4]), float(data[5]), float(data[6])])
                pos.append([float(data[7]), float(data[8]), float(data[9])])
                rpy.append([float(data[10]), float(data[11]), float(data[12])])
                pos_err.append([float(data[13]), float(data[14]), float(data[15])])
                att_err.append([float(data[16]), float(data[17]), float(data[18])])
                distTrans.append(float(data[19]))
                distAngGeo.append(float(data[20]))

    return time, pos_gt, rpy_gt, pos, rpy, pos_err, att_err, distTrans, distAngGeo

def plot_data(time, pos_gt, rpy_gt, pos, rpy, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size): #, pos_err, att_err, distTrans, distAngGeo):
    plt.figure(figsize=(15, 10))

    # Position and Ground Truth Position
    plt.subplot(2, 1, 1)
    pos_gt = list(zip(*pos_gt))
    pos = list(zip(*pos))
    plt.plot(time, pos_gt[0], label='pos_gt_x', color='blue')
    plt.plot(time, pos_gt[1], label='pos_gt_y', color='green')
    plt.plot(time, pos_gt[2], label='pos_gt_z', color='red')
    plt.plot(time, pos[0], label='pos_x', linestyle='dashed', color='blue')
    plt.plot(time, pos[1], label='pos_y', linestyle='dashed', color='green')
    plt.plot(time, pos[2], label='pos_z', linestyle='dashed', color='red')
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
    plt.plot(time, rpy_gt[0], label='rpy_gt_roll', color='blue')
    plt.plot(time, rpy_gt[1], label='rpy_gt_pitch', color='green')
    plt.plot(time, rpy_gt[2], label='rpy_gt_yaw', color='red')
    plt.plot(time, rpy[0], label='rpy_roll', linestyle='dashed', color='blue')
    plt.plot(time, rpy[1], label='rpy_pitch', linestyle='dashed', color='green')
    plt.plot(time, rpy[2], label='rpy_yaw', linestyle='dashed', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation (Degrees)', fontsize=axes_label_font_size)
    plt.title('Measured vs. Ground Truth Orientation of the Load Relative to the Camera Frame', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()


def plot_errors(time, pos_err, att_err, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(10, 5))

    # Position Errors
    plt.subplot(2, 1, 1)
    pos_err = list(zip(*pos_err))
    plt.plot(time, pos_err[0], label='pos_err_x', color='blue')
    plt.plot(time, pos_err[1], label='pos_err_y', color='green')
    plt.plot(time, pos_err[2], label='pos_err_z', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Position Error (m)', fontsize=axes_label_font_size)
    plt.title('Measurement Position Error Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # Orientation Errors
    plt.subplot(2, 1, 2)
    att_err = list(zip(*att_err))
    plt.plot(time, att_err[0], label='att_err_roll', color='blue')
    plt.plot(time, att_err[1], label='att_err_pitch', color='green')
    plt.plot(time, att_err[2], label='att_err_yaw', color='red')
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Orientation Error (Degrees)', fontsize=axes_label_font_size)
    plt.title('Measurement Orientation Error Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()

def plot_trans_geo(time, distTrans, distAngGeo, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size):
    plt.figure(figsize=(10, 5))

    # distTrans
    plt.subplot(2, 1, 1)
    plt.plot(time, distTrans, color='blue') #label='distTrans',
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Translation Distance (m)', fontsize=axes_label_font_size)
    plt.title('Distance Measurement Error Magnitude Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    # distAngGeo
    plt.subplot(2, 1, 2)
    plt.plot(time, distAngGeo, color='red') #label='distAngGeo',
    plt.xlabel('Time (s)', fontsize=axes_label_font_size)
    plt.ylabel('Geodesic Distance (Degrees)', fontsize=axes_label_font_size)
    plt.title('Geodesic Distance Attitude Measurement Error Over Time', fontsize=title_font_size)
    plt.legend(fontsize=legend_font_size)
    plt.xticks(fontsize=ticks_font_size)
    plt.yticks(fontsize=ticks_font_size)

    plt.tight_layout()
    plt.show()



def main():
    # Set parameters
    title_font_size = 16
    axes_label_font_size = 14
    legend_font_size = 12
    ticks_font_size = 10

    # Retrieve data from log file
    path = '/home/harvey/px4_ros_com_ros2/install/slung_pose_estimation/share/slung_pose_estimation/data/'
    filename = path + 'pnp_errors_p3p.txt'  # replace with your log file path
    time, pos_gt, rpy_gt, pos, rpy, pos_err, att_err, distTrans, distAngGeo = read_log_file(filename)
    
    # Convert
    time = [t - time[0] for t in time]
    rpy_gt = [[r * 180 / math.pi for r in rpy] for rpy in rpy_gt]
    rpy = [[r * 180 / math.pi for r in rpy] for rpy in rpy]
    att_err = [[r * 180 / math.pi for r in rpy] for rpy in att_err]
    #distAngGeo = [d * 180 / math.pi for d in distAngGeo]

    # Plot
    plot_data(time, pos_gt, rpy_gt, pos, rpy, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size) #, pos_err, att_err, distTrans, distAngGeo)
    plot_errors(time, pos_err, att_err, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)
    plot_trans_geo(time, distTrans, distAngGeo, title_font_size, axes_label_font_size, legend_font_size, ticks_font_size)

if __name__ == '__main__':
    main()