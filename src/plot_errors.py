import matplotlib.pyplot as plt

def read_data_from_file(filename):
    times, distTrans, distAng = [], [], []

    with open(filename, 'r') as file:
        for line in file:
            data = line.split()
            if len(data) == 3:
                time, trans, ang = data
                times.append(float(time))
                distTrans.append(float(trans))
                distAng.append(float(ang))

    return times, distTrans, distAng

def plot_data(times, distTrans, distAng):
    plt.figure(figsize=(10, 5))

    # Plotting distTrans over time
    plt.subplot(1, 2, 1)
    plt.plot(times, distTrans, color='blue') #label='distTrans', 
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.title('Translation Estimation Error')
    plt.grid(True)
    plt.ylim(bottom=0.0)
    plt.legend()

    # Plotting distAng over time
    plt.subplot(1, 2, 2)
    plt.plot(times, distAng, color='red') #label='distAng', 
    plt.xlabel('Time (s)')
    plt.ylabel('Geodesic distance error (degrees)')
    plt.title('Orientation Estimation Error')
    plt.grid(True)
    plt.ylim(bottom=0.0)
    plt.legend()

    plt.tight_layout()
    plt.show()

def main():
    path = '/home/harvey/px4_ros_com_ros2/install/slung_pose_estimation/share/slung_pose_estimation/data/'
    filename = 'pnp_errors.txt'  # replace with your log file path
    times, distTrans, distAng = read_data_from_file(path + filename)
    plot_data(times - times[0], distTrans, distAng)

if __name__ == '__main__':
    main()