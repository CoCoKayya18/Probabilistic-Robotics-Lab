import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sklearn.metrics import mean_squared_error, mean_absolute_error
import atexit
import os

# Initialize global variables to store data
ground_truth_data = []
ekf_estimate_data = []
landmark_coords = (-2, 0)  # Single known landmark

# Callback for ground truth data
def ground_truth_callback(msg):
    global ground_truth_data
    timestamp = msg.header.stamp.to_sec()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ground_truth_data.append([timestamp, x, y])

# Callback for EKF estimate data (PoseStamped)
def ekf_estimate_callback(msg):
    global ekf_estimate_data
    timestamp = msg.header.stamp.to_sec()
    x = msg.pose.position.x
    y = msg.pose.position.y
    ekf_estimate_data.append([timestamp, x, y])

def compute_and_plot_metrics(save_dir=None):
    global ground_truth_data, ekf_estimate_data, landmark_coords

    # Convert to DataFrame for easier manipulation
    ground_truth_df = pd.DataFrame(ground_truth_data, columns=['timestamp', 'x_truth', 'y_truth'])
    ekf_estimate_df = pd.DataFrame(ekf_estimate_data, columns=['timestamp', 'x_ekf', 'y_ekf'])

    # Ensure timestamps are float
    ground_truth_df['timestamp'] = pd.to_numeric(ground_truth_df['timestamp'], errors='coerce', downcast="float")
    ekf_estimate_df['timestamp'] = pd.to_numeric(ekf_estimate_df['timestamp'], errors='coerce', downcast="float")

    # Merge data on timestamp
    data = pd.merge_asof(ground_truth_df, ekf_estimate_df, on='timestamp', direction='nearest')

    # Ensure the merged data columns are numeric
    data['x_truth'] = pd.to_numeric(data['x_truth'], errors='coerce')
    data['y_truth'] = pd.to_numeric(data['y_truth'], errors='coerce')
    data['x_ekf'] = pd.to_numeric(data['x_ekf'], errors='coerce')
    data['y_ekf'] = pd.to_numeric(data['y_ekf'], errors='coerce')

    # Drop rows with NaN values
    data.dropna(inplace=True)

    # Ensure there's data left after dropping NaNs
    if data.empty:
        print("No valid data to compute metrics.")
        return

    # Compute errors
    data['error_x'] = data['x_truth'] - data['x_ekf']
    data['error_y'] = data['y_truth'] - data['y_ekf']

    # Ensure the error columns are numeric
    data['error_x'] = pd.to_numeric(data['error_x'], errors='coerce')
    data['error_y'] = pd.to_numeric(data['error_y'], errors='coerce')

    # Compute the total error
    data['error'] = np.sqrt(data['error_x']**2 + data['error_y']**2)

    # Compute RMSE and MAE
    if len(data) > 0:
        rmse = np.sqrt(mean_squared_error(data[['x_truth', 'y_truth']], data[['x_ekf', 'y_ekf']]))
        mae = mean_absolute_error(data[['x_truth', 'y_truth']], data[['x_ekf', 'y_ekf']])
        print(f'RMSE: {rmse}')
        print(f'MAE: {mae}')
    else:
        rmse = mae = None
        print("No data to compute RMSE and MAE.")

    # Plot Trajectory Comparison
    if not data.empty:
        plt.figure(figsize=(10, 6))
        plt.plot(data['x_truth'].values, data['y_truth'].values, label='Ground Truth', color='blue')
        plt.plot(data['x_ekf'].values, data['y_ekf'].values, label='EKF Estimate', color='red')
        plt.scatter([landmark_coords[0]], [landmark_coords[1]], label='Landmark', color='green', marker='x', s=100)
        plt.legend()
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Trajectory Comparison with Landmark')
        if save_dir:
            plt.savefig(os.path.join(save_dir, 'trajectory_comparison_with_landmark.png'))
        # plt.show()

    # Plot Error Histogram
    if not data.empty:
        plt.figure(figsize=(10, 6))
        plt.hist(data['error'].values, bins=50, color='grey')
        plt.xlabel('Error (m)')
        plt.ylabel('Frequency')
        plt.title('Error Histogram')
        if save_dir:
            plt.savefig(os.path.join(save_dir, 'error_histogram.png'))
        # plt.show()

    # Plot Convergence (Error over time)
    if not data.empty:
        plt.figure(figsize=(10, 6))
        plt.plot(data['timestamp'].values, data['error'].values, color='green')
        plt.xlabel('Time')
        plt.ylabel('Error (m)')
        plt.title('Convergence Plot')
        if save_dir:
            plt.savefig(os.path.join(save_dir, 'convergence_plot.png'))
        # plt.show()

def save_plots_on_shutdown():
    save_dir = '/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/scripts/Plots'  # Replace with your desired directory
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    compute_and_plot_metrics(save_dir)

def listener():
    rospy.init_node('ekf_evaluation_node', anonymous=True)

    rospy.Subscriber('/ground_truth/state', Odometry, ground_truth_callback)
    rospy.Subscriber('/main_node/EKF_pose', PoseStamped, ekf_estimate_callback)

    # Register the shutdown function
    atexit.register(save_plots_on_shutdown)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
