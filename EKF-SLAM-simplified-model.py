import numpy as np

# SLAM Variables
landmarks = []          # List to store landmarks [x, y]
robot_pose = np.zeros(3)  # Robot pose [x, y, theta]
motion_noise = [0.1, 0.1, 0.1]  # Motion noise parameters
measurement_noise = 0.1  # Measurement noise parameter

# EKF-SLAM Function
def ekf_slam(measurements, controls):
    global landmarks, robot_pose

    for t in range(len(measurements)):
        # Motion Update (Prediction)
        robot_pose = motion_update(robot_pose, controls[t])
        
        # Measurement Update
        for measurement in measurements[t]:
            landmark_id, range_measurement, bearing_measurement = measurement
            landmark = get_landmark(landmark_id)
            robot_pose, landmark = measurement_update(robot_pose, landmark, range_measurement, bearing_measurement)

            # Update or add the landmark to the landmark list
            if landmark_id not in [l[0] for l in landmarks]:
                landmarks.append([landmark_id, landmark])

        print("Robot Pose: ", robot_pose)
        print("Landmarks: ", landmarks)

# Motion Update (Prediction)
def motion_update(pose, control):
    delta_x = control[0]
    delta_y = control[1]
    delta_theta = control[2]
    
    x, y, theta = pose
    
    # Update the pose
    new_x = x + delta_x + np.random.normal(0, motion_noise[0])
    new_y = y + delta_y + np.random.normal(0, motion_noise[1])
    new_theta = theta + delta_theta + np.random.normal(0, motion_noise[2])

    return [new_x, new_y, new_theta]

# Measurement Update
def measurement_update(pose, landmark, range_measurement, bearing_measurement):
    x, y, theta = pose
    lx, ly = landmark
    
    # Calculate the expected range and bearing
    expected_range = np.sqrt((lx - x)**2 + (ly - y)**2)
    expected_bearing = np.arctan2(ly - y, lx - x) - theta
    
    # Compute the Jacobians for the measurement model
    H = np.array([
        [(lx - x) / expected_range, (ly - y) / expected_range, 0],
        [(y - ly) / (expected_range**2), (x - lx) / (expected_range**2), -1]
    ])

    # Compute the Kalman gain
    Q = np.diag([measurement_noise**2, measurement_noise**2])
    K = np.dot(np.dot(pose_covariance, H.T), np.linalg.inv(np.dot(np.dot(H, pose_covariance), H.T) + Q))
    
    # Update the pose estimate
    innovation = np.array([range_measurement - expected_range, bearing_measurement - expected_bearing])
    pose += np.dot(K, innovation)

    # Update the pose covariance
    pose_covariance = np.dot((np.eye(3) - np.dot(K, H)), pose_covariance)

    return pose, landmark

# Helper function to get landmark from the list
def get_landmark(landmark_id):
    for landmark in landmarks:
        if landmark[0] == landmark_id:
            return landmark[1]
    return None

# Test the EKF-SLAM implementation
measurements = [[(1, 2.5, 0.1), (2, 3.2, -0.3)], [(1, 2.0, 0.2), (3, 4.1, -0.2)]]
controls = [(0.5, 0.5, 0.1), (0.3, 0.2, -0.1)]
ekf_slam(measurements, controls)
