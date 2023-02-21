import numpy as np
import robot_model as rm

np.set_printoptions(precision=3, suppress=True)

# Problem 2 A:
# Compute the kinematic chain for the manipulator
#H = rm.kinematic_chain([[dh_params[i][0], dh_params[i][1], dh_params[i][2], theta[i]] for i in range(len(dh_params))])
p1 = rm.dh_transformation(1, 0, 0, np.pi/2)
p2 = rm.dh_transformation(1, 0, 0, np.pi/2)
H = rm.kinematic_chain([p1, p2])

# Extract the position and orientation information from the homogeneous transformation matrix
x, y, z = rm.get_pos(H)
roll, pitch, yaw = rm.get_rot(H)

# Print the results
print(f"2A) Position: ({x}, {y}, {z})")
print(f"2A) Orientation: Roll={roll}, Pitch={pitch}, Yaw={yaw}")


# Problem 2 B:
# Case 1
# Define the DH parameters for the UR5e robot
dh_params = [rm.dh_transformation(0, np.pi / 2, 0.1625, 0),     #dh_transformation(a, alpha, d, theta)
            rm.dh_transformation(-0.425, 0, 0, 0), 
            rm.dh_transformation(-0.3922, 0, 0, 0),
            rm.dh_transformation(0, np.pi/2, 0.1333,0), 
            rm.dh_transformation(0, -np.pi/2, 0.0997, 0),
            rm.dh_transformation(0, 0, 0.0996, 0)]
# Compute the kinematic chain for the robot
H = rm.kinematic_chain(dh_params)

# Extract the position and orientation information from the homogeneous transformation matrix
x, y, z = rm.get_pos(H)
roll, pitch, yaw = rm.get_rot(H)

# Print the results for Case 1

print(f"2B) Case 1: Position: ({x}, {y}, {z})")
print(f"2B) Case 1: Orientation: Roll={roll}, Pitch={pitch}, Yaw={yaw}")

# Case 2
dh_params = [rm.dh_transformation(0, np.pi / 2, 0.1625, 0),     #dh_transformation(a, alpha, d, theta)
            rm.dh_transformation(-0.425, 0, 0, -np.pi / 2), 
            rm.dh_transformation(-0.3922, 0, 0, 0),
            rm.dh_transformation(0, np.pi/2, 0.1333,0), 
            rm.dh_transformation(0, -np.pi/2, 0.0997, 0),
            rm.dh_transformation(0, 0, 0.0996, 0)]

# Compute the kinematic chain for Case 2
H = rm.kinematic_chain(dh_params)

# Extract the position and orientation information from the homogeneous transformation matrix
x, y, z = rm.get_pos(H)
roll, pitch, yaw = rm.get_rot(H)

print(f"2B) Case 2: Position: ({x}, {y}, {z})")
print(f"2B) Case 2: Orientation: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
