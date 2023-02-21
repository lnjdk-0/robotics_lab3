import numpy as np

def dh_transformation(a, alpha, d, theta):
    """
    Returns a homogeneous transformation matrix using the Denavit-Hartenberg convention.

    a: float, Denavit-Hartenberg parameter for the distance between the z-axes of adjacent joints
    alpha: float, Denavit-Hartenberg parameter for the twist angle between the z-axes of adjacent joints
    d: float, Denavit-Hartenberg parameter for the distance between the x-axes of adjacent joints
    theta: float, Denavit-Hartenberg parameter for the angle between the x-axes of adjacent joints

    Returns:
    H: numpy.ndarray, 4x4 homogeneous transformation matrix that represents the transformation from
       the coordinate frame of the previous joint to the coordinate frame of the current joint
    """

    # Calculate the cosine and sine of the theta and alpha angles
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    # Construct the homogeneous transformation matrix using the DH parameters
    H = np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

    return H
    
def kinematic_chain(dh_params):
    """
    Returns the homogeneous transformation matrix for the entire kinematic chain.
    """
    H = np.identity(4)

    # Iterate through each row of DH parameters and compute the corresponding homogeneous transformation matrix
    for params in dh_params:
        a, alpha, d, theta = params
        H = np.dot(H, dh_transformation(a, alpha, d, theta))

    return H
    
def get_pos(H):
    """
    Returns the x, y, z components of the position from a homogeneous transformation matrix.
    """
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]

    return x, y, z
    

def get_rot(H):
    """
    Returns the roll-pitch-yaw angles from a homogeneous transformation matrix.
    """
    # Extract the rotation sub-matrix from the homogeneous transformation matrix
    R = H[:3, :3]

    # Calculate the roll, pitch, and yaw angles
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    roll = np.arctan2(R[2, 1], R[2, 2])

    return roll, pitch, yaw
