import numpy as np

def euler_to_rot_matrix(roll, pitch, yaw):
    """
    Converts Euler angles in radians to a rotation matrix
    """
    R_x = np.array([[ 1,              0,              0               ],
                    [ 0,              np.cos(roll),  -np.sin(roll)    ],
                    [ 0,              np.sin(roll),   np.cos(roll)    ]])
        
    R_y = np.array([[ np.cos(pitch),  0,              np.sin(pitch)   ],
                    [ 0,              1,              0               ],
                    [-np.sin(pitch),  0,              np.cos(pitch)   ]])
                
    R_z = np.array([[ np.cos(yaw),   -np.sin(yaw),    0               ],
                    [ np.sin(yaw),    np.cos(yaw),    0               ],
                    [ 0,              0,              1               ]])           
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

def transform_matrix(origin, euler_angles):
    """
    Create a transformation matrix from a point and Euler angles
    """
    R = euler_to_rot_matrix(*euler_angles)
    T = np.zeros((4, 4))
    T[:3, :3] = R
    T[:3, 3] = origin
    T[3, 3] = 1
    return T

def transform_between_frames(
        frame1_origin, frame1_euler_angles, frame2_origin, frame2_euler_angles):
    """
    Returns the transformation matrix from frame1 to frame2
    """
    T1 = transform_matrix(frame1_origin, frame1_euler_angles)
    T2 = transform_matrix(frame2_origin, frame2_euler_angles)
    # Compute the transformation from frame1 to frame2
    T12 = np.dot(np.linalg.inv(T1), T2)
    return T12

def apply_transformation(T, point):
    """
    Apply transformation matrix to a point
    """
    point = np.append(point, 1)  # make it homogenous
    transformed_point = np.dot(T, point)
    return transformed_point[:3]  # return in non-homogenous form