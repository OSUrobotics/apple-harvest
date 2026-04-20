import numpy as np
from scipy.spatial.transform import Rotation as R

position = np.array([ 0.355001,-0.184258,1.534915])
quaternion = [-0.697140, -0.118307, 0.118307, 0.697140]

desired_transform = [
    [0, -1, 0, 0.015],
    [0, 0, -1, 0.0],
    [1, 0, 0, -0.0],
    [0, 0, 0, 1]
]

# Transformation matrix to rotate from optical frame to mount frame
def transform_point(position, quaternion, transform):
    rot = R.from_quat(quaternion)
    rot_matrix = rot.as_matrix()

    # Create a 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[0:3, 0:3] = rot_matrix
    transformation_matrix[0:3, 3] = position

    # Apply the desired transformation
    result_matrix = np.dot(transformation_matrix, transform)
    result_position = result_matrix[0:3, 3]
    result_rotation = R.from_matrix(result_matrix[0:3, 0:3])
    result_quaternion = result_rotation.as_quat()
    return result_position, result_quaternion

new_position, new_quaternion = transform_point(position, quaternion, desired_transform)
print("New Position:", new_position)
print("New roll-pitch-yaw:", R.from_quat(new_quaternion).as_euler('xyz'))