#!/usr/bin/env python3

import numpy as np
from laser_to_pointcloud import *

class CloudTransform():

    def transform_to_quat_vec(self, t):
        '''
        Makes a numpy array consisting of the quaternion and translation vector from the tf2 transform.
        @param t: Incoming transform.
        @type t: geometry_msgs.msg.TransformStamped
        '''
        q = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        v = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        qv = [q,v]
        return qv

    def generate_transform(self, transform):
        '''
        Generates a transformation matrix given a TransformStamped containing a quaternion and vector.
        @param transform: Transform message from which a transformation matrix is to be generated.
        @type transform: geometry_msgs.msg.TransformStamped
        '''
        qv = self.transform_to_quat_vec(transform)

        q = qv[0]
        v = qv[1]

        # Generates a transform matrix from the unit quaternion and displacement vector.
        # Note the sign of the x-coordinate of the vector.
        T = np.array([[q[1]**2 + q[2]**2 - q[3]**2 - q[0]**2, 2*(q[2]*q[3] - q[1]*q[0]), 2*(q[2]*q[0] + q[1]*q[3]),                                       -v[0]],
                      [2*(q[2]*q[3] + q[1]*q[0]),                                        q[1]**2 - q[2]**2 + q[3]**2 - q[0]**2, 2*(q[3]*q[0] - q[1]*q[2]),v[1]],
                      [2*(q[2]*q[0] - q[1]*q[3]),                                        2*(q[3]*q[0] + q[1]*q[2]), q[1]**2 - q[2]**2 - q[3]**2 + q[0]**2,v[2]],
                      [0,                                                                0,                      0,                                       1.0]])

        return T

    def do_transform_cloud(self, cloud, transform_matrix, original_scan, simulated=False):
        '''
        @param cloud: Cloud to be transformed expressed in its original frame.
        @type cloud: sensor_msgs.msg.PointCloud2
        @param transform_matrix: Transformation matrix from the original frame to its destination frame.
        @type transform: NumPy array
        @param original_scan: The LaserScan message from where the PointCloud2 message is generated.
                              Used only for header, which again is overwritten after the clouds are concatenated.
        @type original_scan: sensor_msgs.msg.LaserScan
        '''
        T = transform_matrix

        self.refl_x = np.array([[-1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

        points_out = []

        # Transform each point individually and reflects them about the x-axis of the goal frame.
        # Had to multiply with a reflection matrix about the x-axis.
        for p_in in LaserToPointcloud().read_points(cloud):
            
            # If generated in Gazebo, the points will have a z-coordinate. This sets that to 0.
         
            p_out = np.array((T @ [p_in[0], p_in[1], p_in[2], 1.0])) @ self.refl_x

            #p_out = [p_out[0], p_out[1], p_out[2], p_in[3]]
            if (simulated == False):
                p_out = [p_out[0], p_out[1], p_out[2], p_in[3]]
            else:
                p_out = [p_out[0], p_out[1], p_out[2], p_out[3], p_in[4]]
            points_out.append(p_out)

        res = LaserToPointcloud().create_cloud(original_scan.header, cloud.fields, points_out)
        return res
