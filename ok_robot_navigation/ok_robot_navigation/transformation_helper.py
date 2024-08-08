# 1. COMPUTE X, Y VECTOR: (P2 - P1) / || P2 - P1 ||

# 2. FIND Z VECTOR (X x Y)

# 3. RECALCULATE Y USING NEW Z

# 4. FIND TRANSFORMATION MATRIX (X, Y, Z, P1)

# 5. APPLY TRANSFOMATION TO POINT CLOUD (LIKELY IN PATH_PLANNING.PY)

import numpy as np
from skspatial.objects import Vector
from skspatial.plotting import plot_2d
from skspatial.objects import Point
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D


def get_transform_matrix(p1, p2, p3):
    '''Get unit vectors defining global reference frame of robot, assuming robot is positioned on p1 and oriented
       towards p2. Helper function for getting the transformation matrix from r3d point cloud reference frame to 
       the global re'''
    vector_1to2 = Vector.from_points(p1, p2)
    vector_1to3 = Vector.from_points(p1, p3)
    norm_x = vector_1to2 / (vector_1to2.norm())
    norm_y_tmp = vector_1to3 / (vector_1to3.norm()) 
    # temporary unit vector for y direction 
    # defined by p1 and an arbitrary point p3 on the floor (marker is pasted on floor)
    # temporary norm_y is used to obtain norm_z, the vector normal to the floor surface
    z_vector = norm_x.cross(norm_y_tmp)
    norm_z = z_vector / (z_vector.norm())
    norm_y = norm_z.cross(norm_x)
    norm_y = norm_y / (norm_y.norm())
    
    return np.array([[norm_x[0], norm_y[0], norm_z[0], p1[0]],
                     [norm_x[1], norm_y[1], norm_z[1], p1[1]],
                     [norm_x[2], norm_y[2], norm_z[2], p1[2]],
                     [0, 0, 0, 1]])


if __name__ == '__main__':
    # # Point cloud:
    # p1 = Point([0.476446, 0.780500, -1.285691])
    # p2 = Point([0.717750, 0.642038, -1.285987])
    # p3 = Point([0.628205, 1.021866]) # random third marked point on floor

        

    # From r3d file: 2024-08-06--experimentroomtake3.r3d
    # Coordinates of markers for p1, p2, p3 obtained using CloudCompare 
    p1 = Point([0.066222, 0.450469, -1.183902])
    p2 = Point([0.366797, 0.509809, -1.183809])
    p3_a = Point([0.156285, 0.749222, -1.186817])
    p3_b = Point([0.004243, 0.901338, -1.187805])
    p3_c = Point([-0.286204, 0.802674, -1.188120])

    p3_list = [p3_a, p3_b, p3_c]
    # vector_1to2 = Vector.from_points(p1, p2)
    # norm_z_list = []
    # for p3 in p3_list:
    #     vector_1to3 = Vector.from_points(p1, p3)

    #     """vector_projected = vector_1to2.project_vector(vector_1to3)
    #     vector_1to_new_3 = vector_1to3 - vector_projected"""

    #     """assert vector_1to_new_3.is_perpendicular(vector_1to2)
    #     print(vector_1to_new_3)
    #     print(p1)"""


    #     norm_x = vector_1to2 / (vector_1to2.norm())
    #     norm_y_tmp = vector_1to3 / (vector_1to3.norm()) 
    #     # temporary unit vector for y direction 
    #     # defined by p1 and an arbitrary point p3 on the floor (marker is pasted on floor)
    #     # temporary norm_y is used to obtain norm_z, the vector normal to the floor surface

    #     z_vector = norm_x.cross(norm_y_tmp)
    #     norm_z = z_vector / (z_vector.norm())
    #     norm_y = norm_z.cross(norm_x)
    #     norm_y = norm_y / (norm_y.norm())
    #     norm_z_list.append(norm_z)


    # fig = plt.figure()

    # ax = fig.add_subplot(111, projection='3d')

    # norm_x.plot_3d(ax, color='r')
    # norm_y.plot_3d(ax, color='g')
    # for vec in norm_z_list:
    #     print(vec)
    #     vec.plot_3d(ax, color='b')

    # plt.show()


        