import open3d as o3d
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R
global initialframe


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size= 0.1, origin=[0, 0,0])
    o3d.visualization.draw_geometries([source_temp, target_temp,mesh_frame])

  
def refine_registration(source, target, voxel_size):
    global initialframe

    current_transformation_here =initialframe  
    # print ('currenttransformation',current_transformation)
    distance_threshold = voxel_size * 0.01
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)

    voxel_radius = [0.01, 0.005, 0.002]
    max_iter = [50, 30, 14, 15]

    print("3. Colored point cloud registration")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        # cc=o3d.registration.TransformationEstimationForColoredICP(  )
        # print('fffffffffff',cc.lambda_geometric)
        print("3-3. Applying colored point cloud registration")
        #   result_icp = o3d.registration.registration_icp(
        #      source_down, target_down, radius, current_transformation,
        #     o3d.registration.TransformationEstimationForColoredICP( ),#
        #    o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
        #                                                     relative_rmse=1e-6,
        #                                                    max_iteration=iter))

        result_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down, 0.005, current_transformation_here,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20000))

    return result_icp

 
 
source = o3d.io.read_point_cloud("allpcd.ply",format="xyzrgb")
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size= 0.1, origin=[0, 0,0])
source.paint_uniform_color([1, 0.706, 0])

target=o3d.io.read_point_cloud('./test.ply',format="xyzrgb")
voxel_size = 0.015

#先用roslaunch calibrationpcdandmodel.launch 把读取到的点云拉到一个合适的初始位置，读取rosrun tf tf_echo /world /camera_link
translation =   [0.734, -0.061, 0.835]
RPY =  [-2.974, -0.146, 1.409]
transinitial = np.identity(4)
draw_registration_result(source, target,np.linalg.inv(transinitial) )

transinitial[0:3,0:3] =  R.from_euler('xyz',RPY).as_dcm()
transinitial[0, 3]    =   translation[0]
transinitial[1, 3]    = translation[1]
transinitial[2, 3]    = translation[2]


draw_registration_result(source, target,np.linalg.inv(transinitial) )

initialframe=np.linalg.inv(transinitial)
# initialframe=transinitial

result_icp = refine_registration(source, target, voxel_size)

draw_registration_result(source, target,  result_icp.transformation)

targetframe=np.linalg.inv( result_icp.transformation)  
print(targetframe)
r=R.from_matrix(targetframe[0:3,0:3])
qua=r.as_quat()
print("calibration results: rosrun tf static_transform_publisher "+str(targetframe[0,3])+' '+str(targetframe[1,3])+' '+str(targetframe[2,3])+' '+ str(qua[0])+' '+ str(qua[1])+' '+ str(qua[2])+' '+ str(qua[3])+" /world /camera_link 50")




