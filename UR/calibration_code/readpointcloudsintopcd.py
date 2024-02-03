import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
tranlation=[[-1.9262 ,-0.003806, 0.472351],[-0.031185 , -1.39187,  0.483992],[1.59213  ,0.025712  ,0.461508],[-0.042772  , 1.27131 ,0.461243]]
rotations=[ [-6e-06-np.pi/2, -7e-06, -4.7e-05-np.pi/2],[-7e-06-np.pi/2, 6e-06, 1.57075-np.pi/2],[6e-06-np.pi/2, 7e-06, 3.14155-np.pi/2],[7e-06-np.pi/2 ,-6e-06, -1.57084-np.pi/2]]

def load_point_clouds( tranlation, rotations):
    pcds = []
    pcd_combined = o3d.geometry.PointCloud()
    for i in range(4):
        pcd = o3d.io.read_point_cloud("./%d.pcd" %(i+1))
        o3d.visualization.draw_geometries([pcd])
        transmax = np.identity(4)
        transmax[0:3,0:3] =  R.from_euler('xyz', [rotations[i][0],rotations[i][1],rotations[i][2]]).as_dcm()
        transmax[0, 3]    =   tranlation[i][0]
        transmax[1, 3]    = tranlation[i][1]
        transmax[2, 3]    = tranlation[i][2]
        pcd.transform(transmax )
        pcd_combined+=pcd
        #pcds.append(pcd)
    return pcd_combined


pcds_down = load_point_clouds(tranlation, rotations)
pcds_down.paint_uniform_color([0.5, 0.5, 0.5])

o3d.io.write_point_cloud('./allpcd.ply',pcds_down)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size= 0.1, origin=[0, 0,0])
o3d.visualization.draw_geometries([pcds_down,mesh_frame] )


