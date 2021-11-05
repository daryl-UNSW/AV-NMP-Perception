import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os

PATH = './data/'

def load_points(path, vox_size=0.2):
    pcd = o3d.io.read_point_cloud(path)

    # Downsampling
    pcd = pcd.voxel_down_sample(voxel_size=vox_size)

    # Segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.2,
                                         ransac_n=3,
                                         num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    plane = pcd.select_by_index(inliers)
    plane.paint_uniform_color([1.0, 0, 0])
    pcd = pcd.select_by_index(inliers, invert=True)

    # Clustering
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.5, min_points=6, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    
    o3d.visualization.draw_geometries([plane, pcd])

# file_num = len(os.listdir(PATH))
# for i in range(0, file_num):
#     # Read file
#     f = PATH + str(i).zfill(10) + '.pcd'

# Visualise the PCD file
pcd = load_points(PATH + '0000000000.pcd')
