import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os

PATH = './data/'


def load_points(path, vox_size=0.2):
    pcds = []
    file_num = len(os.listdir(path))
    for i in range(0, file_num):
        # Read file
        f = PATH + str(i).zfill(10) + '.pcd'
        pcd = o3d.io.read_point_cloud(f)

        # Downsampling
        pcd = pcd.voxel_down_sample(voxel_size=vox_size)

        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                pcd.cluster_dbscan(eps=0.5, min_points=6, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        pcds.append(pcd)

    
    return pcds

pcds = load_points(PATH)


# Visualise the PCD file
o3d.visualization.draw_geometries(pcds)
