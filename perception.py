import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import timeit

PATH = './data/'

def load_points(path, vox_size=0.2):
    pcd = o3d.io.read_point_cloud(path)
    start = timeit.default_timer()

    # Downsampling
    pcd = pcd.voxel_down_sample(voxel_size=vox_size)
    now = timeit.default_timer()
    print('Downsampling time: ', now - start)


    # bounding_polygon = np.array([
    # #Vertics Polygon 1
    #         [488.8989868164062, 612.208984375, 286.5320129394531],
    #         [485.114990234375, 612.208984375, 286.5320129394531],
    #         [485.114990234375, 605.0880126953125, 286.5320129394531],
    #         [488.8989868164062, 605.0880126953125, 286.5320129394531],
    # #Vertics Polygon 2
    #         [488.89898681640625, 612.208984375, 291.6619873046875], 
    #         [485.114990234375, 612.208984375, 291.6619873046875], 
    #         [485.114990234375, 605.0880126953125, 291.6619873046875],
    #         [488.89898681640625, 605.0880126953125, 291.6619873046875]]).astype("float64") 


    # vol = o3d.visualization.SelectionPolygonVolume()
    # vol.orthogonal_axis = "Y"
    # vol.axis_max = 500
    # vol.axis_min = 700
    # vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)

    # pcd = vol.crop_point_cloud(pcd)


    # Segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.4,
                                         ransac_n=3,
                                         num_iterations=1500)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    plane = pcd.select_by_index(inliers)
    plane.paint_uniform_color([1.0, 0, 0])
    pcd = pcd.select_by_index(inliers, invert=True)

    print('Segmentation time: ', timeit.default_timer() - now)
    now = timeit.default_timer()

    # Outlier Removal
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.1)
    pcd = cl.select_by_index(ind)

    # Clustering
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=1.5, min_points=3))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    print('Clustering time: ', timeit.default_timer() - now)
    print('Total process: ', timeit.default_timer() - start)

    o3d.visualization.draw_geometries([pcd])


# cycle through files
file_num = len(os.listdir(PATH))
for i in range(0, file_num):
    # Read file
    f = PATH + str(i).zfill(10) + '.pcd'
    load_points(f)

# # Load individual file
# load_points(PATH + '0000000018.pcd')
# load_points(PATH + '0000000001.pcd')