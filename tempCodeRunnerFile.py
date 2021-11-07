# Segmentation
    # plane_model, inliers = pcd.segment_plane(distance_threshold=0.2,
    #                                      ransac_n=30,
    #                                      num_iterations=1000)
    # [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # plane = pcd.select_by_index(inliers)
    # plane.paint_uniform_color([1.0, 0, 0])
    # pcd = pcd.select_by_index(inliers, invert=True)