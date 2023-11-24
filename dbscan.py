import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


def DBScan(input, output):
    # Load your point cloud
    # input = "./filtered/filtered_AI_7F_05.pcd"
    pcd = o3d.io.read_point_cloud(input)

    # Perform DBSCAN clustering
    labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=20, print_progress=True))

    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    # Preparing colors for visualization
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0  # Setting noise to black
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])



    # Correctly convert Open3D Vector3dVector to NumPy array before indexing
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # Now you can use numpy boolean indexing
    clustered_points = points[labels >= 0]
    clustered_colors = colors[labels >= 0]

    # Create a new point cloud for clustered points
    clustered_pcd = o3d.geometry.PointCloud()
    clustered_pcd.points = o3d.utility.Vector3dVector(clustered_points)
    clustered_pcd.colors = o3d.utility.Vector3dVector(clustered_colors)

    # output = './conveted_AI_7F_05'
    # Save the entire point cloud with cluster coloring
    o3d.io.write_point_cloud(f"{output}_with_noise.pcd", pcd)

    # Save the point cloud with only clustered points
    o3d.io.write_point_cloud(f"{output}_clustered.pcd", clustered_pcd)

    # Visualize both point clouds
    o3d.visualization.draw_geometries([pcd])  # Visualize all points with cluster coloring
    o3d.visualization.draw_geometries([clustered_pcd])  # Visualize only clustered points

DBScan('/home/ubuntu/catkin_ws/src/LeGO-LOAM/pcd/gachonhall/inlier.pcd', '/home/ubuntu/catkin_ws/src/LeGO-LOAM/pcd/gachonhall/dbscan.pcd')