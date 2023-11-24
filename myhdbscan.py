import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

def label_pcd_with_hdbscan(input_pcd_path, output_pcd_path):
    """
    Loads a point cloud, applies HDBSCAN clustering, and saves the point cloud with cluster labels.

    Parameters:
    input_pcd_path (str): Path to the input point cloud file (.pcd).
    output_pcd_path (str): Path to save the labeled output point cloud file (.pcd).
    """
    # Load the point cloud
    import hdbscan

    pcd = o3d.io.read_point_cloud(input_pcd_path)

    # Convert Open3D PCD to NumPy array
    points = np.asarray(pcd.points)

    # Apply HDBSCAN clustering
    clusterer = hdbscan.HDBSCAN(min_cluster_size=20)
    labels = clusterer.fit_predict(points)

    # Create a color map for cluster labels
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
    o3d.io.write_point_cloud(f"{output_pcd_path}_with_noise.pcd", pcd)

    # Save the point cloud with only clustered points
    o3d.io.write_point_cloud(f"{output_pcd_path}_clustered.pcd", clustered_pcd)

    # Visualize both point clouds
    o3d.visualization.draw_geometries([pcd])  # Visualize all points with cluster coloring
    o3d.visualization.draw_geometries([clustered_pcd])  # Visualize only clustered points

# Example usage
input_pcd_path = '/home/ubuntu/catkin_ws/src/LeGO-LOAM/pcd/gachonstation/inlier.pcd'
output_pcd_path = '/home/ubuntu/catkin_ws/src/LeGO-LOAM/pcd/gachonstation/hdbscan.pcd'
label_pcd_with_hdbscan(input_pcd_path, output_pcd_path)