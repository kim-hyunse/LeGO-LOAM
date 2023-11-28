import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import hdbscan


def get_rainbow_color(frequency, max_frequency):
    # Normalize frequency to a value between 0 and 1
    normalized_freq = frequency / max_frequency
    # Use a colormap that goes from red (high frequency) to violet (low frequency)
    color = plt.get_cmap('rainbow')(normalized_freq)
    return color[:3]  # Exclude the alpha channel

def visualize_shared_xy_pcd(input_file, output_file):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(input_file)
    points = np.asarray(pcd.points)
    voxel_size=0.1

    min_z = np.min(points[:, 2])
    max_z = np.max(points[:, 2])
    height = max_z - min_z

    # Compute voxel indices for each point
    voxel_indices = np.floor(points / voxel_size).astype(int)
 
    threshold = (max(voxel_indices[:, 2]) - min(voxel_indices[:, 2])) / 5 

    # Group points by their (x, y) voxel indices and count occurrences
    voxel_to_points = {}
    xy_frequency = {}
    z_already_counted = {}  # New dictionary to track Z-coordinates for each voxel
    
    for voxel_index, point in zip(voxel_indices, points):
        xy_key = tuple(voxel_index[:2])
        z_index = voxel_index[2]

        if xy_key not in voxel_to_points:
            voxel_to_points[xy_key] = []
            xy_frequency[xy_key] = 0
        

        # Check if this Z index has already been counted for this XY key
        if not any(z_index == existing_point_z_index for existing_point, existing_point_z_index in voxel_to_points[xy_key]):
            
            xy_frequency[xy_key] += 1

        # Add the point and its Z index to the list
        voxel_to_points[xy_key].append((point, z_index))
 
    # Separate points and colors for the new point cloud
    max_frequency = max(xy_frequency.values())
    colored_points = []

    for xy_key, pts in voxel_to_points.items():
        if xy_frequency[xy_key] >= threshold:
            color = get_rainbow_color(xy_frequency[xy_key], max_frequency)
            for pt, _ in pts:
                colored_points.append((*pt, *color))

    if not colored_points:
        print("No points meet the shoulder threshold.")
        return

    colored_points = np.array(colored_points)
    points_filtered = colored_points[:, :3]
    colors_filtered = colored_points[:, 3:6]

    # Create a point cloud for visualization
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points_filtered)
    filtered_pcd.colors = o3d.utility.Vector3dVector(colors_filtered)

    o3d.io.write_point_cloud(output_file, filtered_pcd)
    print(f"Filtered point cloud saved to {output_file}")

    # Visualize the point cloud
    o3d.visualization.draw_geometries([filtered_pcd])

def HDBSCAN(input_pcd_path, output_pcd_path, min_cluster_size=100):
    """
    Loads a point cloud, applies HDBSCAN clustering, and saves the point cloud with cluster labels.

    Parameters:
    input_pcd_path (str): Path to the input point cloud file (.pcd).
    output_pcd_path (str): Path to save the labeled output point cloud file (.pcd).
    min_cluster_size (int): Minimum size of clusters for HDBSCAN.
    """
    # Load your point cloud
    pcd = o3d.io.read_point_cloud(input_pcd_path)  # Use input_pcd_path here
    
    # Convert Open3D PCD to NumPy array
    points = np.asarray(pcd.points)

    # Apply HDBSCAN clustering
    clusterer = hdbscan.HDBSCAN(min_cluster_size=min_cluster_size)
    labels = clusterer.fit_predict(points)

    # Filter out noise and create a new point cloud for clustered points
    clustered_points = points[labels >= 0]
    clustered_pcd = o3d.geometry.PointCloud()
    clustered_pcd.points = o3d.utility.Vector3dVector(clustered_points)

    # Save the clustered point cloud
    o3d.io.write_point_cloud(output_pcd_path, clustered_pcd)
    print(f"Filtered point cloud saved to {output_pcd_path}")  

    # Visualize the clustered point cloud
    o3d.visualization.draw_geometries([clustered_pcd])



def sor_filter(input_pcd_path, output_pcd_path, nb_neighbors=100, std_ratio=0.001):
    """
    Apply Statistical Outlier Removal (SOR) to a point cloud and save the filtered point cloud.

    :param input_pcd_path: Path to the input point cloud file (.pcd).
    :param output_pcd_path: Path to save the filtered point cloud file (.pcd).
    :param nb_neighbors: Number of neighbors to consider in the statistical analysis.
    :param std_ratio: Standard deviation ratio. Points with a distance larger than 
                      this ratio times the standard deviation of the mean distance 
                      will be considered as outliers.
    """
    # Load the input point cloud
    pcd = o3d.io.read_point_cloud(input_pcd_path)

    # Perform Statistical Outlier Removal
    pcd_filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    # Save the filtered point cloud to the specified output file
    o3d.io.write_point_cloud(output_pcd_path, pcd_filtered)

    print(f"Filtered point cloud saved to {output_pcd_path}")

    # Visualize the original and filtered point clouds (optional)
    o3d.visualization.draw_geometries([pcd_filtered])



input_pcd_path = '/tmp/finalCloud_converted.pcd'
height_output = '/tmp/gachonstation_height.pcd'
hdbscan_output='/tmp/gachonstation_hdb.pcd'
final_ouptut = '/tmp/gachonstation_sor_final.pcd'


HDBSCAN(input_pcd_path=input_pcd_path, output_pcd_path=hdbscan_output)
visualize_shared_xy_pcd(input_file=hdbscan_output, output_file=height_output)
sor_filter(input_pcd_path=height_output, output_pcd_path=final_ouptut)
