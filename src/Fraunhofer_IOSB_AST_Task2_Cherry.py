import open3d as o3d
import numpy as np
import tkinter as tk
from tkinter import ttk

# Common parameters for segmentation
voxel_sz = 0.3
ransac_n = 50
num_iterations = 1000

# Global variables
global pcd, downpcd
# Change this directory to where the pcd is stored
path_to_pcd = "data/HiL_Innen_subsampled - Cloud.pcd"
#pcd = o3d.io.read_point_cloud("../../test_data/my_points.txt", format='xyz')
pcd = o3d.io.read_point_cloud(path_to_pcd)
downpcd = pcd.voxel_down_sample(voxel_size=voxel_sz)

# Colors per differente plane
colors = [
    [1.0, 0, 0],   # Red
    [0, 1.0, 0]    # Green
]

# Function to segmentate and color planes
def segment_and_color_plane(downpcd, distance_threshold, ransac_n, num_iterations, color):
    plane_model, inliers = downpcd.segment_plane(distance_threshold=distance_threshold,
                                                 ransac_n=ransac_n,
                                                 num_iterations=num_iterations)
    inlier_cloud = downpcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color(color)
    downpcd = downpcd.select_by_index(inliers, invert=True)
    return inlier_cloud, downpcd, plane_model, inliers

# Function to update and visualize the path available for the robot to move with the dynamic parameters
def update_and_visualize():
    global downpcd
    #downpcd = pcd.voxel_down_sample(voxel_size=voxel_sz)
    
    # Obtaining values from the interface
    distance_threshold = float(distance_threshold_var.get())
    a = float(a_var.get())
    b = float(b_var.get())
    c = float(c_var.get())

    # Segmenting and coloring first plane
    inlier_cloud, _, plane_model_1, _ = segment_and_color_plane(downpcd, distance_threshold, ransac_n, num_iterations, colors[0])
    
    # Getting equation of the first plane 
    [a_1, b_1, c_1, d_1] = plane_model_1
    normal_plane_1 = np.array([a_1, b_1, c_1])

    # Apply the plane equation to hightlight the points on the original point cloud
    points = np.asarray(pcd.points)
    distances = np.abs(a_1 * points[:, 0] + b_1 * points[:, 1] + c_1 * points[:, 2] + d_1) / np.sqrt(a_1**2 + b_1**2 + c_1**2)

    # Selecting nearby points to the plane within the distance threshold
    inliers = np.where(distances < distance_threshold)[0]
    inlier_cloud_original = pcd.select_by_index(inliers)  # "Ground"
    inlier_cloud_original.paint_uniform_color([0, 1, 0])  # Applying green color to the ground

    # Creating the pointcloud outside the plane
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # Estimate the normal function of the points (of PCD) outside the plane
    outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Calculate the angle between the normal of the plane and the estimates normals
    normals = np.asarray(outlier_cloud.normals)
    dot_products = np.dot(normals, normal_plane_1)
    angles = np.arccos(dot_products / (np.linalg.norm(normals, axis=1) * np.linalg.norm(normal_plane_1)))
    angles_degrees = np.degrees(angles)

    # Selecting points with a higher angle of n degrees
    angle_threshold = 70  # Threshold in degrees
    inliers_by_angle = np.where(angles_degrees > angle_threshold)[0]
    inlier_cloud_by_angle = outlier_cloud.select_by_index(inliers_by_angle)

    # Creating the pointclouds outside the plane which have more than "angle_threshold" degress from the normal
    outlier_cloud_2 = outlier_cloud.select_by_index(inliers_by_angle, invert=True)

    # Converting the point cloud to a numpy array
    points_array = np.asarray(outlier_cloud_2.points)

    # Obtaining the z coordinates of the points
    z_coords = points_array[:, 2]

    # Finding the point indices with a z higher than "c" value
    indices_z_gt_c = np.where(z_coords > c)[0]

    # Selecting the point in outlier_cloud_2 that do not match this indices
    outlier_cloud_2_filtered = outlier_cloud_2.select_by_index(indices_z_gt_c, invert=True)  # cut
    outlier_cloud_2_filtered.paint_uniform_color([1, 0, 0]) # Red color to the area not to trespass  

    # Converting the point cloud to a numpy array
    points_array_pcd = np.asarray(pcd.points)

    # Obtaining the z coordinates of the points
    z_coords_pcd = points_array_pcd[:, 2]

    # Finding the point indices with a z lower than "c" value
    indices_z_lt_c = np.where(z_coords_pcd < c)[0]

    # Selecting the point in "pcd" that do not match this last found indices
    pcd_filtered = pcd.select_by_index(indices_z_lt_c, invert=True)  # corte

    # Visualizing the final plot of area to move (green) and area to not trespass (red)
    # inlier_cloud_original -> Ground
    # outlier_cloud_2_filtered -> Collider
    # pcd_filtered -> corte
    o3d.visualization.draw_geometries([inlier_cloud_original,pcd_filtered],
                                      width=1280, height=720,
                                      zoom=0.8,
                                      front=[0.4999, 0.1659, 0.8499],
                                      lookat=[2.1813, 2.0619, 2.0999],
                                      up=[0, 0, 1])

###################################################### Interface ######################################################
root = tk.Tk()
root.title("Parameters Selection")


# Dynamic variable
distance_threshold_var = tk.StringVar(value="0.5")
# "Fixed" robot dimensions
a_var = tk.StringVar(value="0.3")
b_var = tk.StringVar(value="0.3")
c_var = tk.StringVar(value="0.3")

# Distance threshold value
ttk.Label(root, text="Distance Threshold:").grid(column=0, row=0, padx=10, pady=10)
ttk.Entry(root, textvariable=distance_threshold_var).grid(column=1, row=0, padx=10, pady=10)

# Robot dimension values
ttk.Label(root, text="Robot length:").grid(column=0, row=1, padx=10, pady=10)
ttk.Entry(root, textvariable=a_var).grid(column=1, row=1, padx=10, pady=10)
ttk.Label(root, text="Robot width:").grid(column=0, row=2, padx=10, pady=10)
ttk.Entry(root, textvariable=b_var).grid(column=1, row=2, padx=10, pady=10)
ttk.Label(root, text="Robot height:").grid(column=0, row=3, padx=10, pady=10)
ttk.Entry(root, textvariable=c_var).grid(column=1, row=3, padx=10, pady=10)
# Update and visualize button
ttk.Button(root, text="Update", command=update_and_visualize).grid(column=1, row=4, columnspan=2, padx=10, pady=10)
root.mainloop()
