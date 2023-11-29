import tkinter as tk
from tkinter import filedialog, Label, Button
import subprocess
import os
import open3d as o3d
import cv2
import numpy as np
from PIL import Image, ImageTk

def select_file():
    filepath = filedialog.askopenfilename(filetypes=[("PCD Files", "*.pcd")])
    if filepath:
        entry.delete(0, tk.END)
        entry.insert(0, filepath)

def run_conv():
    filepath = entry.get()
    print(os.getcwd())
    if filepath:
        subprocess.run(["python", "conv.py", filepath])

def run_p4():
    subprocess.run(["python", "p4.py"])
    generate_top_view_images()


def generate_top_view_images():
    pcd_files = ['/tmp/converted_hdb.pcd',
                 '/tmp/converted_height.pcd',
                 '/tmp/converted_sor_final.pcd']

    # Create a frame to hold the images horizontally
    image_frame = tk.Frame(root)
    image_frame.pack()

    for pcd_file in pcd_files:
        pcd = o3d.io.read_point_cloud(pcd_file)
        points = np.asarray(pcd.points)

        # Define the size of the image
        image_size = 200  # Adjust this size as needed
        img = np.ones((image_size, image_size, 3), dtype=np.uint8) * 255  # White background

        # Find the center of the point cloud
        center = np.mean(points, axis=0)

        # Find the range of the point cloud
        range_x = np.max(points[:, 0]) - np.min(points[:, 0])
        range_y = np.max(points[:, 1]) - np.min(points[:, 1])
        scale = min(image_size / range_x, image_size / range_y)

        # Scale and center the points
        scaled_points = (points - center) * scale + image_size / 2

        # Draw the points on the image
        for point in scaled_points:
            x, y = int(point[0]), int(point[1])
            if 0 <= x < image_size and 0 <= y < image_size:
                cv2.circle(img, (x, y), 1, (0, 0, 0), -1)  # Draw points in black

        # Display the image horizontally with a smaller size
        display_image(img, pcd_file, image_frame)

def display_image(img, pcd_file, parent_frame):
    # Convert the OpenCV image to a PIL image
    image = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    image.thumbnail((1000, 1000))  # Resize the image for display (adjust as needed)
    photo = ImageTk.PhotoImage(image)

    # Create a frame for each image and its save button
    frame = tk.Frame(parent_frame)
    frame.pack(side=tk.LEFT)

    # Create a label for the image
    img_label = Label(frame, image=photo)
    img_label.image = photo  # Keep a reference to avoid garbage collection
    img_label.pack()

    # Create a save button for the image
    save_button = Button(frame, text="Save Image", command=lambda: save_selected_image(img, pcd_file))
    save_button.pack()

def save_selected_image(img, pcd_file):
    save_path = filedialog.asksaveasfilename(defaultextension=".jpg",
                                             filetypes=[("JPEG files", "*.jpg")])
    if save_path:
        # Convert the OpenCV image to a PIL image and save it
        Image.fromarray(img).save(save_path)
        selected_image_label.config(text=f"Selected Image Saved: {save_path}")


# Create the main window
root = tk.Tk()
root.title("PCD Processing GUI")

# Create and place widgets
entry = tk.Entry(root, width=50)
entry.pack()

select_button = tk.Button(root, text="Select PCD File", command=select_file)
select_button.pack()

conv_button = tk.Button(root, text="convert file", command=run_conv)
conv_button.pack()

p4_button = tk.Button(root, text="Filtering and Generate Images", command=run_p4)
p4_button.pack()

selected_image_label = Label(root, text="No Image Selected")
selected_image_label.pack()

# Start the GUI event loop
root.mainloop()