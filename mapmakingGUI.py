import os
import subprocess
import tkinter as tk
from tkinter import filedialog
import rospy
from std_msgs.msg import Empty
import signal  # Import the signal module

class ROSBagToMapGUI:
    def __init__(self, root):
        self.root = root
        root.title("ROS Bag to Map")

        # Start roscore
        roscore_cmd = "roscore"
        subprocess.Popen(roscore_cmd, shell=True)

        # Initialize ROS node
        rospy.init_node('rosbag_to_map_gui', anonymous=True)

        # Create a ROS publisher to send the stop signal
        self.stop_lego_loam_pub = rospy.Publisher('stop_lego_loam', Empty, queue_size=1)

        # Create a label to instruct the user
        label = tk.Label(root, text="Select a ROS bag file and click 'Make a Map'")
        label.pack(padx=10, pady=10)

        # Create a button to open a file dialog for selecting a ROS bag
        self.select_bag_button = tk.Button(root, text="Select ROS Bag", command=self.select_rosbag)
        self.select_bag_button.pack(padx=10, pady=5)

        # Create a button to make a map
        self.make_map_button = tk.Button(root, text="Make a Map", command=self.launch_lego_loam)
        self.make_map_button.pack(padx=10, pady=10)
        self.make_map_button.config(state=tk.DISABLED)  # Disable the button initially

        # Create a button to stop the lego_loam process
        self.stop_lego_loam_button = tk.Button(root, text="Stop Lego Loam", command=self.stop_lego_loam)
        self.stop_lego_loam_button.pack(padx=10, pady=10)
        self.stop_lego_loam_button.config(state=tk.DISABLED)  # Disable the button initially

        self.selected_rosbag = None
        self.rosbag_process = None
        self.lego_loam_process = None

    def select_rosbag(self):
        self.selected_rosbag = filedialog.askopenfilename(filetypes=[("ROS Bags", "*.bag")])
        if self.selected_rosbag:
            self.make_map_button.config(state=tk.NORMAL)  # Enable the "Make a Map" button

    def launch_lego_loam(self):
        if self.selected_rosbag:
            # Replace 'your_package' and 'your_launch_file' with the actual package and launch file names for lego_loam
            lego_loam_cmd = f"roslaunch lego_loam run.launch"
            rosbag_play_cmd = f"rosbag play {self.selected_rosbag} --clock --topic /ouster/points -r 3"

            # Start lego_loam process using roslaunch
            self.lego_loam_process = subprocess.Popen(lego_loam_cmd, shell=True)

            # Enable the "Stop Lego Loam" button while lego_loam is running
            self.stop_lego_loam_button.config(state=tk.NORMAL)

            # Disable the "Make a Map" button while lego_loam is running
            self.make_map_button.config(state=tk.DISABLED)

    def stop_lego_loam(self):
        if self.lego_loam_process is not None:
            try:
                # Send a SIGINT signal (Ctrl+C equivalent) to gracefully terminate lego_loam
                os.kill(self.lego_loam_process.pid, signal.SIGINT)
            except Exception as e:
                print(f"Error while stopping lego_loam: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ROSBagToMapGUI(root)
    root.mainloop()