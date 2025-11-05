#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import subprocess
import os
import shutil
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage, ImageTk


class GuiRosNode(Node):
    def __init__(self, update_callback, update_camera_callback=None):
        super().__init__('gui_ros_node')
        self.estop_publisher = self.create_publisher(Bool, '/estop_status', 10)
        self.subscription = self.create_subscription(
            Float32,
            '/distance_to_goal',
            self.distance_callback,
            10)
        self.update_callback = update_callback

        # Camera subscriber setup
        self.bridge = CvBridge()
        self.camera_callback = update_camera_callback
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)

    def publish_estop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.estop_publisher.publish(msg)
        self.get_logger().info(f'Published e-stop: {active}')

    def distance_callback(self, msg: Float32):
        self.update_callback(msg.data)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.camera_callback:
                self.camera_callback(cv_image)
        except Exception as e:
            self.get_logger().error(f"Camera conversion failed: {e}")

class RobotLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Foxtrack Rover Control")
<<<<<<< HEAD
        self.geometry("1100x700")
=======
        self.geometry("500x675")
>>>>>>> 3c002498d8a8df05c3ab11520e6423c3ac998d9e
        self.estop_active = False
        self.simulation_process = None
        self.nav_process = None

        self.ros_node = None
        self.init_ros()
        self.create_widgets()
        self.project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def init_ros(self):
        try:
            rclpy.init()
            self.ros_node = GuiRosNode(self.update_distance_label, self.update_camera_view)
            self.ros_thread = threading.Thread(
                target=lambda: rclpy.spin(self.ros_node),
                daemon=True
            )
            self.ros_thread.start()
            print("Unified ROS2 GUI node initialized")
        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")
            self.ros_node = None

    def update_distance_label(self, distance):
        self.after(0, lambda: self.distance_label.config(text=f"Distance to goal: {distance:.2f} m"))

    def create_widgets(self):
        # --- Main layout split ---
        main_frame = ttk.Frame(self)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Left and right sections
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(0,10))
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky="nsew")

        # Make grid expand properly
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=2)
        main_frame.rowconfigure(0, weight=1)

        # ========================
        # LEFT SIDE (CONTROL PANEL)
        # ========================
        title_label = ttk.Label(left_frame, text="Foxtrack Rover Control Panel", font=("Arial", 16, "bold"))
        title_label.pack(pady=20)

<<<<<<< HEAD
        # Launch controls
        launch_frame = ttk.LabelFrame(left_frame, text="Launch Control", padding=10)
        launch_frame.pack(pady=10, fill="x")
=======
        launch_frame = ttk.LabelFrame(self, text="Simulation Control", padding=10)
        launch_frame.pack(pady=10, padx=20, fill="x")
>>>>>>> 3c002498d8a8df05c3ab11520e6423c3ac998d9e

        self.launch_button = ttk.Button(
<<<<<<< HEAD
            launch_frame, text="Launch Simulation",
            command=self.launch_simulation_with_optional_coords, width=30
=======
            launch_frame,
            text="Launch Simulation",
            command=self.launch_simulation,
            width=30
>>>>>>> 3c002498d8a8df05c3ab11520e6423c3ac998d9e
        )
        self.launch_button.pack(pady=5)

        self.stop_button = ttk.Button(
            launch_frame, text="Stop Simulation",
            command=self.stop_simulation, state="disabled", width=30
        )
        self.stop_button.pack(pady=5)

        self.status_label = ttk.Label(launch_frame, text="Status: Not Running", foreground="red")
        self.status_label.pack(pady=5)

        # E-stop
        estop_frame = ttk.LabelFrame(left_frame, text="Emergency Control", padding=10)
        estop_frame.pack(pady=10, fill="x")
        self.estop_button = ttk.Button(estop_frame, text="Emergency Stop", command=self.toggle_estop, width=30)
        self.estop_button.pack(pady=5)
        self.estop_status_label = ttk.Label(estop_frame, text="E-Stop: Inactive", foreground="green")
        self.estop_status_label.pack(pady=5)

        # Navigation
        nav_frame = ttk.LabelFrame(left_frame, text="Navigation Control", padding=10)
        nav_frame.pack(pady=10, fill="x")
        self.nav_button = ttk.Button(nav_frame, text="Start Navigation", command=self.start_navigation,
                                    state="disabled", width=30)
        self.nav_button.pack(pady=5)

<<<<<<< HEAD
        # Terminal
        terminal_frame = ttk.LabelFrame(left_frame, text="New Terminal", padding=10)
        terminal_frame.pack(pady=10, fill="x")
        self.open_terminal_button = ttk.Button(terminal_frame, text="Open New Terminal",
                                            command=self.open_new_terminal, width=30)
        self.open_terminal_button.pack(pady=5)

        # Distance
        dist_frame = ttk.LabelFrame(left_frame, text="Distance to goal", padding=10)
        dist_frame.pack(pady=10, fill="x")
        self.distance_label = ttk.Label(dist_frame, text="N/A", font=("Arial", 12))
        self.distance_label.pack(pady=10)

        # ========================
        # RIGHT SIDE (CAMERA FEEDS)
        # ========================
        cam_frame = ttk.LabelFrame(right_frame, text="Camera Feed", padding=10)
        cam_frame.pack(fill="both", expand=True)

        image_frame = ttk.Frame(cam_frame)
        image_frame.pack(fill="both", expand=True)

        # Camera (top)
        self.camera_label = ttk.Label(image_frame)
        self.camera_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")

        # Red mask (bottom)
        self.mask_label = ttk.Label(image_frame)
        self.mask_label.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        # Configure vertical stacking
        image_frame.rowconfigure(0, weight=1)
        image_frame.rowconfigure(1, weight=1)
        image_frame.columnconfigure(0, weight=1)

=======
        terminal_frame = ttk.LabelFrame(self, text="New Terminal", padding=10)
        terminal_frame.pack(pady=10, padx=20, fill="x")

        self.open_terminal_button = ttk.Button(
            terminal_frame, 
            text="Open New Terminal", 
            command=self.open_new_terminal,
            width=30
        )
        self.open_terminal_button.pack(pady=5)

        dist_frame = ttk.LabelFrame(self, text="Distance to goal", padding=10)
        dist_frame.pack(pady=10, padx=20, fill="x")

        self.distance_label = ttk.Label(dist_frame,
            text="N/A", 
            font=("Arial", 12)
        )
        self.distance_label.pack(pady=10)

>>>>>>> 3c002498d8a8df05c3ab11520e6423c3ac998d9e
    def get_terminal_emulator(self):
        terminals = ['xterm', 'gnome-terminal', 'konsole', 'xfce4-terminal', 'terminator']
        for term in terminals:
            if shutil.which(term):
                return term
        return None

    def launch_simulation(self):
        if self.simulation_process is not None:
            print("Simulation already running!")
            return

        terminal = self.get_terminal_emulator()
        if terminal is None:
            self.status_label.config(
                text="Status: Error - No terminal found",
                foreground="red"
            )
            print("Error: No terminal emulator found!")
            return

        setup_bash = "/opt/ros/humble/setup.bash"
        ws_setup_bash = os.path.join(self.project_root, "install", "setup.bash")
        launch_file = os.path.join(self.project_root, "launch", "41068_ignition.launch.py")
        rviz_config = os.path.join(self.project_root, "config", "41068.rviz")

        launch_cmd = (
            f"source {setup_bash} && "
            f"source {ws_setup_bash} && "
            "export LIBGL_ALWAYS_SOFTWARE=1 && "
            f"ros2 launch {launch_file} "
            "slam:=true nav2:=true rviz:=true "
            f"rviz_config:={rviz_config} world:=large_demo; exec bash"
        )
        if terminal == 'gnome-terminal':
            cmd = [terminal, '--', 'bash', '-c', launch_cmd]
        elif terminal == 'konsole':
            cmd = [terminal, '-e', 'bash', '-c', launch_cmd]
        elif terminal in ['xfce4-terminal', 'xterm', 'terminator']:
            cmd = [terminal, '-e', f'bash -c "{launch_cmd}"']
        else:
            cmd = [terminal, '-e', 'bash', '-c', launch_cmd]

        self.simulation_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        self.launch_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.nav_button.config(state="normal")
        self.status_label.config(text="Status: Running", foreground="green")
        print(f"Simulation launched successfully using {terminal}!")

    def stop_simulation(self):
        if self.simulation_process is not None:
            try:
                os.killpg(os.getpgid(self.simulation_process.pid), signal.SIGTERM)
            except:
                try:
                    self.simulation_process.terminate()
                    self.simulation_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.simulation_process.kill()
            self.simulation_process = None
            self.launch_button.config(state="normal")
            self.stop_button.config(state="disabled")
            self.nav_button.config(state="disabled")
            self.status_label.config(text="Status: Not Running", foreground="red")
            print("Simulation stopped")

            reset_script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reset.sh")
            subprocess.Popen(['bash', reset_script_path])
            print("Reset script executed.")

    def start_navigation(self):
        if self.nav_process is not None and self.nav_process.poll() is None:
            print("Navigation already running.")
            return
        send_goal_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "send_goal.py")
        self.nav_process = subprocess.Popen(
            ['python3', send_goal_script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("Navigation started")

    def toggle_estop(self):
        self.estop_active = not self.estop_active
        if self.estop_active:
            self.estop_button.config(text="Resume")
            self.estop_status_label.config(text="E-Stop: ACTIVE", foreground="red")
            print("Emergency Stop ACTIVATED!")
        else:
            self.estop_button.config(text="Emergency Stop")
            self.estop_status_label.config(text="E-Stop: Inactive", foreground="green")
            print("Emergency Stop released")
        if self.ros_node is not None:
            self.ros_node.publish_estop(self.estop_active)

    def open_new_terminal(self):
        terminal = self.get_terminal_emulator()
        if terminal is not None:
            subprocess.Popen([terminal])
        else:
            print("No terminal emulator found!")

    def on_closing(self):
        if self.simulation_process is not None:
            self.stop_simulation()
        if self.ros_node is not None:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        if hasattr(self.ros_node, 'camera_callback'):
            self.ros_node.camera_callback = None
        self.destroy()

    def update_camera_view(self, frame):
        """Display the camera feed and red mask in the GUI"""
        # Scale down both images (tweak size as needed)
        frame = cv2.resize(frame, (400, 250))  # width, height in pixels

        # Create red mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = (0, 70, 50)
        upper_red = (10, 255, 255)
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Convert to RGB for Tkinter
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        imgtk = ImageTk.PhotoImage(image=PILImage.fromarray(rgb))
        mask_imgtk = ImageTk.PhotoImage(image=PILImage.fromarray(mask))

        # Update labels
        self.camera_label.imgtk = imgtk
        self.camera_label.configure(image=imgtk)
        self.mask_label.imgtk = mask_imgtk
        self.mask_label.configure(image=mask_imgtk)


def main():
    app = RobotLauncher()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()

if __name__ == "__main__":
    main()