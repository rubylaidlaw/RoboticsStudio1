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

class GuiRosNode(Node):
    def __init__(self, update_callback):
        super().__init__('gui_ros_node')
        self.estop_publisher = self.create_publisher(Bool, '/estop_status', 10)
        self.subscription = self.create_subscription(
            Float32,
            '/distance_to_goal',
            self.distance_callback,
            10)
        self.update_callback = update_callback

    def publish_estop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.estop_publisher.publish(msg)
        self.get_logger().info(f'Published e-stop: {active}')

    def distance_callback(self, msg: Float32):
        self.update_callback(msg.data)

class RobotLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Foxtrack Rover Control")
        self.geometry("500x600")
        self.estop_active = False
        self.simulation_process = None
        self.nav_process = None

        self.ros_node = None
        self.init_ros()
        self.create_widgets()
        self.project_root = os.path.dirname(os.path.abspath(__file__))

    def init_ros(self):
        try:
            rclpy.init()
            self.ros_node = GuiRosNode(self.update_distance_label)
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
        title_label = ttk.Label(self, text="Foxtrack Rover Control Panel", font=("Arial", 16, "bold"))
        title_label.pack(pady=20)

        launch_frame = ttk.LabelFrame(self, text="Simulation Control", padding=10)
        launch_frame.pack(pady=10, padx=20, fill="x")

        self.launch_button = ttk.Button(
            launch_frame,
            text="Launch Simulation",
            command=self.launch_simulation,
            width=30
        )
        self.launch_button.pack(pady=5)

        self.stop_button = ttk.Button(
            launch_frame,
            text="Stop Simulation",
            command=self.stop_simulation,
            state="disabled",
            width=30
        )
        self.stop_button.pack(pady=5)

        self.status_label = ttk.Label(
            launch_frame,
            text="Status: Not Running",
            foreground="red"
        )
        self.status_label.pack(pady=5)

        estop_frame = ttk.LabelFrame(self, text="Emergency Control", padding=10)
        estop_frame.pack(pady=10, padx=20, fill="x")

        self.estop_button = ttk.Button(
            estop_frame,
            text="Emergency Stop",
            command=self.toggle_estop,
            width=30
        )
        self.estop_button.pack(pady=5)

        self.estop_status_label = ttk.Label(
            estop_frame,
            text="E-Stop: Inactive",
            foreground="green"
        )
        self.estop_status_label.pack(pady=5)

        nav_frame = ttk.LabelFrame(self, text="Navigation Control", padding=10)
        nav_frame.pack(pady=10, padx=20, fill="x")

        self.nav_button = ttk.Button(
            nav_frame,
            text="Start Navigation",
            command=self.start_navigation,
            state="disabled",
            width=30
        )
        self.nav_button.pack(pady=5)

        dist_frame = ttk.LabelFrame(self, text="Distance to goal", padding=10)
        dist_frame.pack(pady=10, padx=20, fill="x")

        self.distance_label = ttk.Label(dist_frame,
            text="N/A", 
            font=("Arial", 12)
        )
        self.distance_label.pack(pady=10)

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

        terminal = 'xterm'
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        setup_bash = "/opt/ros/humble/setup.bash"
        ws_setup_bash = os.path.join(root, "install", "setup.bash")
        launch_file = os.path.join(root, "launch", "41068_ignition.launch.py")
        rviz_config = os.path.join(root, "config", "41068.rviz")
        models_path = os.path.join(root, "models")

        if not os.path.exists(setup_bash):
            self.status_label.config(
                text="Status: Error - ROS2 Humble not installed",
                foreground="red"
            )
            return
        if not os.path.exists(launch_file):
            self.status_label.config(
                text="Status: Error - Launch file not found",
                foreground="red"
            )
            return

        launch_cmd = (
            f"cd {root} && colcon build --symlink-install && "
            f"source {setup_bash} && "
            f"source {ws_setup_bash} && "
            "export LIBGL_ALWAYS_SOFTWARE=1 && "
            f"export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:{models_path} && "
            f"ros2 launch {launch_file} "
            "slam:=true nav2:=true rviz:=true "
            f"rviz_config:={rviz_config} world:=large_demo; exec bash"
        )
        cmd = [terminal, '-e', f'bash -c "{launch_cmd}"']

        self.simulation_process = subprocess.Popen(cmd)
        self.launch_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.nav_button.config(state="normal")
        self.status_label.config(text="Status: Running", foreground="green")


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

            reset_script_path = os.path.join(self.project_root, "reset.sh")
            subprocess.Popen(['bash', reset_script_path])
            print("Reset script executed.")

    def start_navigation(self):
        if self.nav_process is not None and self.nav_process.poll() is None:
            print("Navigation already running.")
            return
        send_goal_script = os.path.join(self.project_root, "send_goal.py")
        self.nav_process = subprocess.Popen(
            ['python3', send_goal_script]
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

    def on_closing(self):
        if self.simulation_process is not None:
            self.stop_simulation()
        if self.ros_node is not None:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        self.destroy()

def main():
    app = RobotLauncher()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()

if __name__ == "__main__":
    main()