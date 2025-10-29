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
        self.geometry("500x800")
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

        launch_frame = ttk.LabelFrame(self, text="Launch Control", padding=10)
        launch_frame.pack(pady=10, padx=20, fill="x")

        ttk.Label(launch_frame, text="X Coordinate (-10 to 10):").pack(anchor='w')
        self.spawn_x_entry = ttk.Entry(launch_frame)
        self.spawn_x_entry.pack(fill='x', pady=5)

        ttk.Label(launch_frame, text="Y Coordinate (-10 to 10):").pack(anchor='w')
        self.spawn_y_entry = ttk.Entry(launch_frame)
        self.spawn_y_entry.pack(fill='x', pady=5)

        self.launch_button = ttk.Button(
            launch_frame,
            text="Launch Simulation",
            command=self.launch_simulation_with_optional_coords,
            width=30
        )
        self.launch_button.pack(pady=10)

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


    def get_terminal_emulator(self):
        terminals = ['xterm', 'gnome-terminal', 'konsole', 'xfce4-terminal', 'terminator']
        for term in terminals:
            if shutil.which(term):
                return term
        return None

    def launch_simulation_with_optional_coords(self):
        if self.simulation_process is not None:
            print("Simulation already running!")
            return

        # Default values
        x, y = 0.0, 0.0

        # Attempt to read coordinates if provided
        x_str = self.spawn_x_entry.get().strip()
        y_str = self.spawn_y_entry.get().strip()
        if x_str != "" and y_str != "":
            x = float(x_str)
            y = float(y_str)
            if not (-10 <= x <= 10) or not (-10 <= y <= 10):
                print("Coordinates must be within -10 to 10 range.")
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

        # Build launch command with or without spawn arguments
        spawn_args = ""
        if x is not None and y is not None:
            spawn_args = f"spawn_x:={x} spawn_y:={y}"

        launch_cmd = (
            f"source {setup_bash} && "
            f"source {ws_setup_bash} && "
            "export LIBGL_ALWAYS_SOFTWARE=1 && "
            f"ros2 launch {launch_file} "
            f"{spawn_args} "
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
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # to enable killing process group
        )

        self.launch_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.nav_button.config(state="normal")
        self.status_label.config(text="Status: Running", foreground="green")

        if x is not None and y is not None:
            print(f"Simulation launched successfully at ({x}, {y}) using {terminal}!")
        else:
            print(f"Simulation launched successfully using {terminal}!")


    def stop_simulation(self):
        if self.simulation_process is not None:
            try:
                os.killpg(os.getpgid(self.simulation_process.pid), signal.SIGTERM)
            except Exception as e:
                print(f"Error stopping simulation process group: {e}")
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
            if os.path.exists(reset_script_path):
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
        self.destroy()

def main():
    app = RobotLauncher()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()

if __name__ == "__main__":
    main()