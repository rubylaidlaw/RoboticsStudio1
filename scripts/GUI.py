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


class EstopPublisher(Node):
    """ROS2 Node to publish e-stop status"""
    def __init__(self):
        super().__init__('estop_publisher')
        self.publisher = self.create_publisher(Bool, '/estop_status', 10)
        
    def publish_estop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.publisher.publish(msg)
        self.get_logger().info(f'Published e-stop: {active}')


class RobotLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Foxtrack Rover Control")
        self.geometry("500x400")
        
        self.estop_active = False
        self.simulation_process = None
        self.nav_process = None
        
        # Initialize ROS2 node in separate thread
        self.ros_node = None
        self.init_ros()
        
        # Create GUI widgets
        self.create_widgets()
        
    def init_ros(self):
        """Initialize ROS2 node for e-stop publishing"""
        try:
            rclpy.init()
            self.ros_node = EstopPublisher()
            
            # Spin ROS2 node in separate thread
            self.ros_thread = threading.Thread(
                target=lambda: rclpy.spin(self.ros_node),
                daemon=True
            )
            self.ros_thread.start()
            print("ROS2 e-stop publisher initialized")
        except Exception as e:
            print(f"Failed to initialize ROS2: {e}")
            self.ros_node = None
        
    def create_widgets(self):
        """Create all GUI elements"""
        # Title
        title_label = ttk.Label(
            self, 
            text="Foxtrack Rover Control Panel",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=20)
        
        # Launch Controls Frame
        launch_frame = ttk.LabelFrame(self, text="Simulation Control", padding=10)
        launch_frame.pack(pady=10, padx=20, fill="x")
        
        # Launch button
        self.launch_button = ttk.Button(
            launch_frame,
            text="Launch Simulation",
            command=self.launch_simulation,
            width=30
        )
        self.launch_button.pack(pady=5)
        
        # Stop simulation button
        self.stop_button = ttk.Button(
            launch_frame,
            text="⏹ Stop Simulation",
            command=self.stop_simulation,
            state="disabled",
            width=30
        )
        self.stop_button.pack(pady=5)
        
        # Status label
        self.status_label = ttk.Label(
            launch_frame,
            text="Status: Not Running",
            foreground="red"
        )
        self.status_label.pack(pady=5)
        
        # Emergency Stop Frame
        estop_frame = ttk.LabelFrame(self, text="Emergency Control", padding=10)
        estop_frame.pack(pady=10, padx=20, fill="x")
        
        # E-Stop button
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
        
        # Navigation Control Frame
        nav_frame = ttk.LabelFrame(self, text="Navigation Control", padding=10)
        nav_frame.pack(pady=10, padx=20, fill="x")
        
        # Start navigation button
        self.nav_button = ttk.Button(
            nav_frame,
            text="▶ Start Navigation",
            command=self.start_navigation,
            state="disabled",
            width=30
        )
        self.nav_button.pack(pady=5)
        
    def get_terminal_emulator(self):
        """Detect available terminal emulator"""
        terminals = [
            'gnome-terminal',
            'konsole',
            'xfce4-terminal',
            'xterm',
            'terminator'
        ]
        
        for term in terminals:
            if shutil.which(term):
                return term
        
        return None
        
    def launch_simulation(self):
        """Launch the entire ROS2 simulation stack"""
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
        
        try:
            # Source ROS2 workspace and launch
            launch_cmd = (
                "source /opt/ros/humble/setup.bash && "
                "source ~/41068_ws/install/setup.bash && "
                "export LIBGL_ALWAYS_SOFTWARE=1 && "
                "ros2 launch 41068_ignition_bringup 41068_ignition.launch.py "
                "slam:=true nav2:=true rviz:=true world:=large_demo; "
                "exec bash"
            )
            
            # Launch based on terminal type
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
            
            # Update GUI
            self.launch_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.nav_button.config(state="normal")
            self.status_label.config(text="Status: Running", foreground="green")
            
            print(f"Simulation launched successfully using {terminal}!")
            
        except Exception as e:
            print(f"Failed to launch simulation: {e}")
            self.status_label.config(text=f"Status: Error - {e}", foreground="red")
    
    def stop_simulation(self):
        """Stop the simulation"""
        if self.simulation_process is not None:
            try:
                # Kill the entire process group
                os.killpg(os.getpgid(self.simulation_process.pid), signal.SIGTERM)
            except:
                try:
                    self.simulation_process.terminate()
                    self.simulation_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.simulation_process.kill()
            
            self.simulation_process = None
            
            # Update GUI
            self.launch_button.config(state="normal")
            self.stop_button.config(state="disabled")
            self.nav_button.config(state="disabled")
            self.status_label.config(text="Status: Not Running", foreground="red")
            
            print("Simulation stopped")
    
    def start_navigation(self):
        """Start the autonomous navigation script"""
        try:
            # Launch send_goal.py in background
            self.nav_process = subprocess.Popen(
                ['python3', os.path.expanduser('~/41068_ws/src/RoboticsStudio1/scripts/send_goal.py')],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("Navigation started")
        except Exception as e:
            print(f"Failed to start navigation: {e}")
    
    def toggle_estop(self):
        """Toggle emergency stop"""
        self.estop_active = not self.estop_active
        
        if self.estop_active:
            self.estop_button.config(text="Resume")
            self.estop_status_label.config(text="E-Stop: ACTIVE", foreground="red")
            print("Emergency Stop ACTIVATED!")
        else:
            self.estop_button.config(text="Emergency Stop")
            self.estop_status_label.config(text="E-Stop: Inactive", foreground="green")
            print("Emergency Stop released")
        
        # Publish e-stop status to ROS2 topic
        if self.ros_node is not None:
            self.ros_node.publish_estop(self.estop_active)
    
    def on_closing(self):
        """Clean up when window is closed"""
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
