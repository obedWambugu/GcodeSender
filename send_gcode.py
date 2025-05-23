import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import threading
import time
import queue
import json
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class GCodeSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("G-code Sender")
        self.root.geometry("1200x800")

        self.serial = None
        self.running = False
        self.paused = False
        self.queue = queue.Queue()
        self.total_lines = 0
        self.translated_file = None

        # Robot state
        self.theta = 0.0  # degrees
        self.z = 0.0      # mm
        self.r = 0.0      # mm
        self.trajectory = []  # List of (theta, z, r) for preview

        self.load_settings()

        # Serial Port Selection
        tk.Label(root, text="Serial Port:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.port_var = tk.StringVar(value=self.settings.get("port", ""))
        self.port_combo = ttk.Combobox(root, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.refresh_ports()

        self.refresh_button = tk.Button(root, text="Refresh", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=5, pady=5)

        tk.Label(root, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.baud_var = tk.StringVar(value=self.settings.get("baud", "115200"))
        tk.Entry(root, textvariable=self.baud_var, width=10).grid(row=1, column=1, padx=5, pady=5, sticky="w")

        self.connect_button = tk.Button(root, text="Connect", command=self.connect_serial)
        self.connect_button.grid(row=1, column=2, padx=5, pady=5)

        self.disconnect_button = tk.Button(root, text="Disconnect", command=self.disconnect_serial)
        self.disconnect_button.grid(row=1, column=3, padx=5, pady=5)

        # Cura G-code Upload
        tk.Label(root, text="Upload Cura G-code:").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.cura_file_var = tk.StringVar()
        tk.Entry(root, textvariable=self.cura_file_var, width=40, state="readonly").grid(row=2, column=1, padx=5, pady=5, sticky="w")
        tk.Button(root, text="Browse Cura File", command=self.browse_cura_file).grid(row=2, column=2, padx=5, pady=5)
        tk.Label(root, text="Note: Upload Cura G-code to translate.").grid(row=3, column=1, columnspan=2, padx=5, pady=2, sticky="w")

        # Standard G-code Upload
        tk.Label(root, text="G-code File:").grid(row=4, column=0, padx=5, pady=5, sticky="e")
        self.file_var = tk.StringVar(value=self.settings.get("file", ""))
        tk.Entry(root, textvariable=self.file_var, width=40, state="readonly").grid(row=4, column=1, padx=5, pady=5, sticky="w")
        tk.Button(root, text="Browse", command=self.browse_file).grid(row=4, column=2, padx=5, pady=5)
        tk.Label(root, text="Note: Use translated or compatible G-code.").grid(row=5, column=1, columnspan=2, padx=5, pady=2, sticky="w")

        # Jog Controls
        jog_frame = tk.LabelFrame(root, text="Jog Controls", padx=5, pady=5)
        jog_frame.grid(row=6, column=0, columnspan=4, padx=5, pady=5, sticky="ew")

        tk.Label(jog_frame, text="Distance (mm):").grid(row=0, column=0, padx=5, pady=2)
        self.jog_distance_var = tk.StringVar(value="10")
        self.jog_distance_combo = ttk.Combobox(jog_frame, textvariable=self.jog_distance_var, values=["0.1", "1", "10", "100"], width=8, state="readonly")
        self.jog_distance_combo.grid(row=0, column=1, padx=5, pady=2)

        tk.Label(jog_frame, text="Feedrate (mm/min):").grid(row=0, column=2, padx=5, pady=2)
        self.jog_feedrate_var = tk.StringVar(value="1000")
        tk.Entry(jog_frame, textvariable=self.jog_feedrate_var, width=10).grid(row=0, column=3, padx=5, pady=2)

        self.theta_plus_button = tk.Button(jog_frame, text="θ+", command=lambda: self.jog_axis("1", 1), width=5, state="disabled")
        self.theta_plus_button.grid(row=1, column=0, padx=2, pady=2)
        self.theta_minus_button = tk.Button(jog_frame, text="θ-", command=lambda: self.jog_axis("1", -1), width=5, state="disabled")
        self.theta_minus_button.grid(row=1, column=1, padx=2, pady=2)

        self.z_plus_button = tk.Button(jog_frame, text="Z+", command=lambda: self.jog_axis("2", 1), width=5, state="disabled")
        self.z_plus_button.grid(row=1, column=2, padx=2, pady=2)
        self.z_minus_button = tk.Button(jog_frame, text="Z-", command=lambda: self.jog_axis("2", -1), width=5, state="disabled")
        self.z_minus_button.grid(row=1, column=3, padx=2, pady=2)

        self.r_plus_button = tk.Button(jog_frame, text="R+", command=lambda: self.jog_axis("3", 1), width=5, state="disabled")
        self.r_plus_button.grid(row=1, column=4, padx=2, pady=2)
        self.r_minus_button = tk.Button(jog_frame, text="R-", command=lambda: self.jog_axis("3", -1), width=5, state="disabled")
        self.r_minus_button.grid(row=1, column=5, padx=2, pady=2)

        self.home_button = tk.Button(jog_frame, text="Home All Axes", command=self.home_axes, state="disabled")
        self.home_button.grid(row=0, column=4, columnspan=2, padx=5, pady=2)

        # Manual Command
        tk.Label(root, text="Manual Command:").grid(row=7, column=0, padx=5, pady=5, sticky="e")
        self.command_var = tk.StringVar()
        self.command_entry = tk.Entry(root, textvariable=self.command_var, width=40, state="disabled")
        self.command_entry.grid(row=7, column=1, padx=5, pady=5, sticky="w")
        self.send_button = tk.Button(root, text="Send", command=self.send_manual_command, state="disabled")
        self.send_button.grid(row=7, column=2, padx=5, pady=5)
        self.command_entry.bind("<Return>", lambda event: self.send_manual_command())

        # Controls
        self.start_button = tk.Button(root, text="Start", command=self.start_sending, state="disabled")
        self.start_button.grid(row=8, column=0, padx=5, pady=5)
        self.stop_button = tk.Button(root, text="Stop", command=self.stop_sending, state="disabled")
        self.stop_button.grid(row=8, column=1, padx=5, pady=5)
        self.progress = ttk.Progressbar(root, length=200, mode="determinate")
        self.progress.grid(row=8, column=2, padx=5, pady=5)
        self.pause_button = tk.Button(root, text="Pause", command=self.toggle_pause, state="disabled")
        self.pause_button.grid(row=8, column=3, padx=5, pady=5)

        # Output Log
        self.output_text = tk.Text(root, height=10, width=50, state="disabled")
        self.output_text.grid(row=9, column=0, columnspan=4, padx=5, pady=5)

        # 3D Visualization
        self.fig = plt.Figure(figsize=(5, 4))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().grid(row=0, column=4, rowspan=10, padx=5, pady=5, sticky="nsew")
        self.init_3d_plot()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.check_queue)

    def init_3d_plot(self):
        self.ax.clear()
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_xlim(-200, 200)
        self.ax.set_ylim(-200, 200)
        self.ax.set_zlim(0, 300)
        self.update_robot_visual()
        self.canvas.draw()

    def update_robot_visual(self):
        self.ax.clear()
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_xlim(-200, 200)
        self.ax.set_ylim(-200, 200)
        self.ax.set_zlim(0, 300)

        # Robot parameters
        base_radius = 50
        vertical_arm_length = 200
        horizontal_arm_length = 150

        # Base
        circle = np.linspace(0, 2 * np.pi, 100)
        x_base = base_radius * np.cos(circle)
        y_base = base_radius * np.sin(circle)
        z_base = np.zeros_like(circle)
        self.ax.plot(x_base, y_base, z_base, 'b-', label='Base')

        # Vertical arm
        x_vertical = [0, 0]
        y_vertical = [0, 0]
        z_vertical = [0, self.z]
        self.ax.plot(x_vertical, y_vertical, z_vertical, 'r-', linewidth=3, label='Vertical Arm')

        # Horizontal arm
        theta_rad = np.radians(self.theta)
        x_horizontal = [0, self.r * np.cos(theta_rad)]
        y_horizontal = [0, self.r * np.sin(theta_rad)]
        z_horizontal = [self.z, self.z]
        self.ax.plot(x_horizontal, y_horizontal, z_horizontal, 'g-', linewidth=3, label='Horizontal Arm')

        # End effector
        x_ee = self.r * np.cos(theta_rad)
        y_ee = self.r * np.sin(theta_rad)
        z_ee = self.z
        self.ax.scatter([x_ee], [y_ee], [z_ee], color='k', s=50, label='End Effector')

        # Trajectory preview
        if self.trajectory:
            x_traj = [r * np.cos(np.radians(theta)) for theta, _, r in self.trajectory]
            y_traj = [r * np.sin(np.radians(theta)) for theta, _, r in self.trajectory]
            z_traj = [z for _, z, _ in self.trajectory]
            self.ax.plot(x_traj, y_traj, z_traj, 'c--', alpha=0.7, label='Trajectory')

        self.ax.legend()
        self.canvas.draw()

    def parse_gcode_for_trajectory(self, file_path):
        self.trajectory = []
        current_theta = 0.0
        current_z = 0.0
        current_r = 0.0

        try:
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.split(';', 1)[0].strip().upper()
                    if not line:
                        continue
                    parts = line.split()
                    if not parts:
                        continue

                    if parts[0] == 'G28':
                        current_theta = 0.0
                        current_z = 0.0
                        current_r = 0.0
                        self.trajectory.append((current_theta, current_z, current_r))
                    elif parts[0] in ('G00', 'G01'):
                        x, y, z = None, None, None
                        for part in parts[1:]:
                            if part.startswith('X'):
                                x = float(part[1:])
                            elif part.startswith('Y'):
                                y = float(part[1:])
                            elif part.startswith('Z'):
                                z = float(part[1:])
                        if x is not None and y is not None:
                            current_r = (x**2 + y**2)**0.5
                            current_theta = np.degrees(np.arctan2(y, x))
                        if z is not None:
                            current_z = max(0, z)
                        if x is not None or y is not None or z is not None:
                            self.trajectory.append((current_theta, current_z, current_r))
                    elif parts[0].startswith('J'):
                        axis = parts[0][1]
                        distance = None
                        for part in parts[1:]:
                            if part.startswith('D'):
                                distance = float(part[1:])
                        if distance is not None:
                            if axis == '1':
                                current_theta += distance
                            elif axis == '2':
                                current_z = max(0, current_z + distance)
                            elif axis == '3':
                                current_r = max(0, current_r + distance)
                            self.trajectory.append((current_theta, current_z, current_r))
        except Exception as e:
            self.log(f"Error parsing G-code for trajectory: {e}")
            self.trajectory = []

    def load_settings(self):
        self.settings = {}
        config_file = "settings.json"
        if os.path.exists(config_file):
            try:
                with open(config_file, "r") as f:
                    self.settings = json.load(f)
            except (json.JSONDecodeError, IOError) as e:
                self.settings = {}
                self.log(f"Error loading settings: {e}")

    def save_settings(self):
        self.settings = {
            "port": self.port_var.get(),
            "baud": self.baud_var.get(),
            "file": self.file_var.get()
        }
        try:
            with open("settings.json", "w") as f:
                json.dump(self.settings, f, indent=4)
        except IOError as e:
            self.log(f"Error saving settings: {e}")

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")
            self.log("No serial ports detected.")

    def browse_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("G-code Files", "*.gcode"), ("All Files", "*.*")])
        if file_path:
            self.file_var.set(file_path)
            self.cura_file_var.set("")
            self.log(f"Selected standard G-code file: {file_path}")
            self.parse_gcode_for_trajectory(file_path)
            self.update_robot_visual()
            self.log("Trajectory preview updated")

    def browse_cura_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("G-code Files", "*.gcode"), ("All Files", "*.*")])
        if file_path:
            self.cura_file_var.set(file_path)
            self.log(f"Selected Cura G-code file: {file_path}")
            try:
                translated_path = self.translate_cura_gcode(file_path)
                self.file_var.set(translated_path)
                self.log(f"Translated Cura G-code saved as: {translated_path}")
                self.parse_gcode_for_trajectory(translated_path)
                self.update_robot_visual()
                self.log("Trajectory preview updated")
            except Exception as e:
                self.log(f"Error translating Cura G-code: {e}")
                messagebox.showerror("Error", f"Failed to translate Cura G-code: {e}")

    def translate_cura_gcode(self, input_path):
        output_path = "translated.gcode"
        max_feedrate = 1000.0

        output_lines = ["G28\n", "G90\n"]
        current_f = max_feedrate

        with open(input_path, 'r') as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip()
            if not line or line.startswith(';'):
                continue
            if line.startswith(('M104', 'M105', 'M109', 'M82', 'M107', 'G92')):
                continue
            if line.startswith(('G0 ', 'G1 ')):
                parts = line.split()
                cmd = 'G00' if line.startswith('G0') else 'G01'
                x, y, z, f = None, None, None, None
                for part in parts[1:]:
                    if part.startswith('X'):
                        x = float(part[1:])
                    elif part.startswith('Y'):
                        y = float(part[1:])
                    elif part.startswith('Z'):
                        z = float(part[1:])
                    elif part.startswith('F'):
                        f = float(part[1:])
                if x is not None or y is not None or z is not None:
                    if f is not None:
                        current_f = min(f, max_feedrate)
                    new_line = f"{cmd} "
                    if x is not None:
                        new_line += f"X{x:.3f} "
                    if y is not None:
                        new_line += f"Y{y:.3f} "
                    if z is not None:
                        new_line += f"Z{z:.3f} "
                    new_line += f"F{current_f}"
                    output_lines.append(new_line + "\n")
            elif line == 'G90':
                output_lines.append("G90\n")
            elif line == 'G28':
                output_lines.append("G28\n")

        output_lines.append("M114\n")

        with open(output_path, 'w') as f:
            f.writelines(output_lines)

        return output_path

    def jog_axis(self, axis, direction):
        if not self.serial or not self.serial.is_open:
            self.log("Not connected. Please connect first.")
            return
        if self.running:
            self.log("Cannot jog while G-code is running.")
            return

        try:
            distance = float(self.jog_distance_var.get())
            feedrate = float(self.jog_feedrate_var.get())
        except ValueError:
            self.log("Invalid jog distance or feedrate.")
            return

        distance *= direction
        command = f"J{axis} D{distance:.3f} F{feedrate}"
        self.log(f"Jogging {'θ' if axis=='1' else 'Z' if axis=='2' else 'R'} {'+' if direction > 0 else '-'}{abs(distance)} {'deg' if axis=='1' else 'mm'} at {feedrate} {'deg/min' if axis=='1' else 'mm/min'}")
        
        if axis == "1":
            self.theta += distance
        elif axis == "2":
            self.z = max(0, self.z + distance)
        elif axis == "3":
            self.r = max(0, self.r + distance)
        self.update_robot_visual()
        
        self.send_manual_command(command)

    def home_axes(self):
        if not self.serial or not self.serial.is_open:
            self.log("Not connected. Please connect first.")
            return
        if self.running:
            self.log("Cannot home while G-code is running.")
            return

        self.log("Homing all axes")
        self.theta = 0.0
        self.z = 0.0
        self.r = 0.0
        self.update_robot_visual()
        self.send_manual_command("G28")

    def send_manual_command(self, command=None):
        if not self.serial or not self.serial.is_open:
            self.log("Not connected. Please connect first.")
            return

        if command is None:
            command = self.command_var.get().strip()
            self.command_var.set("")
        if not command:
            self.log("No command entered")
            return

        try:
            parts = command.upper().split()
            if parts[0].startswith('J'):
                axis = parts[0][1]
                distance = None
                for part in parts[1:]:
                    if part.startswith('D'):
                        distance = float(part[1:])
                if distance is not None:
                    if axis == "1":
                        self.theta += distance
                    elif axis == "2":
                        self.z = max(0, self.z + distance)
                    elif axis == "3":
                        self.r = max(0, self.r + distance)
                    self.update_robot_visual()
            elif parts[0] == "G28":
                self.theta = 0.0
                self.z = 0.0
                self.r = 0.0
                self.update_robot_visual()
        except Exception as e:
            self.log(f"Error parsing command for visualization: {e}")

        try:
            self.log(f"Sending command: {command}")
            self.serial.write((command + '\n').encode('utf-8'))
            self.serial.flush()
            start_time = time.time()
            prompt_seen = False
            while time.time() - start_time < 60:
                if self.serial.in_waiting > 0:
                    raw_data = self.serial.readline()
                    try:
                        response = raw_data.decode('utf-8').strip()
                        self.log(f"Received: {response}")
                        if 'ready>' in response:
                            prompt_seen = True
                            break
                    except UnicodeDecodeError:
                        self.log(f"Decode error: {raw_data.hex()}")
                time.sleep(0.01)
            if not prompt_seen:
                self.log("No prompt received after command")
        except serial.SerialException as e:
            self.log(f"Error sending command: {e}")

    def log(self, message):
        self.queue.put(message)

    def check_queue(self):
        while not self.queue.empty():
            message = self.queue.get()
            self.output_text.config(state="normal")
            self.output_text.insert(tk.END, message + "\n")
            self.output_text.see(tk.END)
            self.output_text.config(state="disabled")
        self.root.after(100, self.check_queue)

    def connect_serial(self):
        if not self.port_var.get():
            messagebox.showerror("Error", "Please select a serial port.")
            return
        try:
            baud_rate = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid baud rate.")
            return

        if self.serial and self.serial.is_open:
            self.log("Already connected.")
            return

        try:
            self.serial = serial.Serial(self.port_var.get(), baud_rate, timeout=1)
            self.log(f"Connected to {self.port_var.get()} at {baud_rate} baud")
            time.sleep(2)
            self.serial.flush()

            start_time = time.time()
            while time.time() - start_time < 5:
                if self.serial.in_waiting > 0:
                    raw_data = self.serial.readline()
                    try:
                        response = raw_data.decode('utf-8').strip()
                        self.log(f"Arduino: {response}")
                        if 'ready>' in response:
                            break
                    except UnicodeDecodeError:
                        self.log(f"Initial noise: {raw_data.hex()}")
                time.sleep(0.05)
            else:
                self.log("No prompt received within 5 seconds")

            self.connect_button.config(state="disabled")
            self.start_button.config(state="normal")
            self.stop_button.config(state="normal")
            self.command_entry.config(state="normal")
            self.send_button.config(state="normal")
            self.theta_plus_button.config(state="normal")
            self.theta_minus_button.config(state="normal")
            self.z_plus_button.config(state="normal")
            self.z_minus_button.config(state="normal")
            self.r_plus_button.config(state="normal")
            self.r_minus_button.config(state="normal")
            self.home_button.config(state="normal")
            self.log("Ready for commands, jogging, or file sending")
        except serial.SerialException as e:
            self.log(f"Error opening serial port: {e}")
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.serial = None

    def disconnect_serial(self):
        self.running = False
        self.paused = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.log("Serial connection closed")
            self.serial = None
            self.connect_button.config(state="normal")
            self.start_button.config(state="disabled")
            self.stop_button.config(state="disabled")
            self.command_entry.config(state="disabled")
            self.send_button.config(state="disabled")
            self.theta_plus_button.config(state="disabled")
            self.theta_minus_button.config(state="disabled")
            self.z_plus_button.config(state="disabled")
            self.z_minus_button.config(state="disabled")
            self.r_plus_button.config(state="disabled")
            self.r_minus_button.config(state="disabled")
            self.home_button.config(state="disabled")

    def start_sending(self):
        if not self.serial or not self.serial.is_open:
            self.log("Not connected. Please connect first.")
            return
        if not self.file_var.get():
            messagebox.showerror("Error", "Please select a G-code file.")
            return

        try:
            with open(self.file_var.get(), 'r') as f:
                self.total_lines = sum(1 for line in f if line.split(';', 1)[0].strip())
            self.progress["maximum"] = self.total_lines
            self.progress["value"] = 0
            self.log(f"File loaded: {self.total_lines} lines")
        except Exception as e:
            self.log(f"Error counting lines: {e}")
            return

        self.start_button.config(state="disabled")
        self.pause_button.config(state="normal")
        self.theta_plus_button.config(state="disabled")
        self.theta_minus_button.config(state="disabled")
        self.z_plus_button.config(state="disabled")
        self.z_minus_button.config(state="disabled")
        self.r_plus_button.config(state="disabled")
        self.r_minus_button.config(state="disabled")
        self.home_button.config(state="disabled")
        self.running = True
        self.paused = False
        self.log(f"Starting G-code transmission from {self.file_var.get()}")
        
        threading.Thread(target=self.send_gcode_thread, daemon=True).start()

    def stop_sending(self):
        self.running = False
        self.paused = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.pause_button.config(state="disabled")
        self.pause_button.config(text="Pause")
        self.progress["value"] = 0
        self.command_entry.config(state="disabled")
        self.send_button.config(state="disabled")
        self.connect_button.config(state="normal")
        self.theta_plus_button.config(state="disabled")
        self.theta_minus_button.config(state="disabled")
        self.z_plus_button.config(state="disabled")
        self.z_minus_button.config(state="disabled")
        self.r_plus_button.config(state="disabled")
        self.r_minus_button.config(state="disabled")
        self.home_button.config(state="disabled")
        self.log("Stopped by user.")
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
            self.log("Serial connection closed")

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.config(text="Resume" if self.paused else "Pause")
        self.log("Paused" if self.paused else "Resumed")

    def send_gcode_thread(self):
        try:
            with open(self.file_var.get(), 'r') as f:
                for line_number, line in enumerate(f, 1):
                    if not self.running:
                        self.log("Transmission stopped")
                        break
                    while self.paused and self.running:
                        time.sleep(0.1)
                    if not self.running:
                        self.log("Transmission stopped")
                        break
                    try:
                        self.send_gcode_line(line, line_number)
                        self.progress["value"] = line_number
                        self.root.update_idletasks()
                    except serial.SerialTimeoutException:
                        self.log(f"Timeout on line {line_number}: {line.strip()}")
                        break
                    except Exception as e:
                        self.log(f"Error on line {line_number}: {e}")
                        break
            self.log("G-code transmission complete")
        except FileNotFoundError:
            self.log(f"Error: G-code file '{self.file_var.get()}' not found")
        except Exception as e:
            self.log(f"Error reading file: {e}")

        self.start_button.config(state="normal")
        self.stop_button.config(state="normal")
        self.pause_button.config(state="disabled")
        self.pause_button.config(text="Pause")
        self.running = False
        self.paused = False
        self.progress["value"] = 0
        if self.serial and self.serial.is_open:
            self.theta_plus_button.config(state="normal")
            self.theta_minus_button.config(state="normal")
            self.z_plus_button.config(state="normal")
            self.z_minus_button.config(state="normal")
            self.r_plus_button.config(state="normal")
            self.r_minus_button.config(state="normal")
            self.home_button.config(state="normal")

    def send_gcode_line(self, line, line_number):
        line = line.split(';', 1)[0].strip()
        if not line:
            return

        try:
            parts = line.upper().split()
            if parts[0] == 'G28':
                self.theta = 0.0
                self.z = 0.0
                self.r = 0.0
            elif parts[0] in ('G00', 'G01'):
                x, y, z = None, None, None
                for part in parts[1:]:
                    if part.startswith('X'):
                        x = float(part[1:])
                    elif part.startswith('Y'):
                        y = float(part[1:])
                    elif part.startswith('Z'):
                        z = float(part[1:])
                if x is not None and y is not None:
                    self.r = (x**2 + y**2)**0.5
                    self.theta = np.degrees(np.arctan2(y, x))
                if z is not None:
                    self.z = max(0, z)
            elif parts[0].startswith('J'):
                axis = parts[0][1]
                distance = None
                for part in parts[1:]:
                    if part.startswith('D'):
                        distance = float(part[1:])
                if distance is not None:
                    if axis == '1':
                        self.theta += distance
                    elif axis == '2':
                        self.z = max(0, self.z + distance)
                    elif axis == '3':
                        self.r = max(0, self.r + distance)
            self.update_robot_visual()
        except Exception as e:
            self.log(f"Error parsing line {line_number} for visualization: {e}")

        try:
            self.log(f"Sending line {line_number}: {line}")
            self.serial.write((line + '\n').encode('utf-8'))
            self.serial.flush()
            start_time = time.time()
            prompt_seen = False
            while time.time() - start_time < 60:
                if self.serial.in_waiting > 0:
                    raw_data = self.serial.readline()
                    try:
                        response = raw_data.decode('utf-8').strip()
                        self.log(f"Received: {response}")
                        if 'ready>' in response:
                            prompt_seen = True
                            break
                    except UnicodeDecodeError:
                        self.log(f"Decode error: {raw_data.hex()}")
                time.sleep(0.01)
            if not prompt_seen:
                self.log(f"No prompt received for line {line_number} - check Arduino")
        except serial.SerialException as e:
            self.log(f"Error sending line {line_number}: {e}")
            raise

    def on_closing(self):
        self.running = False
        self.paused = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.log("Serial connection closed")
        self.save_settings()
        plt.close(self.fig)
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = GCodeSenderApp(root)
    root.mainloop()