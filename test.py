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
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class GCodeSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("G-code Sender")
        self.root.geometry("1000x800")

        self.serial = None
        self.running = False
        self.paused = False
        self.queue = queue.Queue()  # For log messages
        self.plot_queue = queue.Queue()  # For thread-safe plot updates
        self.total_lines = 0
        self.translated_file = None

        # Initialize joint positions and current Cartesian state
        self.joints = {'theta1': 0.0, 'd2': 0.0, 'd3': 0.0}  # θ1 (deg), d2 (mm), d3 (mm)
        self.current_pos = {'X': 0.0, 'Y': 0.0, 'Z': 300.0}  # Start at base height
        self.positions = []  # Store end effector position history
        # Robot dimensions
        self.base_height = 0.0  # mm
        self.d2_max = 1000.0  # mm (vertical arm)
        self.d3_max = 1000.0  # mm (radial arm)
        self.z_offset = self.base_height  # Map Cura Z=0 to robot Z=300 mm

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

        tk.Label(jog_frame, text="Distance (mm/deg):").grid(row=0, column=0, padx=5, pady=2)
        self.jog_distance_var = tk.StringVar(value="10")
        self.jog_distance_combo = ttk.Combobox(jog_frame, textvariable=self.jog_distance_var, values=["0.1", "1", "10", "100"], width=8, state="readonly")
        self.jog_distance_combo.grid(row=0, column=1, padx=5, pady=2)

        tk.Label(jog_frame, text="Feedrate (mm/min or deg/min):").grid(row=0, column=2, padx=5, pady=2)
        self.jog_feedrate_var = tk.StringVar(value="1000")
        tk.Entry(jog_frame, textvariable=self.jog_feedrate_var, width=10).grid(row=0, column=3, padx=5, pady=2)

        # θ1 Axis (J1)
        self.theta1_plus_button = tk.Button(jog_frame, text="θ1+", command=lambda: self.jog_axis("1", 1), width=5, state="disabled")
        self.theta1_plus_button.grid(row=1, column=0, padx=2, pady=2)
        self.theta1_minus_button = tk.Button(jog_frame, text="θ1-", command=lambda: self.jog_axis("1", -1), width=5, state="disabled")
        self.theta1_minus_button.grid(row=1, column=1, padx=2, pady=2)

        # d2 Axis (J2, vertical)
        self.d2_plus_button = tk.Button(jog_frame, text="d2+", command=lambda: self.jog_axis("2", 1), width=5, state="disabled")
        self.d2_plus_button.grid(row=1, column=2, padx=2, pady=2)
        self.d2_minus_button = tk.Button(jog_frame, text="d2-", command=lambda: self.jog_axis("2", -1), width=5, state="disabled")
        self.d2_minus_button.grid(row=1, column=3, padx=2, pady=2)

        # d3 Axis (J3, radial)
        self.d3_plus_button = tk.Button(jog_frame, text="d3+", command=lambda: self.jog_axis("3", 1), width=5, state="disabled")
        self.d3_plus_button.grid(row=1, column=4, padx=2, pady=2)
        self.d3_minus_button = tk.Button(jog_frame, text="d3-", command=lambda: self.jog_axis("3", -1), width=5, state="disabled")
        self.d3_minus_button.grid(row=1, column=5, padx=2, pady=2)

        # Home Button
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
        self.preview_button = tk.Button(root, text="Preview Trajectory", command=self.preview_trajectory)
        self.preview_button.grid(row=8, column=4, padx=5, pady=5)
        self.save_traj_button = tk.Button(root, text="Save Trajectory", command=self.save_trajectory)
        self.save_traj_button.grid(row=8, column=5, padx=5, pady=5)

        # Output Log
        self.output_text = tk.Text(root, height=15, width=80, state="disabled")
        self.output_text.grid(row=9, column=0, columnspan=4, padx=5, pady=5)

        # Initialize 3D visualization
        self.fig = None
        self.ax = None
        self.init_3d_plot()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(10, self.check_queues)

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

    def browse_cura_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("G-code Files", "*.gcode"), ("All Files", "*.*")])
        if file_path:
            self.cura_file_var.set(file_path)
            self.log(f"Selected Cura G-code file: {file_path}")
            try:
                translated_path = self.translate_cura_gcode(file_path)
                self.file_var.set(translated_path)
                self.log(f"Translated Cura G-code saved as: {translated_path}")
            except Exception as e:
                self.log(f"Error translating Cura G-code: {e}")
                messagebox.showerror("Error", f"Failed to translate Cura G-code: {e}")

    def translate_cura_gcode(self, input_path):
        output_path = "C:/Users/Obed Wambugu/Documents/Gcode Sender/newtranslated.gcode"
        max_feedrate = 1000.0

        output_lines = ["G28\n", "G90\n"]
        current_pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        current_f = max_feedrate

        # Offset to center print (adjust based on problematic coordinates)
        x_offset = 700.0  # Shift X to reduce d3
        y_offset = 500.0  # Shift Y to reduce d3

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
                x, y, z, f = current_pos['X'], current_pos['Y'], current_pos['Z'], None
                for part in parts[1:]:
                    if part.startswith('X'):
                        x = float(part[1:])
                    elif part.startswith('Y'):
                        y = float(part[1:])
                    elif part.startswith('Z'):
                        z = float(part[1:])
                    elif part.startswith('F'):
                        f = float(part[1:])
                current_pos.update({'X': x, 'Y': y, 'Z': z})
                if f is not None:
                    current_f = min(f, max_feedrate)
                # Apply offsets
                x_trans = x - x_offset
                y_trans = y - y_offset
                z_trans = z + self.z_offset
                # Check workspace
                d2 = z_trans - self.base_height
                d3 = np.sqrt(x_trans**2 + y_trans**2)
                if 0 <= d2 <= self.d2_max and d3 <= self.d3_max:
                    new_line = f"{cmd} X{x_trans:.3f} Y{y_trans:.3f} Z{z_trans:.3f} F{current_f}"
                    output_lines.append(new_line + "\n")
                else:
                    self.log(f"Warning: Translated position (X={x_trans}, Y={y_trans}, Z={z_trans}) out of range (d2: [0, {self.d2_max}], d3: [0, {self.d3_max}])")
            elif line == 'G90':
                output_lines.append("G90\n")
            elif line == 'G28':
                current_pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
                output_lines.append("G28\n")

        output_lines.append("M114\n")

        with open(output_path, 'w') as f:
            f.writelines(output_lines)

        return output_path

    def forward_kinematics(self, theta1, d2, d3):
        theta1_rad = np.radians(theta1)
        x = d3 * np.cos(theta1_rad)
        y = d3 * np.sin(theta1_rad)
        z = self.base_height + d2
        return x, y, z

    def init_3d_plot(self):
        self.fig = plt.figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('RPP Robot (Base: 300 mm, d2: Vertical, d3: Radial)')
        self.ax.set_xlim([-1200, 1200])
        self.ax.set_ylim([-1200, 1200])
        self.ax.set_zlim([0, 1500])
        x, y, z = self.forward_kinematics(self.joints['theta1'], self.joints['d2'], self.joints['d3'])
        self.positions.append([x, y, z])
        self.update_3d_plot()
        plt.ion()
        plt.show()

    def update_3d_plot(self):
        self.ax.clear()
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('RPP Robot (Base: 300 mm, d2: Vertical, d3: Radial)')
        self.ax.set_xlim([-1200, 1200])
        self.ax.set_ylim([-1200, 1200])
        self.ax.set_zlim([0, 1500])

        theta1 = self.joints['theta1']
        d2 = np.clip(self.joints['d2'], 0, self.d2_max)
        d3 = np.clip(self.joints['d3'], 0, self.d3_max)
        theta1_rad = np.radians(theta1)

        # Base
        r = 50
        u = np.linspace(0, 2 * np.pi, 20)
        h = np.linspace(0, self.base_height, 10)
        x_base = r * np.outer(np.cos(u), np.ones(len(h)))
        y_base = r * np.outer(np.sin(u), np.ones(len(h)))
        z_base = np.outer(np.ones(len(u)), h)
        self.ax.plot_surface(x_base, y_base, z_base, color='gray', alpha=0.3)
        self.ax.text(0, 0, self.base_height / 2, f'Base\n{self.base_height} mm', color='black', fontsize=10, ha='center')

        # First arm (d2)
        x0, y0, z0 = 0, 0, self.base_height
        x1, y1, z1 = x0, y0, self.base_height + d2
        self.ax.plot([x0, x1], [y0, y1], [z0, z1], 'g-', linewidth=5, label='d2 (Vertical)')
        self.ax.text(x1 + 20, y1, (z0 + z1) / 2, f'd2: {d2:.1f} mm', color='green', fontsize=10, va='center')

        # Second arm (d3)
        x2 = x1 + d3 * np.cos(theta1_rad)
        y2 = y1 + d3 * np.sin(theta1_rad)
        z2 = z1
        self.ax.plot([x1, x2], [y1, y2], [z1, z2], 'b-', linewidth=5, label='d3 (Radial)')
        self.ax.text((x1 + x2) / 2, (y1 + y2) / 2, z2 + 20, f'd3: {d3:.1f} mm', color='blue', fontsize=10, ha='center')

        # End effector
        self.ax.scatter([x2], [y2], [z2], color='red', s=100, label='End Effector')

        # Trajectory
        pos_array = np.array(self.positions[-1000:])
        if len(pos_array) > 1:
            self.ax.plot(pos_array[:, 0], pos_array[:, 1], pos_array[:, 2], 'b--', label='Trajectory')

        self.ax.legend(loc='upper left')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def parse_command_for_position(self, command):
        command = command.strip()
        if command.startswith(('G0', 'G1', 'G00', 'G01')):
            parts = command.split()
            x, y, z = self.current_pos['X'], self.current_pos['Y'], self.current_pos['Z']
            update = False
            for part in parts[1:]:
                if part.startswith('X'):
                    x = float(part[1:])
                    update = True
                elif part.startswith('Y'):
                    y = float(part[1:])
                    update = True
                elif part.startswith('Z'):
                    z = float(part[1:])
                    update = True
            if update:
                d2 = z - self.base_height
                d3 = np.sqrt(x**2 + y**2)
                if 0 <= d2 <= self.d2_max and d3 <= self.d3_max:
                    self.joints['d2'] = d2
                    self.joints['d3'] = d3
                    self.joints['theta1'] = np.degrees(np.arctan2(y, x)) if d3 > 0.001 else self.joints['theta1']
                    self.current_pos.update({'X': x, 'Y': y, 'Z': z})
                    self.log(f"Parsed G0/G1: θ1={self.joints['theta1']:.1f}°, d2={d2:.1f} mm, d3={d3:.1f} mm")
                    x_pos, y_pos, z_pos = self.forward_kinematics(self.joints['theta1'], self.joints['d2'], self.joints['d3'])
                    self.positions.append([x_pos, y_pos, z_pos])
                    return True
                else:
                    self.log(f"Warning: Position (X={x}, Y={y}, Z={z}) out of range (d2: [0, {self.d2_max}], d3: [0, {self.d3_max}])")
                    return False
            else:
                self.log(f"Skipped empty G0/G1: {command}")
                return False
        elif command.startswith('J'):
            parts = command.split()
            if len(parts) >= 3:
                axis = parts[0][1:]
                distance = float(parts[1][1:])
                if axis == '1':
                    self.joints['theta1'] += distance
                elif axis == '2':
                    self.joints['d2'] = np.clip(self.joints['d2'] + distance, 0, self.d2_max)
                elif axis == '3':
                    self.joints['d3'] = np.clip(self.joints['d3'] + distance, 0, self.d3_max)
                x, y, z = self.forward_kinematics(self.joints['theta1'], self.joints['d2'], self.joints['d3'])
                self.current_pos.update({'X': x, 'Y': y, 'Z': z})
                self.positions.append([x, y, z])
                return True
            else:
                self.log(f"Skipped invalid jog: {command}")
                return False
        elif command == 'G28':
            self.joints = {'theta1': 0.0, 'd2': 0.0, 'd3': 0.0}
            self.current_pos = {'X': 0.0, 'Y': 0.0, 'Z': self.base_height}
            self.positions.append([0, 0, self.base_height])
            self.log("Parsed G28: Homing to θ1=0°, d2=0 mm, d3=0 mm")
            return True
        else:
            self.log(f"Skipped unsupported command: {command}")
            return False

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
        self.log(f"Jogging {'θ1' if axis=='1' else 'd2' if axis=='2' else 'd3'} {'+' if direction > 0 else '-'}{abs(distance)} {'deg' if axis=='1' else 'mm'} at {feedrate} {'deg/min' if axis=='1' else 'mm/min'}")
        if self.parse_command_for_position(command):
            self.update_3d_plot()
            self.send_manual_command(command)

    def home_axes(self):
        if not self.serial or not self.serial.is_open:
            self.log("Not connected. Please connect first.")
            return
        if self.running:
            self.log("Cannot home while G-code is running.")
            return

        self.log("Homing all axes")
        self.parse_command_for_position("G28")
        self.update_3d_plot()
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

        if self.parse_command_for_position(command):
            self.update_3d_plot()

        try:
            self.log(f"Sending command: {command}")
            self.serial.write((command + '\n').encode('utf-8'))
            self.serial.flush()
            start_time = time.time()
            prompt_seen = False
            while time.time() - start_time < 3600:
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

    def preview_trajectory(self):
        if not self.file_var.get():
            messagebox.showerror("Error", "Please select a G-code file.")
            return

        self.positions = []
        self.joints = {'theta1': 0.0, 'd2': 0.0, 'd3': 0.0}
        self.current_pos = {'X': 0.0, 'Y': 0.0, 'Z': self.base_height}
        try:
            with open(self.file_var.get(), 'r') as f:
                for line_number, line in enumerate(f, 1):
                    line = line.split(';', 1)[0].strip()
                    if not line:
                        continue
                    if self.parse_command_for_position(line):
                        self.log(f"Preview line {line_number}: {line}")
                    else:
                        self.log(f"Preview line {line_number} failed: {line}")
            self.update_3d_plot()
            self.log(f"Trajectory preview complete with {len(self.positions)} points")
        except Exception as e:
            self.log(f"Error previewing file: {e}")
            messagebox.showerror("Error", f"Failed to preview trajectory: {e}")

    def save_trajectory(self):
        if not self.positions:
            messagebox.showwarning("Warning", "No trajectory data to save.")
            return
        file_path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")])
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    f.write("X,Y,Z\n")
                    for x, y, z in self.positions:
                        f.write(f"{x:.3f},{y:.3f},{z:.3f}\n")
                self.log(f"Trajectory saved to {file_path}")
            except Exception as e:
                self.log(f"Error saving trajectory: {e}")
                messagebox.showerror("Error", f"Failed to save trajectory: {e}")

    def log(self, message):
        self.queue.put(message)

    def check_queues(self):
        while not self.queue.empty():
            message = self.queue.get()
            self.output_text.config(state="normal")
            self.output_text.insert(tk.END, message + "\n")
            self.output_text.see(tk.END)
            self.output_text.config(state="disabled")

        updated = False
        while not self.plot_queue.empty():
            joints = self.plot_queue.get()
            self.joints.update(joints)
            updated = True
        if updated:
            self.log(f"Updating plot: θ1={self.joints['theta1']:.1f}°, d2={self.joints['d2']:.1f} mm, d3={self.joints['d3']:.1f} mm")
            self.update_3d_plot()

        self.root.after(10, self.check_queues)

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
            self.theta1_plus_button.config(state="normal")
            self.theta1_minus_button.config(state="normal")
            self.d2_plus_button.config(state="normal")
            self.d2_minus_button.config(state="normal")
            self.d3_plus_button.config(state="normal")
            self.d3_minus_button.config(state="normal")
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
            self.theta1_plus_button.config(state="disabled")
            self.theta1_minus_button.config(state="disabled")
            self.d2_plus_button.config(state="disabled")
            self.d2_minus_button.config(state="disabled")
            self.d3_plus_button.config(state="disabled")
            self.d3_minus_button.config(state="disabled")
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
        self.theta1_plus_button.config(state="disabled")
        self.theta1_minus_button.config(state="disabled")
        self.d2_plus_button.config(state="disabled")
        self.d2_minus_button.config(state="disabled")
        self.d3_plus_button.config(state="disabled")
        self.d3_minus_button.config(state="disabled")
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
        self.theta1_plus_button.config(state="disabled")
        self.theta1_minus_button.config(state="disabled")
        self.d2_plus_button.config(state="disabled")
        self.d2_minus_button.config(state="disabled")
        self.d3_plus_button.config(state="disabled")
        self.d3_minus_button.config(state="disabled")
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
            self.theta1_plus_button.config(state="normal")
            self.theta1_minus_button.config(state="normal")
            self.d2_plus_button.config(state="normal")
            self.d2_minus_button.config(state="normal")
            self.d3_plus_button.config(state="normal")
            self.d3_minus_button.config(state="normal")
            self.home_button.config(state="normal")

    def send_gcode_line(self, line, line_number):
        line = line.split(';', 1)[0].strip()
        if not line:
            return

        try:
            self.log(f"Sending line {line_number}: {line}")
            if self.parse_command_for_position(line):
                self.plot_queue.put(self.joints.copy())
            self.serial.write((line + '\n').encode('utf-8'))
            self.serial.flush()
            start_time = time.time()
            prompt_seen = False
            while time.time() - start_time < 3600:
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