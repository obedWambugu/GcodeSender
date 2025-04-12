import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import threading
import time
import queue
import json
import os

class GCodeSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("G-code Sender")
        self.root.geometry("700x700")

        self.serial = None
        self.running = False
        self.paused = False
        self.queue = queue.Queue()
        self.total_lines = 0

        self.load_settings()

        tk.Label(root, text="Serial Port:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.port_var = tk.StringVar(value=self.settings.get("port", ""))
        self.port_combo = ttk.Combobox(root, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.refresh_ports()

        tk.Label(root, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.baud_var = tk.StringVar(value=self.settings.get("baud", "115200"))
        tk.Entry(root, textvariable=self.baud_var, width=10).grid(row=1, column=1, padx=5, pady=5, sticky="w")

        self.connect_button = tk.Button(root, text="Connect", command=self.connect_serial)
        self.connect_button.grid(row=1, column=2, padx=5, pady=5)

        tk.Label(root, text="G-code File:").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.file_var = tk.StringVar(value=self.settings.get("file", ""))
        tk.Entry(root, textvariable=self.file_var, width=40, state="readonly").grid(row=2, column=1, padx=5, pady=5, sticky="w")
        tk.Button(root, text="Browse", command=self.browse_file).grid(row=2, column=2, padx=5, pady=5)

        tk.Label(root, text="Manual Command:").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        self.command_var = tk.StringVar()
        self.command_entry = tk.Entry(root, textvariable=self.command_var, width=40, state="disabled")
        self.command_entry.grid(row=3, column=1, padx=5, pady=5, sticky="w")
        self.send_button = tk.Button(root, text="Send", command=self.send_manual_command, state="disabled")
        self.send_button.grid(row=3, column=2, padx=5, pady=5)
        self.command_entry.bind("<Return>", lambda event: self.send_manual_command())

        self.start_button = tk.Button(root, text="Start", command=self.start_sending, state="disabled")
        self.start_button.grid(row=4, column=0, padx=5, pady=5)
        self.stop_button = tk.Button(root, text="Stop", command=self.stop_sending, state="disabled")
        self.stop_button.grid(row=4, column=1, padx=5, pady=5)
        self.progress = ttk.Progressbar(root, length=200, mode="determinate")
        self.progress.grid(row=4, column=2, padx=5, pady=5)
        self.pause_button = tk.Button(root, text="Pause", command=self.toggle_pause, state="disabled")
        self.pause_button.grid(row=4, column=3, padx=5, pady=5)

        self.output_text = tk.Text(root, height=15, width=70, state="disabled")
        self.output_text.grid(row=5, column=0, columnspan=4, padx=5, pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.check_queue)

    def load_settings(self):
        self.settings = {}
        config_file = "settings.json"
        if os.path.exists(config_file):
            try:
                with open(config_file, "r") as f:
                    self.settings = json.load(f)
            except (json.JSONDecodeError, IOError) as e:
                self.settings = {}
                print(f"Error loading settings: {e}")

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
            time.sleep(2)  # Longer delay for Arduino reset
            self.serial.flush()  # Clear initial noise

            # Hardcoded help output
            help_output = [
                "RPP_Robot_Demo vX.Y",
                "Commands:",
                "G00/G01 [X/Y/Z(mm)] [F(feedrate)]; - linear move",
                "G04 P[seconds]; - delay",
                "G28; - home all axes",
                "G90; - absolute mode",
                "G91; - relative mode",
                "G92 [X/Y/Z(mm)]; - change logical position",
                "J1/J2/J3 [D(distance)]; - jog θ1 (deg), d2 (mm), d3 (mm)",
                "M17; - enable motors",
                "M18; - disable motors",
                "M100; - this help message",
                "M114; - report position and feedrate",
                "C01; - run square pattern canned cycle",
                ">"
            ]
            self.log("Arduino Help:")
            for line in help_output:
                self.log(line)

            # Wait for prompt to ensure Arduino is ready
            start_time = time.time()
            while time.time() - start_time < 5:
                if self.serial.in_waiting > 0:
                    raw_data = self.serial.readline()
                    try:
                        response = raw_data.decode('utf-8').strip()
                        self.log(f"Arduino: {response}")
                        if '>' in response:
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
            self.log("Ready for commands or file sending")
        except serial.SerialException as e:
            self.log(f"Error opening serial port: {e}")
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.serial = None

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
        self.log("Stopped by user.")
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
            self.log("Serial connection closed")

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.config(text="Resume" if self.paused else "Pause")
        self.log("Paused" if self.paused else "Resumed")

    def send_manual_command(self):
        if not self.serial or not self.serial.is_open:
            self.log("Not connected. Please connect first.")
            return

        command = self.command_var.get().strip()
        if not command:
            self.log("No command entered")
            return

        try:
            self.log(f"Sending command: {command}")
            self.serial.write((command + '\n').encode('utf-8'))
            self.serial.flush()
            self.command_var.set("")
            start_time = time.time()
            prompt_seen = False
            while time.time() - start_time < 2:  # Longer timeout
                if self.serial.in_waiting > 0:
                    raw_data = self.serial.readline()
                    try:
                        response = raw_data.decode('utf-8').strip()
                        self.log(f"Received: {response}")
                        if '>' in response:
                            prompt_seen = True
                    except UnicodeDecodeError:
                        self.log(f"Decode error: {raw_data.hex()}")
                time.sleep(0.01)
            if not prompt_seen:
                self.log("No prompt received after command")
        except serial.SerialException as e:
            self.log(f"Error sending command: {e}")

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

    # Replace send_gcode_line in your GUI code
    def send_gcode_line(self, line, line_number):
        line = line.split(';', 1)[0].strip()
        if not line:
            return

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
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = GCodeSenderApp(root)
    root.mainloop()