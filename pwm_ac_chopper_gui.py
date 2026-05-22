import csv
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports


BAUDRATE = 115200
CSV_HEADER = [
    "pc_timestamp",
    "stm32_ms",
    "adc_raw",
    "adc_voltage_v",
    "sensor_voltage_v",
    "pressure_bar",
    "duty_percent",
    "frequency_hz",
    "estimated_output_rms_v",
]


class SerialWorker:
    def __init__(self, line_queue):
        self.line_queue = line_queue
        self.serial_port = None
        self.thread = None
        self.running = False

    def connect(self, port):
        self.disconnect()
        self.serial_port = serial.Serial(port, BAUDRATE, timeout=0.2)
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.serial_port = None

    def send(self, command):
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not connected")
        self.serial_port.write((command.strip() + "\n").encode("ascii"))

    def _read_loop(self):
        while self.running and self.serial_port is not None:
            try:
                raw = self.serial_port.readline()
            except serial.SerialException as exc:
                self.line_queue.put(("ERROR", str(exc)))
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                self.line_queue.put(("LINE", line))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PWM AC Chopper Controller")
        self.geometry("980x620")
        self.minsize(900, 560)

        self.line_queue = queue.Queue()
        self.serial_worker = SerialWorker(self.line_queue)
        self.csv_file = None
        self.csv_writer = None

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.duty_var = tk.DoubleVar(value=50.0)
        self.freq_var = tk.IntVar(value=5000)
        self.csv_path_var = tk.StringVar(value=str(Path.cwd() / "pwm_ac_chopper_log.csv"))

        self.telemetry_vars = {
            "adc_raw": tk.StringVar(value="-"),
            "adc_voltage": tk.StringVar(value="-"),
            "sensor_voltage": tk.StringVar(value="-"),
            "pressure": tk.StringVar(value="-"),
            "duty": tk.StringVar(value="-"),
            "frequency": tk.StringVar(value="-"),
            "rms": tk.StringVar(value="-"),
        }

        self._build_ui()
        self.refresh_ports()
        self.after(50, self.process_queue)

    def _build_ui(self):
        outer = ttk.Frame(self, padding=12)
        outer.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(outer, text="Connection", padding=10)
        connection.pack(fill="x")

        ttk.Label(connection, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=24)
        self.port_combo.grid(row=0, column=1, padx=8, sticky="w")
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=4)
        ttk.Label(connection, textvariable=self.status_var).grid(row=0, column=4, padx=12, sticky="w")
        connection.columnconfigure(5, weight=1)

        controls = ttk.LabelFrame(outer, text="PWM Control", padding=10)
        controls.pack(fill="x", pady=10)

        ttk.Label(controls, text="Duty Cycle (%)").grid(row=0, column=0, sticky="w")
        duty_scale = ttk.Scale(
            controls,
            from_=0.0,
            to=95.0,
            variable=self.duty_var,
            command=lambda _: self._sync_duty_label(),
        )
        duty_scale.grid(row=0, column=1, padx=8, sticky="ew")
        self.duty_label = ttk.Label(controls, width=8, text="50.0")
        self.duty_label.grid(row=0, column=2, sticky="w")

        ttk.Label(controls, text="Frequency (Hz)").grid(row=1, column=0, sticky="w", pady=8)
        ttk.Spinbox(controls, from_=100, to=50000, increment=100, textvariable=self.freq_var, width=12).grid(
            row=1, column=1, padx=8, sticky="w"
        )

        ttk.Button(controls, text="Apply Duty", command=self.apply_duty).grid(row=0, column=3, padx=4)
        ttk.Button(controls, text="Apply Frequency", command=self.apply_frequency).grid(row=1, column=3, padx=4)
        ttk.Button(controls, text="Apply Both", command=self.apply_both).grid(row=0, column=4, padx=4)
        ttk.Button(controls, text="Stop PWM", command=self.stop_pwm).grid(row=1, column=4, padx=4)
        controls.columnconfigure(1, weight=1)

        telemetry = ttk.LabelFrame(outer, text="Telemetry", padding=10)
        telemetry.pack(fill="x")

        labels = [
            ("ADC Raw", "adc_raw"),
            ("ADC Voltage", "adc_voltage"),
            ("Sensor Voltage", "sensor_voltage"),
            ("Pressure", "pressure"),
            ("Duty", "duty"),
            ("Frequency", "frequency"),
            ("Est. Output RMS", "rms"),
        ]
        for idx, (label, key) in enumerate(labels):
            row = idx // 4
            col = (idx % 4) * 2
            ttk.Label(telemetry, text=label).grid(row=row, column=col, sticky="w", padx=(0, 4), pady=4)
            ttk.Label(telemetry, textvariable=self.telemetry_vars[key], width=18).grid(
                row=row, column=col + 1, sticky="w", padx=(0, 12), pady=4
            )

        logging = ttk.LabelFrame(outer, text="CSV Logging", padding=10)
        logging.pack(fill="x", pady=10)
        ttk.Entry(logging, textvariable=self.csv_path_var).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Button(logging, text="Browse", command=self.choose_csv).grid(row=0, column=1, padx=4)
        self.log_button = ttk.Button(logging, text="Start Logging", command=self.toggle_logging)
        self.log_button.grid(row=0, column=2, padx=4)
        logging.columnconfigure(0, weight=1)

        log_frame = ttk.LabelFrame(outer, text="Serial Log", padding=10)
        log_frame.pack(fill="both", expand=True)
        self.log_text = tk.Text(log_frame, height=12, wrap="none")
        self.log_text.pack(fill="both", expand=True)

    def _sync_duty_label(self):
        self.duty_label.configure(text=f"{self.duty_var.get():.1f}")

    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.serial_worker.serial_port is not None and self.serial_worker.serial_port.is_open:
            self.serial_worker.disconnect()
            self.connect_button.configure(text="Connect")
            self.status_var.set("Disconnected")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("Port kosong", "Pilih COM port STM32 terlebih dahulu.")
            return
        try:
            self.serial_worker.connect(port)
        except serial.SerialException as exc:
            messagebox.showerror("Serial error", str(exc))
            return
        self.connect_button.configure(text="Disconnect")
        self.status_var.set(f"Connected to {port}")

    def apply_duty(self):
        self._send(f"SET,DUTY,{self.duty_var.get():.2f}")

    def apply_frequency(self):
        self._send(f"SET,FREQ,{int(self.freq_var.get())}")

    def apply_both(self):
        self._send(f"SET,BOTH,{self.duty_var.get():.2f},{int(self.freq_var.get())}")

    def stop_pwm(self):
        self._send("STOP")

    def _send(self, command):
        try:
            self.serial_worker.send(command)
        except RuntimeError as exc:
            messagebox.showwarning("Belum connect", str(exc))
            return
        self.append_log(f"> {command}")

    def choose_csv(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if path:
            self.csv_path_var.set(path)

    def toggle_logging(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.log_button.configure(text="Start Logging")
            self.append_log("CSV logging stopped")
            return

        path = Path(self.csv_path_var.get())
        file_exists = path.exists() and path.stat().st_size > 0
        self.csv_file = path.open("a", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if not file_exists:
            self.csv_writer.writerow(CSV_HEADER)
            self.csv_file.flush()
        self.log_button.configure(text="Stop Logging")
        self.append_log(f"CSV logging started: {path}")

    def process_queue(self):
        while True:
            try:
                kind, payload = self.line_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "ERROR":
                self.append_log(f"! {payload}")
                self.status_var.set("Serial error")
            else:
                self.handle_line(payload)
        self.after(50, self.process_queue)

    def handle_line(self, line):
        self.append_log(line)
        parts = line.split(",")
        if len(parts) != 9 or parts[0] != "DATA":
            return
        try:
            has_float_payload = any("." in value for value in parts[3:9])
            if has_float_payload:
                adc_voltage = float(parts[3])
                sensor_voltage = float(parts[4])
                pressure_bar = float(parts[5])
                duty_percent = float(parts[6])
                estimated_rms = float(parts[8])
            else:
                adc_voltage = int(parts[3]) / 1000.0
                sensor_voltage = int(parts[4]) / 1000.0
                pressure_bar = int(parts[5]) / 1000.0
                duty_percent = int(parts[6]) / 100.0
                estimated_rms = int(parts[8]) / 100.0

            row = {
                "pc_timestamp": datetime.now().isoformat(timespec="milliseconds"),
                "stm32_ms": int(parts[1]),
                "adc_raw": int(parts[2]),
                "adc_voltage_v": adc_voltage,
                "sensor_voltage_v": sensor_voltage,
                "pressure_bar": pressure_bar,
                "duty_percent": duty_percent,
                "frequency_hz": int(parts[7]),
                "estimated_output_rms_v": estimated_rms,
            }
        except ValueError:
            self.status_var.set("DATA parse error")
            return

        self.telemetry_vars["adc_raw"].set(str(row["adc_raw"]))
        self.telemetry_vars["adc_voltage"].set(f'{row["adc_voltage_v"]:.4f} V')
        self.telemetry_vars["sensor_voltage"].set(f'{row["sensor_voltage_v"]:.4f} V')
        self.telemetry_vars["pressure"].set(f'{row["pressure_bar"]:.3f} bar')
        self.telemetry_vars["duty"].set(f'{row["duty_percent"]:.2f} %')
        self.telemetry_vars["frequency"].set(f'{row["frequency_hz"]} Hz')
        self.telemetry_vars["rms"].set(f'{row["estimated_output_rms_v"]:.2f} Vrms')

        if self.csv_writer is not None:
            try:
                self.csv_writer.writerow([row[key] for key in CSV_HEADER])
                self.csv_file.flush()
            except OSError as exc:
                self.append_log(f"! CSV write error: {exc}")
                self.status_var.set("CSV write error")

    def append_log(self, line):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {line}\n")
        self.log_text.see("end")

    def destroy(self):
        self.serial_worker.disconnect()
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy()


if __name__ == "__main__":
    App().mainloop()
