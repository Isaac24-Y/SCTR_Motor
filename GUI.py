import threading
import time
import queue
from collections import deque

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ---------------- CONFIG ----------------
DEFAULT_BAUDRATE = 115200
MAX_POINTS = 300
PLOT_UPDATE_MS = 100

# Campos esperados desde firmware
# SP,PV,OP,ERR,Tvel_us,JvelMax_us,Tpid_us,JpidMax_us,Lpid_us,Lpwm_us,Ltotal_us


class MotorGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Motor Control GUI - ESP32")

        self.ser = None
        self.serial_thread_running = False
        self.serial_thread_obj = None

        self.samples_queue = queue.Queue(maxsize=1000)

        self.pv_data = deque(maxlen=MAX_POINTS)
        self.op_data = deque(maxlen=MAX_POINTS)
        self.err_data = deque(maxlen=MAX_POINTS)
        self.sp_data = deque(maxlen=MAX_POINTS)

        self.current_metrics = {
            "Tvel_us": 0,
            "JvelMax_us": 0,
            "Tpid_us": 0,
            "JpidMax_us": 0,
            "Lpid_us": 0,
            "Lpwm_us": 0,
            "Ltotal_us": 0,
        }

        self._build_layout()
        self._refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(PLOT_UPDATE_MS, self._update_gui)

    # ---------------- GUI BUILD ----------------
    def _build_layout(self):
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        plot_frame = ttk.Frame(main)
        plot_frame.grid(row=0, column=0, rowspan=20, sticky="nsew")

        control_frame = ttk.Frame(main)
        control_frame.grid(row=0, column=1, sticky="n", padx=(12, 0))

        main.columnconfigure(0, weight=1)
        main.rowconfigure(0, weight=1)

        # Matplotlib
        self.fig, self.ax = plt.subplots(3, 1, figsize=(7.2, 6.2))
        self.fig.tight_layout(pad=2.2)

        self.lines = {
            "pv": self.ax[0].plot([], [], label="PV", linewidth=1.3)[0],
            "sp": self.ax[0].plot([], [], label="SP", linewidth=1.3)[0],
            "op": self.ax[1].plot([], [], label="OP", linewidth=1.3)[0],
            "err": self.ax[2].plot([], [], label="Error", linewidth=1.3)[0],
        }

        self.ax[0].set_ylabel("Velocidad")
        self.ax[1].set_ylabel("PWM")
        self.ax[2].set_ylabel("Error")
        self.ax[2].set_xlabel("Muestra")
        self.ax[0].legend(loc="upper right")
        self.ax[1].legend(loc="upper right")
        self.ax[2].legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Conexión serial
        ttk.Label(control_frame, text="Puerto:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=14, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew")

        ttk.Button(control_frame, text="Refrescar", command=self._refresh_ports).grid(row=0, column=2, padx=(4, 0))

        ttk.Label(control_frame, text="Baud:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        ttk.Entry(control_frame, textvariable=self.baud_var, width=16).grid(row=1, column=1, sticky="ew", pady=(6, 0))

        self.connect_btn = ttk.Button(control_frame, text="Conectar", command=self._toggle_connection)
        self.connect_btn.grid(row=1, column=2, padx=(4, 0), pady=(6, 0))

        self.status_var = tk.StringVar(value="Desconectado")
        ttk.Label(control_frame, textvariable=self.status_var, foreground="#0066cc").grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(6, 10)
        )

        # Setpoint
        ttk.Label(control_frame, text="Setpoint").grid(row=3, column=0, sticky="w")
        self.sp_entry = ttk.Entry(control_frame, width=16)
        self.sp_entry.grid(row=3, column=1, sticky="ew")
        ttk.Button(control_frame, text="Enviar", command=self._send_sp).grid(row=3, column=2, padx=(4, 0))

        # PID
        ttk.Label(control_frame, text="Kp").grid(row=4, column=0, sticky="w", pady=(6, 0))
        self.kp_entry = ttk.Entry(control_frame, width=16)
        self.kp_entry.grid(row=4, column=1, sticky="ew", pady=(6, 0))

        ttk.Label(control_frame, text="Ki").grid(row=5, column=0, sticky="w")
        self.ki_entry = ttk.Entry(control_frame, width=16)
        self.ki_entry.grid(row=5, column=1, sticky="ew")

        ttk.Label(control_frame, text="Kd").grid(row=6, column=0, sticky="w")
        self.kd_entry = ttk.Entry(control_frame, width=16)
        self.kd_entry.grid(row=6, column=1, sticky="ew")

        ttk.Button(control_frame, text="Actualizar PID", command=self._send_pid).grid(
            row=7, column=0, columnspan=3, sticky="ew", pady=(6, 10)
        )

        # Métricas
        ttk.Separator(control_frame, orient="horizontal").grid(row=8, column=0, columnspan=3, sticky="ew", pady=(4, 8))
        ttk.Label(control_frame, text="Métricas (us)", font=("TkDefaultFont", 9, "bold")).grid(
            row=9, column=0, columnspan=3, sticky="w"
        )

        self.metric_vars = {k: tk.StringVar(value=f"{k}: 0") for k in self.current_metrics.keys()}
        row = 10
        for key, var in self.metric_vars.items():
            ttk.Label(control_frame, textvariable=var).grid(row=row, column=0, columnspan=3, sticky="w")
            row += 1

        control_frame.columnconfigure(1, weight=1)

    # ---------------- SERIAL ----------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Puerto serial", "Selecciona un puerto serial.")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showwarning("Baudrate", "Baudrate inválido.")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            time.sleep(0.2)
        except Exception as exc:
            self.status_var.set(f"Error de conexión: {exc}")
            self.ser = None
            return

        self.serial_thread_running = True
        self.serial_thread_obj = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread_obj.start()

        self.connect_btn.config(text="Desconectar")
        self.status_var.set(f"Conectado a {port} @ {baud}")

    def _disconnect_serial(self):
        self.serial_thread_running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.config(text="Conectar")
        self.status_var.set("Desconectado")

    def _send_line(self, text: str):
        if not (self.ser and self.ser.is_open):
            self.status_var.set("No hay conexión serial")
            return
        try:
            self.ser.write((text + "\n").encode("utf-8"))
        except Exception as exc:
            self.status_var.set(f"Error TX: {exc}")

    def _send_sp(self):
        sp = self.sp_entry.get().strip()
        if not sp:
            return
        self._send_line(f"SP:{sp}")

    def _send_pid(self):
        kp = self.kp_entry.get().strip()
        ki = self.ki_entry.get().strip()
        kd = self.kd_entry.get().strip()
        if not (kp and ki and kd):
            return
        self._send_line(f"PID:{kp},{ki},{kd}")

    @staticmethod
    def _parse_telemetry(line: str):
        data = {}
        for item in line.split(","):
            if ":" not in item:
                continue
            key, value = item.split(":", 1)
            data[key.strip()] = value.strip()

        required = ["SP", "PV", "OP", "ERR"]
        if any(k not in data for k in required):
            return None

        parsed = {
            "SP": float(data["SP"]),
            "PV": float(data["PV"]),
            "OP": float(data["OP"]),
            "ERR": float(data["ERR"]),
        }

        for k in ["Tvel_us", "JvelMax_us", "Tpid_us", "JpidMax_us", "Lpid_us", "Lpwm_us", "Ltotal_us"]:
            parsed[k] = int(float(data.get(k, 0)))

        return parsed

    def _serial_reader(self):
        while self.serial_thread_running:
            if not (self.ser and self.ser.is_open):
                break
            try:
                raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue
                sample = self._parse_telemetry(raw)
                if sample is None:
                    continue
                try:
                    self.samples_queue.put_nowait(sample)
                except queue.Full:
                    # Descarta la más vieja para mantener la GUI en tiempo real
                    try:
                        _ = self.samples_queue.get_nowait()
                        self.samples_queue.put_nowait(sample)
                    except queue.Empty:
                        pass
            except Exception as exc:
                self.status_var.set(f"Error RX: {exc}")
                self.serial_thread_running = False

    # ---------------- UPDATE LOOP ----------------
    def _update_gui(self):
        updated = False
        for _ in range(200):
            try:
                sample = self.samples_queue.get_nowait()
            except queue.Empty:
                break

            self.sp_data.append(sample["SP"])
            self.pv_data.append(sample["PV"])
            self.op_data.append(sample["OP"])
            self.err_data.append(sample["ERR"])

            for k in self.current_metrics:
                self.current_metrics[k] = sample[k]

            updated = True

        if updated:
            x = range(len(self.pv_data))
            self.lines["pv"].set_data(x, self.pv_data)
            self.lines["sp"].set_data(x, self.sp_data)
            self.lines["op"].set_data(x, self.op_data)
            self.lines["err"].set_data(x, self.err_data)

            for axis in self.ax:
                axis.relim()
                axis.autoscale_view()

            for k, var in self.metric_vars.items():
                var.set(f"{k}: {self.current_metrics[k]}")

            self.canvas.draw_idle()

        self.root.after(PLOT_UPDATE_MS, self._update_gui)

    def _on_close(self):
        self._disconnect_serial()
        self.root.destroy()


def main():
    root = tk.Tk()
    _app = MotorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
