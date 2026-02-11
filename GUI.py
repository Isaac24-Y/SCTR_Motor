import serial
import threading
import time
import tkinter as tk
from tkinter import ttk
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ---------------- CONFIG ----------------
SERIAL_PORT = "COM6"      # cambia esto
BAUDRATE = 115200
MAX_POINTS = 200

# ---------------- SERIAL ----------------
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

# ---------------- DATA BUFFERS ----------------
pv_data = deque(maxlen=MAX_POINTS)
op_data = deque(maxlen=MAX_POINTS)
err_data = deque(maxlen=MAX_POINTS)
sp_data = deque(maxlen=MAX_POINTS)

# ---------------- GUI ----------------
root = tk.Tk()
root.title("Motor Control GUI")

# ---------------- FIGURE ----------------
fig, ax = plt.subplots(3, 1, figsize=(6, 6))
fig.tight_layout(pad=3)

lines = {
    "pv": ax[0].plot([], [], label="PV")[0],
    "sp": ax[0].plot([], [], label="SP")[0],
    "op": ax[1].plot([], [], label="OP")[0],
    "err": ax[2].plot([], [], label="Error")[0],
}

ax[0].legend()
ax[1].legend()
ax[2].legend()

ax[0].set_ylabel("Velocidad")
ax[1].set_ylabel("PWM")
ax[2].set_ylabel("Error")
ax[2].set_xlabel("Tiempo")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(row=0, column=0, rowspan=10)

# ---------------- INPUTS ----------------
frame = ttk.Frame(root)
frame.grid(row=0, column=1, padx=10)

def send_sp():
    sp = sp_entry.get()
    ser.write(f"SP:{sp}\n".encode())

def send_pid():
    kc = kc_entry.get()
    ti = ti_entry.get()
    td = td_entry.get()
    ser.write(f"PID:{kc},{ti},{td}\n".encode())

ttk.Label(frame, text="Setpoint").grid(row=0, column=0)
sp_entry = ttk.Entry(frame)
sp_entry.grid(row=0, column=1)
ttk.Button(frame, text="Enviar", command=send_sp).grid(row=0, column=2)

ttk.Label(frame, text="Kc").grid(row=1, column=0)
kc_entry = ttk.Entry(frame)
kc_entry.grid(row=1, column=1)

ttk.Label(frame, text="Tau_i").grid(row=2, column=0)
ti_entry = ttk.Entry(frame)
ti_entry.grid(row=2, column=1)

ttk.Label(frame, text="Tau_d").grid(row=3, column=0)
td_entry = ttk.Entry(frame)
td_entry.grid(row=3, column=1)

ttk.Button(frame, text="Actualizar PID", command=send_pid)\
    .grid(row=4, column=0, columnspan=3)

# ---------------- SERIAL THREAD ----------------
def serial_thread():
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue

        try:
            data = dict(item.split(":") for item in line.split(","))
            pv_data.append(float(data["PV"]))
            op_data.append(float(data["OP"]))
            err_data.append(float(data["E"]))
            sp_data.append(float(data["SP"]))
        except:
            pass

threading.Thread(target=serial_thread, daemon=True).start()

# ---------------- UPDATE PLOT ----------------
def update_plot():
    x = range(len(pv_data))

    lines["pv"].set_data(x, pv_data)
    lines["sp"].set_data(x, sp_data)
    lines["op"].set_data(x, op_data)
    lines["err"].set_data(x, err_data)

    for a in ax:
        a.relim()
        a.autoscale_view()

    canvas.draw()
    root.after(100, update_plot)

update_plot()
root.mainloop()
