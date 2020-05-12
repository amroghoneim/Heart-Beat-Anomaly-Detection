import serial
import time
import csv
import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial(port='COM3', baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, timeout=2)
#ser = serial.Serial('COM3')
ser.flushInput()
plot_window = 200
y_var = np.array(np.zeros([plot_window]))

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(y_var)
# try:
#     ser.isOpen()
#     print("serial is opne!!")
# except:
#     print("error")

if ser.isOpen():
    try:
        while 1:
            ser_bytes = ser.readline()
            decoded_bytes = float(ser_bytes[0:len(ser_bytes) - 2].decode("utf-8"))
            print(decoded_bytes)
            with open("test_data.csv", "a") as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow([time.time(), decoded_bytes])
                y_var = np.append(y_var, decoded_bytes)
                y_var = y_var[1:plot_window + 1]
                line.set_ydata(y_var)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()
    except Exception:
        print("error")
else:
    print("can't connect to serial")
