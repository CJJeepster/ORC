import matplotlib.pyplot as plt
import csv
import numpy as np
from scipy.signal import medfilt
from collections import defaultdict

# File path
filename = r"D:\ORCLOG.csv"  # Update as needed

# Data storage
logs = defaultdict(lambda: {"Acceleration [g]": [], "Pitch [°]": [], "Roll [°]": [], "Time [s]": []})
current_log = None
actuator_status = None
interval = 1.0  # Default interval in case it's missing

# Read the file
with open(filename, "r") as file:
    reader = csv.reader(file)
    for row in reader:
        if row:
            if row[0].startswith("Log #"):
                current_log = row[0]
            elif "Actuators" in row[0]:
                actuator_status = row[0]
            elif row[0].startswith("Interval:"):
                try:
                    interval = float(row[0].split(":")[1].strip())
                except ValueError:
                    interval = 1.0  # Fallback if parsing fails
            elif len(row) == 3 and current_log and actuator_status:
                try:
                    acc, p, r = map(float, row)
                    key = f"{current_log} - {actuator_status}"
                    logs[key]["Acceleration [g]"].append(acc)
                    logs[key]["Pitch [°]"].append(p)
                    logs[key]["Roll [°]"].append(r)
                    logs[key]["Time [s]"].append(len(logs[key]["Time [s]"]) * interval)
                except ValueError:
                    continue  # Skip invalid rows
            elif row[0].startswith("Log Paused"): #special mark when there is a break in the log
                try:
                    key = f"{current_log} - {actuator_status}"
                    logs[key]["Acceleration [g]"].append(300)
                    logs[key]["Pitch [°]"].append(3000)
                    logs[key]["Roll [°]"].append(3000)
                    logs[key]["Time [s]"].append(len(logs[key]["Time [s]"]) * interval)
                except ValueError:
                    continue  # Skip invalid rows


# Function to apply median filter
def filter_data(data):
    return medfilt(data, kernel_size=15)

# Function to compute stats
def data_stats(data):
    rms = np.sqrt(np.mean(np.square(data)))  # Compute RMS value
    min_val, max_val = np.min(data), np.max(data)  # Find min/max
    return rms, min_val, max_val


# Function to plot data and display statistics
def plot_data(logs, data_key, title, filename):
    fig, ax = plt.subplots(figsize=(38.4, 21.6), dpi=100)  # 4K resolution
    stats_texts = []
    for key, data in logs.items():
        filtered = filter_data(data[data_key]) #filter will help ignore new log marks
        rms, min_val, max_val = data_stats(filtered)
        ax.plot(data["Time [s]"], filtered, label=key, linestyle="-", marker="")
        stats_texts.append(f"{key}\nRMS: {rms:.5f}\nMin: {min_val:.5f}\nMax: {max_val:.5f}")

    # Set plot labels and grid
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(data_key)
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

    # Add statistics text to the right of the plot
    stats_text = "\n\n".join(stats_texts)
    plt.gcf().text(0.02, .3, stats_text, fontsize=14, verticalalignment='center',
                   bbox=dict(facecolor='white', alpha=0.5))

    # Save and close the figure
    plt.savefig(filename, dpi=100)  # Save as high-res image
    plt.close()

def stable_derivative_plot(logs, data_key, title, filename):
    fig, ax = plt.subplots(figsize=(38.4, 21.6), dpi=100)  # 4K resolution
    stats_texts = []
    for key, data in logs.items():
        unstable_indices = [i for i, x in enumerate(data[data_key]) if x == 300]#find where unstable derivatives can occur (log start and stop)
        filtered = filter_data(data[data_key])
        grad = np.gradient(filtered)
        for i in unstable_indices:#remove the unstable derivatives
            grad[i-1] = 0
            grad[i] = 0
            grad[i+1] = 0
        rms, min_val, max_val = data_stats(grad)
        ax.plot(data["Time [s]"], grad, label=key, linestyle="-", marker="")
        stats_texts.append(f"{key}\nRMS: {rms:.5f}\nMin: {min_val:.5f}\nMax: {max_val:.5f}")

    # Set plot labels and grid
    ax.set_xlabel("Time [s]")
    if(data_key == "Acceleration [g]"):
        ax.set_ylabel("Jerk [g/s]")
    else:
        ax.set_ylabel("Angular velocity [rad/s]")
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

    # Add statistics text to the right of the plot
    stats_text = "\n\n".join(stats_texts)
    plt.gcf().text(0.02, .3, stats_text, fontsize=14, verticalalignment='center',
                   bbox=dict(facecolor='white', alpha=0.5))

    # Save and close the figure
    plt.savefig(filename, dpi=100)  # Save as high-res image
    plt.close()


# Generate separate plots
stable_derivative_plot(logs, "Acceleration [g]", "Median Filtered (ks=15) Computed Jerk", "jerk_4k_camber.png")
plot_data(logs, "Acceleration [g]", "Median Filtered (ks=15) Acceleration Data", "acceleration_4k_camber.png")
plot_data(logs, "Pitch [°]", "Median Filtered (ks=15) Pitch Data", "pitch_4k_camber.png")
plot_data(logs, "Roll [°]", "Median Filtered (ks=15) Roll Data", "roll_4k_camber.png")

print("Plots saved as 4K images with statistics.")
