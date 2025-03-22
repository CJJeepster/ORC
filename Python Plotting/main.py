import matplotlib.pyplot as plt
import csv
import numpy as np
from scipy.signal import medfilt
from scipy.stats import ttest_ind
from collections import defaultdict
#default plot text size
plt.rcParams.update({'font.size': 50})  # Sets default font size to 14
# File path
filename = r"D:\ORCLOG.csv"  # Update as needed

# Data storage
logs = defaultdict(lambda: {"Acceleration [g]": [], "Pitch [°]": [], "Roll [°]": [], "Time [s]": []})
current_log = None
actuator_status = None
interval = 1.0  # Default interval in case it's missing
sample_number = 0

# Read the file
with open(filename, "r") as file:
    reader = csv.reader(file)
    #iterate through each row in file
    for row in reader:
        #only read rows with content
        if row:
            #row starts with log number, save that
            if row[0].startswith("Log #"):
                current_log = row[0]
                sample_number = 0
            #the following row should have the actuator status (enabled/disabled)
            elif "Actuators" in row[0]:
                actuator_status = row[0]
            #the row after that should have the interval between samples (for conversion to actual seconds)
            elif row[0].startswith("Interval:"):
                try:
                    interval = float(row[0].split(":")[1].strip())
                    key = f"{actuator_status}"
                    logs[key]["Acceleration [g]"].append([])
                    logs[key]["Pitch [°]"].append([])
                    logs[key]["Roll [°]"].append([])
                    logs[key]["Time [s]"].append([])
                except ValueError:
                    interval = 1.0  # Fallback if parsing fails
            #rows of data have the acceleration, pitch, and roll as floats in that order.
            elif len(row) == 3 and current_log and actuator_status:
                try:
                    acc, p, r = map(float, row)
                    key = f"{actuator_status}"
                    logs[key]["Acceleration [g]"][sample_number].append(acc)
                    logs[key]["Pitch [°]"][sample_number].append(p)
                    logs[key]["Roll [°]"][sample_number].append(r)
                    logs[key]["Time [s]"][sample_number].append(len(logs[key]["Time [s]"][sample_number]) * interval)
                except ValueError:
                    continue  # Skip invalid rows
            #each time a row starts with "Log Paused" that means a new run has started. These are broken apart to allow us to preform a t test on the RMS, max, and min
            elif row[0].startswith("Log Paused"):
                sample_number += 1
                key = f"{actuator_status}"
                logs[key]["Acceleration [g]"].append([])
                logs[key]["Pitch [°]"].append([])
                logs[key]["Roll [°]"].append([])
                logs[key]["Time [s]"].append([])
            #if anything else, the line can be ignored


# Function to apply median filter
def filter_data(data):
    return medfilt(data, kernel_size=15)

# Function to compute stats
def data_stats(data):
    rms = np.sqrt(np.mean(np.square(data)))  # Compute RMS value
    min_val, max_val = np.min(data), np.max(data)  # Find min/max
    return rms, min_val, max_val


# Function to plot data and display statistics
def plot_data(logs, data_key, title, filename,stats=False,derivative=False):
    # default plot text size
    if stats:
        plt.rcParams.update({'font.size': 36})  # Sets default font size to 14
    else:
        plt.rcParams.update({'font.size': 50})
    fig, ax = plt.subplots(figsize=(38.4, 21.6), dpi=100)  # 4K resolution
    stats_texts = []
    rms_populations = []
    min_populations = []
    max_populations = []
    for key, data in logs.items():
        rms_vals = []
        min_vals = []
        max_vals = []
        first_3_count = 0
        time_index = 0
        for i in data[data_key]:
            filtered_data = filter_data(i)
            if derivative:
                filtered_data = np.gradient(filtered_data)
            rms, min_val, max_val = data_stats(filtered_data)
            rms_vals.append(rms)
            min_vals.append(min_val)
            max_vals.append(max_val)
            if(first_3_count == 10): #plot only the 11th sample
                ax.plot(data["Time [s]"][time_index], filtered_data, label=key, linestyle = "-", linewidth = 5.0)
            first_3_count += 1
            time_index += 1
        rms_populations.append(rms_vals)
        min_populations.append(min_vals)
        max_populations.append(max_vals)
        stats_texts.append(f"{key}\nAverage RMS: {np.mean(rms_vals):.5f}\nAverage Min: {np.mean(min_vals):.5f}\nAverage Max: {np.mean(max_vals):.5f}")
    rms_t_stat, rms_p_value = ttest_ind(rms_populations[0], rms_populations[1], alternative="less", equal_var=False)
    min_t_stat, min_p_value = ttest_ind(min_populations[0], min_populations[1], alternative="greater", equal_var=False)
    max_t_stat, max_p_value = ttest_ind(max_populations[0], max_populations[1], alternative="less", equal_var=False)
    stats_texts.append(f"One-sided T tests:")
    stats_texts.append(f"RMS T statistic: {rms_t_stat:.5f}\nRMS P value: {rms_p_value:.5e}")
    stats_texts.append(f"Min T statistic: {min_t_stat:.5f}\nMin P value: {min_p_value:.5e}")
    stats_texts.append(f"Max T statistic: {max_t_stat:.5f}\nMax P value: {max_p_value:.5e}")

    # Set plot labels and grid
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(data_key)
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

    # Add statistics text to the right of the plot
    stats_text = "\n\n".join(stats_texts)
    if stats:
        plt.gcf().text(.88, .19, stats_text, fontsize=24, verticalalignment='center',
                  bbox=dict(facecolor='white', alpha=0.9))

    # Save and close the figure
    plt.savefig(filename, dpi=100)  # Save as high-res image
    plt.close()

# Generate separate plots
plot_data(logs, "Acceleration [g]", "Single Sample - Median Filtered (ks=15) Acceleration Data", "acceleration_corner.png")
plot_data(logs, "Pitch [°]", "Single Sample - Median Filtered (ks=15) Pitch Data", "pitch_corner.png")
plot_data(logs, "Roll [°]", "Single Sample - Median Filtered (ks=15) Roll Data", "roll_corner.png")
plot_data(logs, "Acceleration [g]", "Single Sample - Median Filtered (ks=15) Computed Jerk", "jerk_corner.png", derivative=True)
plot_data(logs, "Acceleration [g]", "Single Sample - Median Filtered (ks=15) Acceleration Data", "acceleration_corner_stats.png",stats=True)
plot_data(logs, "Pitch [°]", "Single Sample - Median Filtered (ks=15) Pitch Data", "pitch_corner_stats.png",stats=True)
plot_data(logs, "Roll [°]", "Single Sample - Median Filtered (ks=15) Roll Data", "roll_corner_stats.png",stats=True)
plot_data(logs, "Acceleration [g]", "Single Sample - Median Filtered (ks=15) Computed Jerk", "jerk_corner_stats.png", derivative=True, stats=True)
print("Plots saved as 4K images with statistics.")
