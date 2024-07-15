import numpy as np
#import matplotlib.pyplot as plt
from scipy.signal import medfilt
import json

#Version 0.1, 2023-07-13: initial version
#version 0.2, 2023-07-17: add function to delete bad points

FNAME_IN = "cam_calibration_original.json"
FNAME_OUT = "cam_calibration.json"

#in meters
TABLE_MIN_DISTANCE = 1
TABLE_MAX_DISTANCE = 80

#exclude all samples with y-offset near this
EXCLUDE_YOFFSET = 0.0635

MEDFILT_KERNEL_LENGTH = 3

#for debugging
ENABLE_PLOTS = False

with open(FNAME_IN) as f:
    config_dict = json.load(f)

distance = np.array(config_dict["Parallax"]["Distance"])
x_offset = np.array(config_dict["Parallax"]["X"])
y_offset = np.array(config_dict["Parallax"]["Y"])

stddev = np.std(x_offset)
iqr = np.diff(np.percentile(x_offset, (25, 75)))
bad = np.zeros_like(distance, dtype=np.bool8)
if stddev / iqr > 3:
    x_offset_zscore = (x_offset - np.mean(x_offset)) / np.std(x_offset)
    bad |= (np.abs(x_offset_zscore) > 2)
stddev = np.std(y_offset)
iqr = np.diff(np.percentile(y_offset, (25, 75)))
if stddev / iqr > 3:
    y_offset_zscore = (y_offset - np.mean(y_offset)) / stddev
    bad |= (np.abs(y_offset_zscore) > 2)

bad |= np.abs(y_offset - EXCLUDE_YOFFSET) < 0.01

distance_outliers_removed = distance[~bad]
x_offset_outliers_removed = x_offset[~bad]
y_offset_outliers_removed = y_offset[~bad]

distance_filtered = medfilt(distance_outliers_removed,
                            MEDFILT_KERNEL_LENGTH)[MEDFILT_KERNEL_LENGTH//2:-MEDFILT_KERNEL_LENGTH//2]
x_offset_filtered = medfilt(x_offset_outliers_removed,
                            MEDFILT_KERNEL_LENGTH)[MEDFILT_KERNEL_LENGTH//2:-MEDFILT_KERNEL_LENGTH//2]
y_offset_filtered = medfilt(y_offset_outliers_removed,
                            MEDFILT_KERNEL_LENGTH)[MEDFILT_KERNEL_LENGTH//2:-MEDFILT_KERNEL_LENGTH//2]

def fit_offset(distance, offset):
    slope, intercept = np.polyfit(1/distance, offset, deg=1)
    def fit_func(distance_to_calculate):
        calculated_offset = intercept + slope / distance_to_calculate
        return calculated_offset
    return fit_func

distance_full = 1 / np.linspace(1/TABLE_MIN_DISTANCE, 1/TABLE_MAX_DISTANCE, num=200)
y_interp = fit_offset(distance_filtered, y_offset_filtered)
x_interp = fit_offset(distance_filtered, x_offset_filtered)
y_offset_full = y_interp(distance_full)
x_offset_full = x_interp(distance_full)


"""def plot_inverse(distance, offset, title):
    plt.title(title)
    plt.xlabel("Inverse distance (1/m)")
    plt.ylabel("Centroid offset (normalized pixels)")
    slope, intercept = np.polyfit(1/distance, offset, deg=1)
    plt.plot(1/distance, offset, label="Filtered data")
    plt.plot(1/distance, intercept + slope / distance, label="Linear fit")
    plt.legend()
    plt.show()

if ENABLE_PLOTS:
    plot_inverse(distance_filtered, x_offset_filtered, "X-offset")
    plot_inverse(distance_filtered, y_offset_filtered, "Y-offset")

def plot_data(distance, offset, distance_filtered, offset_filtered, distance_full, offset_full, title):
    plt.title(title)
    plt.xlabel("Distance (m)")
    plt.ylabel("Centroid offset (normalized image coordinates)")
    plt.plot(distance, offset, label="Raw data")
    plt.plot(distance_filtered, offset_filtered, label="Filtered data")
    plt.plot(distance_full, offset_full, label="Fit and extrapolated data")
    plt.legend()
    plt.show()

if ENABLE_PLOTS:
    plot_data(distance, x_offset, distance_filtered,
              x_offset_filtered, distance_full, x_offset_full, "X-offset")
    plot_data(distance, y_offset, distance_filtered,
              y_offset_filtered, distance_full, y_offset_full, "Y-offset")
"""
config_dict["Parallax"]["Distance"] = list(distance_full)
config_dict["Parallax"]["X"] = list(x_offset_full)
config_dict["Parallax"]["Y"] = list(y_offset_full)

with open(FNAME_OUT, mode='w') as f:
    json.dump(config_dict, f, indent=1)