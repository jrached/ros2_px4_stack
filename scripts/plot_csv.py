#!/bin/usr/env python3 

from matplotlib import pyplot as plt 
import pandas as pd 
import numpy as np 


#NOTE: arr1 is the measured attitude, arr2 is the response

def plot_from_csv(path_to_csv):

    arr = pd.read_csv(path_to_csv)
    arr = np.array(arr)
    plt.plot(arr[:, 0], arr[:, 1])
    plt.show()

def from_csv_to_numpy(path):
    return np.array(pd.read_csv(path))

def sync_timestamps(arr1, arr2): 
    #Assumes arr1 is longer than arr2 

    # Round timestamps
    arr1[:, 0] = np.round(arr1[:, 0] / 1e18, decimals=9) * 1e9 
    arr2[:, 0] = np.round(arr2[:, 0] / 1e18, decimals=9) * 1e9

    # Get times after latest bag started
    t_start = arr2[0, 0]
    new_arr1 = arr1[arr1[:, 0] >= t_start,:]

    # Shift time to start at zero
    new_arr1[:, 0] = new_arr1[:, 0] - t_start
    new_arr2 = arr2
    new_arr2[:, 0] = new_arr2[:, 0] - t_start

    # Filter out repeated values
    unique_times1, first_indices1 = np.unique(new_arr1[:, 0], return_index=True)
    filtered_arr1 = new_arr1[first_indices1, :]

    unique_times2, first_indices2 = np.unique(new_arr2[:, 0], return_index=True)
    filtered_arr2 = new_arr2[first_indices2, :]

    return filtered_arr1, filtered_arr2 

def plot_roll_response(arr1, arr2):
    plt.plot(arr1[:, 0], arr1[:, 1], arr2[:, 1])
    plt.title("Roll Response")
    plt.legend(["Response", "Setpoint"])
    plt.xlabel("time")
    plt.ylabel("roll")
    plt.show()


if __name__ == "__main__":
    path1 = "/home/juanrached/mavros_ws/src/uwb_drone_experiments/data/pid_response/att_measured.csv"
    path2 = "/home/juanrached/mavros_ws/src/uwb_drone_experiments/data/pid_response/att_setpoints.csv"

    arr1 = from_csv_to_numpy(path1)
    arr2 = from_csv_to_numpy(path2)
    
    new_arr1, new_arr2 = sync_timestamps(arr1, arr2)

    plot_roll_response(new_arr1, new_arr2)

    