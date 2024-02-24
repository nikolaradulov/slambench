import os
import numpy as np
import matplotlib.pyplot as plt
import sys
import re
import math


def parse_log_file(log_file_path):
    with open(log_file_path, 'r') as file:
        lines = file.readlines()
    found_table= False
    mean_ate_values = []
    for line in lines:
        if line.startswith('Frame Number'):
            found_table=True
            continue
        if(found_table):
            values = line.split()
            ate_rmse = float(values[2])
            if not math.isnan(ate_rmse):
                mean_ate_values.append(ate_rmse)
    return np.array(mean_ate_values)

def plot_mean_ate_evolution(dataset_folder, filter_type, dataset_name):
    filter_folder = os.path.join(dataset_folder, filter_type)
    frame_folders = [f for f in os.listdir(filter_folder) if os.path.isdir(os.path.join(filter_folder, f))]
    
    for frame_folder in frame_folders:
        frame_path = os.path.join(filter_folder, frame_folder)
        exp_folders = [f for f in os.listdir(frame_path) if os.path.isdir(os.path.join(frame_path, f))]
        exp_numbers = set()
        filter_values = set()

        # Regular expression pattern to extract numbers
        exp_pattern = r'exp(\d+)_frames_val'
        value_pattern = r'(-?\d+\.\d+)'

        # Extracting values using regular expressions
        for string in exp_folders:
            exp_match = re.search(exp_pattern, string)
            value_match = re.search(value_pattern, string)
            
            if exp_match and value_match:
                exp_numbers.add(int(exp_match.group(1)))
                filter_values.add(float(value_match.group(1)))
        exp_numbers = sorted(exp_numbers)
        filter_values = sorted(filter_values)
        ate_per_filer = []
        for filter_val in filter_values:
            mean_ate_values_all_runs = []
            for run in exp_numbers:
                exp_folder = f"exp{run}_frames_val_{filter_val}"
                # print(exp_folder)
                exp_path = os.path.join(frame_path, exp_folder)
                log_file_path = os.path.join(exp_path, 'log_file.txt')
                mean_ate_values = parse_log_file(log_file_path)
                # print(mean_ate_values)
                # exit(1)
                mean_ate_values_all_runs.append(np.mean(mean_ate_values))
                # print(mean_ate_values_all_runs)
            ate_per_filer.append(np.mean(mean_ate_values_all_runs))
            # print(mean)
        mean_ate_values_all_runs = np.array(mean_ate_values_all_runs)
        mean_ate_values_avg = np.mean(mean_ate_values_all_runs, axis=0)
        if filter_type=="contrast":
            filter_values = np.log10(filter_values)
        plt.plot(filter_values, ate_per_filer, label=frame_folder)
    
    plt.title(f'{dataset_name}: Mean ATE Evolution for {filter_type}')
    plt.xlabel('Filter Value')
    plt.ylabel('Mean ATE')
    plt.legend()
    plt.show()

def plot_dataset_mean_ate_evolution(slam_name_folder):
    dataset_folders = [f for f in os.listdir(slam_name_folder) if os.path.isdir(os.path.join(slam_name_folder, f))]
    
    for dataset_folder in dataset_folders:
        dataset_path = os.path.join(slam_name_folder, dataset_folder)
        plot_mean_ate_evolution(dataset_path, 'blur', dataset_folder)
        plot_mean_ate_evolution(dataset_path, 'contrast', dataset_folder)
        plot_mean_ate_evolution(dataset_path, 'brightness', dataset_folder)

# Main function
def main():
    if len(sys.argv) > 1 :
        slam_folder = sys.argv[1]
    else:
        exit(1)
    plot_dataset_mean_ate_evolution(slam_folder)


if __name__ == "__main__":
    main()
