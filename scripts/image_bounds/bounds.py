import os
import numpy as np
import matplotlib.pyplot as plt
import sys
import re
import math
import json
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
fail_points={
    "icl-nuim": 1,
    "kitty" : 100
}

no_frames={
    "kitty": {
        "test": 1106,
        "lsd": 1106,
        "open_vins": -1,
        "orbslam3":1106,
        "orbslam2":1106
    },
    "eurocMAV":{
        "test": 3682,
        "lsd": 3682
    },
    "tum":{
        "test": 1362,
        "lsd": 1362, 
        "orbslam3": 1359,
        "orbslam2": 1359

    },
    "icl-nuim":{
        "lsd": 967,
        "test": 967,
        "open_vins": -1,
        "orbslam3": 882,
        "orbslam2": 882
    }
}

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


def read_modified_frame_ids(json_file):
    modified_frame_ids = []
    with open(json_file, 'r') as file:
        data = json.load(file)
        frames = data.get('frames', [])
        for frame in frames:
            modified_frame_ids.append(frame.get('id'))
    return modified_frame_ids

def read_image_metrics(file_path):
    image_metrics = []
    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split()
            frame_id = int(values[0])
            sharpness = float(values[1])
            brightness = float(values[2])
            contrast = float(values[3])
            image_metrics.append({'frame_id': frame_id, 'blur': sharpness, 'brightness': brightness, 'contrast': contrast})
    return image_metrics

def compute_mean_metric(quality_metrics, modified_ids, filter_type):
    # Filter quality metrics for perturbed frames with the specified filter type
    perturbed_metrics = [metric[filter_type] for metric in quality_metrics if metric['frame_id'] in modified_ids]
    # print(perturbed_metrics)
    # Compute the mean metric for the filter type
    mean_metric = sum(perturbed_metrics) / len(perturbed_metrics) if perturbed_metrics else None
    
    return mean_metric


def get_mean_ate_evolution(dataset_folder, filter_type, dataset_name):

    global slam_alg

    filter_folder = os.path.join(dataset_folder, filter_type)
    frame_folders = [f for f in os.listdir(filter_folder) if os.path.isdir(os.path.join(filter_folder, f))]
    total_filters=[]
    total_values=[]
    total_ates=[]
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
        ate_per_filer2 = []
        mean_values2 = []
        mean_values =[]
        partial_failure = {'x':[], 'y':[]}
        failure_points = {'x':[], 'y':[]}
        for filter_val in filter_values:
            fail_flag=5
            mean_ate_values_all_runs = []
            mean_values_all_runs = []
            for run in exp_numbers:
                exp_folder = f"exp{run}_frames_val_{filter_val}"
                # print(exp_folder)
                exp_path = os.path.join(frame_path, exp_folder)
                log_file_path = os.path.join(exp_path, 'log_file.txt')
                modified_ids = read_modified_frame_ids(os.path.join(exp_path, "conf.json"))
                quality_metrics = read_image_metrics(os.path.join(exp_path, "image_metrics.txt"))
                # print(quality_metrics[modified_ids[4]])
                mean_ate_values = parse_log_file(log_file_path)
                quality_metric = compute_mean_metric(quality_metrics, modified_ids, filter_type)
                # print(quality_metric)
                mean_values_all_runs.append(quality_metric)
                # just in case give some liniency
                if(len(mean_ate_values)<no_frames[dataset_name][slam_alg]-20):
                    mean_ate_values_all_runs.append(np.mean(fail_points[dataset_name]))
                    fail_flag-=1
                else:
                    mean_ate_values_all_runs.append(np.mean(mean_ate_values))
                # print(mean_values_all_runs)
            mean_values.append(np.mean(mean_values_all_runs))
            ate_per_filer.append(np.mean(mean_ate_values_all_runs))
            if(fail_flag<5 and fail_flag>0):
                partial_failure['x'].append(np.mean(mean_values_all_runs))
                partial_failure['y'].append(np.mean(mean_ate_values_all_runs))
            elif(fail_flag==0):
                failure_points['x'].append(np.mean(mean_values_all_runs))
                failure_points['y'].append(np.mean(mean_ate_values_all_runs))
        total_filters.extend(filter_values)
        total_values.extend(mean_values)
        total_ates.extend(ate_per_filer)       
    return np.array(total_filters), np.array(total_values), np.array(total_ates)
    
    

def plot_filter_data(filter_type,slam_name_folder):
    dataset_folders = [f for f in os.listdir(slam_name_folder) if os.path.isdir(os.path.join(slam_name_folder, f))]
    aggregated_filters = []
    aggregated_values = []
    aggregated_ates=[]
    for dataset_folder in dataset_folders:
        print(f"----> Retrieving from {dataset_folder}")
        dataset_path = os.path.join(slam_name_folder, dataset_folder)
        # get filter data for specific filter and dataset
        filters, mean_values, ates = get_mean_ate_evolution(dataset_path, filter_type, dataset_folder)
        ates = ates/(fail_points[dataset_folder]/2)
        aggregated_filters.extend(filters)
        aggregated_values.extend(mean_values)
        aggregated_ates.extend(ates)
    fig = plt.figure()
    ax = fig.add_subplot( projection='3d')
    # xi = np.linspace(min(aggregated_values), max(aggregated_values), 100)
    # yi = np.linspace(min(aggregated_filters), max(aggregated_filters), 100)
    # xi, yi = np.meshgrid(xi, yi)

    # Perform Delaunay triangulation and interpolate values
    # zi = griddata((aggregated_values, aggregated_filters), aggregated_ates, (xi, yi), method='linear')
    # fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    # Plot the data points
    # ax.plot_surface(xi, yi, zi, cmap='viridis', edgecolor='none')
    # ax.plot_trisurf(aggregated_values, aggregated_filters, aggregated_ates, cmap='viridis', edgecolor='none')
    ax.scatter(aggregated_values, aggregated_filters, aggregated_ates)
    # surf = ax.plot_surface(np.array(aggregated_values), np.array(aggregated_filters), np.array(aggregated_ates), cmap=cm.coolwarm,
                    #    linewidth=0, antialiased=False)
    # Set labels and title
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_title('3D Scatter Plot')
    plt.savefig("test_fig_scat.png")

# Main function1
slam_alg= None
def main():
    global slam_alg
    if len(sys.argv) > 1 :
        slam_folder = sys.argv[1]
        slam_alg = os.path.basename(slam_folder.rstrip('/'))
    else:
        exit(1)
    plot_filter_data("brightness", slam_folder)


if __name__ == "__main__":
    main()
