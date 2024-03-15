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
from scipy.interpolate import interp2d
from matplotlib.colors import LinearSegmentedColormap

def extract_number(folder_name):
    return int(folder_name.replace("frames", ""))

fail_points={
    "icl-nuim": 1,
    "kitty" : 5,
    "icl-nuim3": 1,
    "icl-nuim0": 1,
    "tum": 0.03, 
    "tum1_rpy": 0.1,
    "tum1_xzy": 0.1,
    "tum_desk": 1, 
    "tum_rpy": 0.1,
    "tum_xzy": 0.1
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
        "lsd": 594, 
        "orbslam3": 594,
        "orbslam2": 594

    },
    "tum_desk":{
        "test": 1362,
        "lsd": 594, 
        "orbslam3": 594,
        "orbslam2": 594,
        'efusion': 595

    },
    "icl-nuim":{
        "lsd": 882,
        "test": 882,
        "open_vins": -1,
        "orbslam3": 882,
        "orbslam2": 882
    },
    "icl-nuim3":{
        "lsd": 882,
        "test": 882,
        "open_vins": -1,
        "orbslam3": 1241,
        "orbslam2": 1241
    },
    "icl-nuim0":{
        "lsd": 882,
        "test": 882,
        "open_vins": -1,
        "orbslam3": 1509,
        "orbslam2": 1509
    },
    'tum1_xzy':{
        'orbslam3': 797,
        'orbslam2': 797
    },
    'tum2_xzy':{
        'orbslam3': 3665,
        'orbslam2': 3665
    },
    'tum_xyz':{
        'orbslam3': 797,
        'orbslam2': 797,
        'efusion': 797
    },
    'tum1_rpy':{
        'orbslam3': 721,
        'orbslam2': 721
    },
    'tum_rpy':{
        'orbslam3': 721,
        'orbslam2': 721,
        'efusion': 721
    },
    'tum2_rpy':{
        'orbslam3': 3286,
        'orbslam2': 3286
    }
}
dataset_names = {
    "icl-nuim": "icl-nuim traj 2",
    "kitty" : "KITTI07",
    "icl-nuim3": "icl-nuim traj 3",
    "icl-nuim0": "icl-nuim traj 0",
    "tum": "Tum Freiburg1 desk",
    "tum1_rpy": "Tum freiburg 1 rpy",
    "tum1_xzy": "Tum freiburg 1 xyz",
    "tum_desk": "Tum Freiburg1 desk",
    "tum_rpy": "Tum freiburg 1 rpy",
    "tum_xyz": "Tum freiburg 1 xyz"
}
def parse_log_file(log_file_path):
    with open(log_file_path, 'r') as file:
        lines = file.readlines()
    found_table= False
    fail= False
    mean_ate_values = []
    current_coord=None
    consecutive_count = 0
    for line in lines:
        if line.startswith('Frame Number'):
            found_table=True
            continue
        if(found_table):
            values = line.split()
            ate_rmse = float(values[2])
            if not math.isnan(ate_rmse):
                mean_ate_values.append(ate_rmse)
            coord = (float(values[10]), float(values[11]), float(values[12]))
            if coord == current_coord:
                consecutive_count += 1
                if consecutive_count > 500 and not fail:
                    fail = True    # Violation detected
                    print(log_file_path)
            else:
                current_coord = coord
                consecutive_count = 1
    return np.array(mean_ate_values), fail


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
    counter = 0
    with open(file_path, 'r') as file:
        for line in file:
            counter+=1
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


def get_default_metrics(dataset_folder):
    base_folder=os.path.join(dataset_folder, "base_1")
    metrics = read_image_metrics(os.path.join(base_folder, "image_metrics.txt"))
    # print(metrics)
    errors=[]
    for i in range(1,4):
        ates, _ = parse_log_file(os.path.join(base_folder, "log_file.txt"))
        errors.append(np.mean(ates))
        base_folder=os.path.join(dataset_folder, f"base_{i+1}")
    return metrics, np.mean(errors) 

def compute_avg_quality_diff(measured, base, modified_ids, filter_type):
    perturbed_metrics = [metric[filter_type] for metric in measured if metric['frame_id'] in modified_ids]
    base_metrics = [metric[filter_type] for metric in base if metric['frame_id'] in modified_ids]
    difference = [x - y for x, y in zip(perturbed_metrics, base_metrics)]
    return np.mean(difference)

def get_mean_ate_evolution(dataset_folder, filter_type, dataset_name):

    global slam_alg
    no_invalid = 0
    filter_folder = os.path.join(dataset_folder, filter_type)
    frame_folders = [f for f in os.listdir(filter_folder) if os.path.isdir(os.path.join(filter_folder, f))]
    total_filters=[]
    total_values=[]
    total_ates=[]
    base_quality, base_error = get_default_metrics(dataset_folder)
    print(base_error)
    frame_folders = sorted(frame_folders,  key=extract_number)
    # print(frame_folder)
    for frame_folder in frame_folders:
        # if(frame_folder!=frame_folders[1]):
        #     continue
        # if dataset_name=="icl-nuim3" and frame_folder!="frames90":
        #     continue
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
        value_change = []
        mean_values =[]
        partial_failure = {'x':[], 'y':[]}
        failure_points = {'x':[], 'y':[]}
        for filter_val in filter_values:
            fail_flag=5
            mean_ate_values_all_runs = []
            mean_values_all_runs = []
            measured_change_all_runs =[]
            for run in exp_numbers:
                invalid = False
                exp_folder = f"exp{run}_frames_val_{filter_val}"
                # print(exp_folder)
                exp_path = os.path.join(frame_path, exp_folder)
                log_file_path = os.path.join(exp_path, 'log_file.txt')
                modified_ids = read_modified_frame_ids(os.path.join(exp_path, "conf.json"))
                quality_metrics = read_image_metrics(os.path.join(exp_path, "image_metrics.txt"))
               
                if len(quality_metrics)<no_frames[dataset_name][slam_alg]-20 :
                    # print("=======================")
                    # print(exp_path)
                    # print("---------------------------")
                    no_invalid +=1
                    invalid = True
                # print(quality_metrics[modified_ids[4]])
                if invalid:
                    continue
                mean_ate_values, failed = parse_log_file(log_file_path)
                # print(failed)
                quality_metric = compute_mean_metric(quality_metrics, modified_ids, filter_type)
                mean_values_all_runs.append(quality_metric)
                # if filter_val == 0:
                #     mean_ate_values_all_runs.append(0)
                #     measured_change_all_runs.append(0)
                #     continue
                
                measured_change_all_runs.append(compute_avg_quality_diff(quality_metrics, base_quality, modified_ids, filter_type))
                # just in case give some liniency
                mean_ate = np.mean(mean_ate_values)
                if mean_ate >0.03:
                    mean_ate=0.03
                
                if(len(mean_ate_values)<no_frames[dataset_name][slam_alg]-20 or failed):
                    mean_ate_values_all_runs.append(fail_points[dataset_name]-base_error)
                    fail_flag-=1
                else:
                    # print(np.mean(mean_ate_values))
                    mean_ate_values_all_runs.append(mean_ate-base_error)
            # print(mean_ate_values_all_runs)
            mean_values.append(np.mean(mean_values_all_runs))
            ate_per_filer.append(np.mean(mean_ate_values_all_runs))
            value_change.append(np.mean(measured_change_all_runs))
            if(fail_flag<5 and fail_flag>0):
                partial_failure['x'].append(filter_val)
                partial_failure['y'].append(np.mean(mean_ate_values_all_runs))
            elif(fail_flag==0):
                failure_points['x'].append(filter_val)
                failure_points['y'].append(np.mean(mean_ate_values_all_runs))
        total_filters = filter_values
        total_values = mean_values
        total_ates = ate_per_filer       
    print(no_invalid)
    return np.array(total_filters), np.array(total_values), np.array(total_ates), failure_points, partial_failure
    
    

def plot_filter_data(filter_type,slam_name_folder):
    dataset_folders = [f for f in os.listdir(slam_name_folder) if os.path.isdir(os.path.join(slam_name_folder, f))]
    aggregated_filters = []
    aggregated_values = []
    aggregated_ates=[]
    aggregated_pfail = {'x':[], 'y':[]}
    aggregated_fail = {'x':[], 'y':[]}
    for dataset_folder in dataset_folders:
        # if not dataset_folder.startswith("tum"):
        #     continue
        if dataset_folder != "tum":
            continue
        # if dataset_folder != "kitty":
        #     continue
        print(f"----> Retrieving from {dataset_folder}")
        dataset_path = os.path.join(slam_name_folder, dataset_folder)
        # get filter data for specific filter and dataset
        filters, mean_values, ates , fail, pfail = get_mean_ate_evolution(dataset_path, filter_type, dataset_folder)
        # ates = ates/(fail/_points[dataset_folder]/2)
        print(mean_values, ates)
        plt.plot(filters, ates, label=f'{dataset_names[dataset_folder]}')
        # plt.scatter(fail['x'], fail['y'], marker='+', label='Total failure', c='black')
        # plt.scatter(pfail['x'], pfail['y'], marker='x', label='Partial failure', c='black')
        aggregated_fail['x'].extend(fail['x'])
        aggregated_pfail['x'].extend(pfail['x'])
        aggregated_fail['y'].extend(fail['y'])
        aggregated_pfail['y'].extend(pfail['y'])
        aggregated_filters.extend(filters)
        aggregated_values.extend(mean_values)
        aggregated_ates.extend(ates)
    
    # Create custom colormap from grey to redbri    
    # colors = [(0, 'darkred'), (1, 'lightgrey')]
    # custom_cmap = LinearSegmentedColormap.from_list('custom_cmap', colors)

    plt.scatter(aggregated_filters, aggregated_ates, c=aggregated_values, cmap='coolwarm',  s=60)
    
    # Set labels and title
    plt.xlabel(f'Applied filter')
    plt.ylabel('Difference of MeanATE')
    # plt.yscale('symlog', base=2)
    # plt.ylim(ymax=2)
    plt.title(f'ORB-SLAM3 {filter_type} influence on error')
    
    cbar = plt.colorbar()
    cbar.set_label('Average Quality metric')
    plt.scatter(aggregated_fail['x'], aggregated_fail['y'], marker='x', label='Total failure', c='black')
    plt.scatter(aggregated_pfail['x'], aggregated_pfail['y'], marker='+', label='Partial failure', c='black')
    
    # Create a dictionary to store unique labels and corresponding handles
    handles, labels = plt.gca().get_legend_handles_labels()
    unique_labels = {}
    for label, handle in zip(labels, handles):
        if label not in unique_labels:
            unique_labels[label] = handle
    plt.legend(unique_labels.values(), unique_labels.keys(),loc='upper center', bbox_to_anchor=(0.5, -0.2), shadow=True, ncol=3)
    # plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), shadow=True, ncol=3)
    plt.subplots_adjust(bottom=0.3) 
    plt.grid(True)
    plt.savefig("contrast_orbslam3_tum.png")
    # plt.show()
# Main function1
slam_alg= None
def main():
    global slam_alg
    if len(sys.argv) > 1 :
        slam_folder = sys.argv[1]
        slam_alg = os.path.basename(slam_folder.rstrip('/'))
    else:
        exit(1)
    plot_filter_data("contrast", slam_folder)


if __name__ == "__main__":
    main()
