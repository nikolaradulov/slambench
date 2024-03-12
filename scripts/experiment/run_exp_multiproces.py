import numpy as np
import json
import os
from pathlib import Path
import sys
import subprocess
from multiprocessing import Pool, cpu_count

############# Change to select datasets and algorithms + settings #############
datasets=[
    #"icl-nuim",
    #"icl-nuim0",
    #"icl-nuim3",
    # "eurocMAV"
    #"kitty"
    # "tum0"
    # 'tum1_xzy',
    'tum1_rpy'
    # 'tum2_xzy',
    # 'tum2_rpy'
]
algorithms=["orbslam3"]
repeats = 5
base_repeat = 5
granularity = 10 # how many range increases will take place till max / min range 
percentage = 0.11
if len(sys.argv) >=2:
    prefix=sys.argv[1]
else: prefix =""
# in each direction
###############################################################################
 
brightness_step = 255/granularity
contrast_step = 255/granularity
# contrast_inv_step = 1/granularity
blur_step = 10/granularity
steps ={"blur":blur_step, "contrast":contrast_step, "brightness":brightness_step }
# filters = {
#     0:"base",
#     1:"blur", 
#     2:"contrast",
#     3:"brightness" 
#     # 4:"noise"
#     }
filters=[
    "base",
    "blur",
    "contrast"
]
base ={ "blur":1, "contrast":1, "brightness":0}
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
        "orbslam3": 594,
        "orbslam2": 594

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
    'tum1_rpy':{
        'orbslam3': 721,
        'orbslam2': 721
    },
    'tum2_rpy':{
        'orbslam3': 3286,
        'orbslam2': 3286
    }
}
dataset_paths={
    "kitty":"./datasets/KITTI/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam",
    "icl-nuim": "datasets/ICL_NUIM/living_room_traj2_loop.slam",
    "icl-nuim0": "datasets/ICL_NUIM/living_room_traj0_loop.slam",
    "icl-nuim3": "datasets/ICL_NUIM/living_room_traj3_loop.slam",
    "tum": "datasets/TUM/freiburg1/rgbd_dataset_freiburg1_desk.slam",
    "eurocMAV": "datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam",
    "tum2_rpy": "datasets/TUM/freiburg2/rgbd_dataset_freiburg2_rpy.slam",
    "tum2_xzy": "datasets/TUM/freiburg2/rgbd_dataset_freiburg2_xyz.slam",
    "tum1_rpy": "datasets/TUM/freiburg1/rgbd_dataset_freiburg1_rpy.slam",
    "tum1_xzy": "datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam"
}

algorithm_paths={
    "test": "~/slambench/build/lib/libtest-cpp-library.so",
    "lsd": "~/slambench/build/lib/liblsdslam-cpp-library.so",
    "open_vins": "build/lib/libopen",
    "orbslam3": "/deps/orbslam3/lib/liborbslam3-original-library.so",
    "orbslam2": "/deps/orbslam2/lib/liborbslam2-original-library.so"
}

def generate_orbslam_pick(k, n, range_start, range_end):
    result = []
    n=max(n,1)
    for _ in range(k):
        picked_numbers = []  # List to store the picked numbers
        
        # Randomly pick n numbers within the specified range without replacement
        while len(picked_numbers) < n:
            num = np.random.randint(range_start + 15, range_end - 15 + 1)  # Ensure no overlap with the surrounding 15 numbers
            if all(abs(num - picked) > 15 for picked in picked_numbers):  # Ensure no overlap with previously picked numbers
                picked_numbers.append(num)
        
        picked_numbers.sort()  # Sort the picked numbers
        
        arrays = []
        for num in picked_numbers:
            # Construct the array with the number and its surrounding 15 numbers
            array = np.arange(num - 15, num + 16)
            arrays.append(array)
        
        result.append(np.concatenate(arrays))
    return result

def generate_conf(filter, setting, frames):
    # frames = np.random.choice(np.arange(total_frames + 1), size=20, replace=False)
    frame_list =[]
    for frame in frames:
            brightness = setting if filter=="brightness" else 0
            contrast = setting if filter=="contrast" else 0
            blur = setting if filter=="blur" else 0

            mean = 0
            std = 0
            sensor_settings= {
                "mean": mean,
                "standard_deviation": std,
                "kernel_size": blur,
                "brightness": brightness,
                "contrast": contrast
            }
            frame={
                "id": (int)(frame),
                "camera": [filter],
                "sensor_settings":{
                    "camera": sensor_settings
                }
            }
            frame_list.append(frame)
    return {"frames":frame_list}

def generate_command(filter: str, setting: int, algorithm: str, dataset: str, frame_idx: int, turn: int, frame_rand):
    dir_path = f"{prefix}/{algorithm}/{dataset}/{filter}/frames{frame_idx}/exp{turn}_frames_val_{setting}"
    os.makedirs(Path(dir_path), exist_ok=True)
    conf = generate_conf(filter, setting, frame_rand[turn])
    # print(conf)
    with open(Path(dir_path+"/conf.json"), "w") as json_file:
        json.dump(conf, json_file, indent=4)
    command = f"./build/bin/slambench -i {dataset_paths[dataset]} -load {algorithm_paths[algorithm]}  --log-file {dir_path}/log_file.txt -enhance {dir_path}/conf.json -img {dir_path}/image_metrics.txt"
    # print(f"        ->{dir_path}")
    # print(command)
    return command, dir_path

def run_bash_command(command, dir_path):
    try:
        print(command)
        result = subprocess.run(command, shell=True, capture_output=True, timeout=900)
    except subprocess.TimeoutExpired:
        print("                 ->timeout")
        open(Path(f'{dir_path}/Timeout.txt'), "a").close()

def generate_all_commands():
    commands = []
    paths = []
    for algorithm in algorithms:
        os.makedirs(Path(f"{prefix}/{algorithm}"), exist_ok=True)
        for dataset in datasets:
            # for algorithm / datasets that don't work together
            if no_frames[dataset][algorithm] ==-1:
                continue
            path = f"{prefix}/{algorithm}/{dataset}"
            os.makedirs(Path(path), exist_ok=True)
            # select max_frames 
            max_frames  = (int)(no_frames[dataset][algorithm]*percentage)
            frame_step = (int)(max_frames /3) 
            print(f"At {algorithm} - {dataset}:")
            print(f"    ->frames={max_frames}")
            print(f"    ->frame_step={frame_step}\n\n\n")
            for filter in filters:
                if filter == "noise":
                    continue
                if filter == "base":
                    for j in range(base_repeat):
                        dir_path = f"{prefix}/{algorithm}/{dataset}/base_{j+1}"
                        os.makedirs(Path(dir_path), exist_ok=True)
                        commands.append(f"./build/bin/slambench -i ~/slambench/{dataset_paths[dataset]} -load {algorithm_paths[algorithm]} --log-file {dir_path}/log_file.txt -img {dir_path}/image_metrics.txt")
                        paths.append(dir_path)
                else:

                    for frame_idx in range(frame_step, max_frames+1, frame_step):
                        # print((int)(t/30))
                        if dataset == "kitty":
                            # print((int)(t/7))
                            frame_rand = generate_orbslam_pick(repeats, (int)(frame_idx/7), 1, no_frames[dataset][algorithm]+1)
                        else:
                            frame_rand = generate_orbslam_pick(repeats, (int)(frame_idx/30), 1, no_frames[dataset][algorithm]+1)
                          
                        #  for each run pick different random, but repeat as the granularity increases
                        # eg. run one for all granularities will be the same frames
                        #  run 2 will be the same for all granularities but different set from run 1, etc.
                        for i in range(0,granularity+1):
                            # skip 0 value for blur
                            for turn in range(repeats):
                                setting = i*steps[filter]
                                new_command, dir_path = generate_command(filter, setting, algorithm, dataset, frame_idx, turn, frame_rand)
                                commands.append(new_command)
                                paths.append(dir_path)
                                #  do reverse for brightness and contrast
                                if(filter == "brightness" or filter == "contrast") and i!=0:
                                    setting=-setting
                                    new_command, dir_path = generate_command(filter, setting, algorithm, dataset, frame_idx, turn, frame_rand)
                                    commands.append(new_command)
                                    paths.append(dir_path)
    assert len(commands) == len(paths)
    return commands, paths

if __name__ == "__main__":
    commands, paths = generate_all_commands()
    num_workers = cpu_count() - 2 # so your computer doesn't crash
    # Create a pool of worker processes
    with Pool(processes=num_workers) as pool:
        # Submit commands to the pool for parallel execution
        results = pool.starmap(run_bash_command, zip(commands, paths))

    # Process the results returned by the pool
    for result in results:
        continue

