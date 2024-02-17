import numpy as np
import json
import os
from pathlib import Path
import sys
import subprocess

############# Change to select datasets and algorithms + settings #############
datasets=[
    "tum",
    "icl-nuim", 
    # "eurocMAV"
    "kitty"
]
algorithms=["lsd"]
repeats = 5
base_repeat = 3
granularity = 10 # how many range increases will take place till max / min range 
percentage = 0.10
if len(sys.argv) >=2:
    prefix=sys.argv[1]
else: prefix =""
# in each direction
###############################################################################
 
brightness_step = 255/granularity
contrast_step = 10/granularity
contrast_inv_step = 1/granularity
blur_step = 10/granularity
steps ={"blur":blur_step, "contrast":contrast_step, "contrast_inv":contrast_inv_step, "brightness":brightness_step }
filters = {1:"blur", 2:"contrast", 3:"brightness", 4:"noise"}
base ={ "blur":1, "contrast":1, "brightness":0}
no_frames={
    "kitty": {
        "test": 1106,
        "lsd": 1106,
        "open_vins": -1
    },
    "eurocMAV":{
        "test": 3682,
        "lsd": 3682
    },
    "tum":{
        "test": 1362,
        "lsd": 1362

    },
    "icl-nuim":{
        "lsd": 967,
        "test": 967,
        "open_vins": -1
    }
}
dataset_paths={
    "kitty":"./datasets/KITTI/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam",
    "icl-nuim": "datasets/ICL_NUIM/living_room_traj1_loop.slam",
    "tum": "datasets/TUM/freiburg1/rgbd_dataset_freiburg1_room.slam",
    "eurocMAV": "datasets/EuRoCMAV/machine_hall/MH_01_easy/MH_01_easy.slam",
}

algorithm_paths={
    "test": "build/lib/libtest-cpp-library.so",
    "lsd": "build/lib/liblsdslam-cpp-library.so"
    "open_vins": "build/lib/libopen"
}


def generate_orbslam_pick(k, n, range_start, range_end):
    result = []
    for _ in range(k):
        # Pick n random numbers within the range
        random_numbers = np.sort(np.random.choice(range_start, range_end + 1, size=n))
        # Construct the array with the number and its 2 preceding and 2 succeeding numbers
        array = np.concatenate([np.arange(num - 2, num + 3) for num in random_numbers])
        result.append(array)
    return result

def generate_conf(filter, setting, frames):
    # frames = np.random.choice(np.arange(total_frames + 1), size=20, replace=False)
    frame_list =[]
    for frame in frames:
        if filter=="brightness":
            brightness = setting
        else: brightness=0
        if filter=="contrast":
            contrast = setting
        else: contrast=0
        if filter=="blur":
            blur = setting
        else:
            blur =0
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
        print(f"At{algorithm} - {dataset}:")
        print(f"    ->frames={max_frames}")
        print(f"    ->frame_step={frame_step}\n\n\n")
        for filter in range(4):
            if filter == 0:
                for j in range(base_repeat):
                    dir_path = f"{prefix}/{algorithm}/{dataset}/base_{j+1}"
                    os.makedirs(Path(dir_path), exist_ok=True)
            else:
                for t in range(2*frame_step, max_frames+1, frame_step):
                    frame_rand = np.random.choice(np.arange(1, no_frames[dataset][algorithm]+1), size=(repeats, t))
                    if algorithm == "orbslam3":
                        frame_rand = generate_orbslam_pick(repeats, (int)(t/5), 1, no_frames[dataset][algorithm]+1)

                    #  for each run pick different random, but repeat as the granularity increases
                    # eg. run one for all granularities will be the same frames
                    #  run 2 will be the same for all granularities but different set from run 1, etc.
                    for i in range(1,granularity+1):
                        for turn in range(repeats):
                            setting = i*steps[filters[filter]]
                            dir_path = f"{prefix}/{algorithm}/{dataset}/{filters[filter]}/frames{t}/exp{turn}_frames_val_{setting}"
                            os.makedirs(Path(dir_path), exist_ok=True)
                            conf = generate_conf(filters[filter], setting, frame_rand[turn])
                            # print(conf)
                            with open(Path(dir_path+"/conf.json"), "w") as json_file:
                                json.dump(conf, json_file, indent=4)
                            command = f"~/slambench/build/bin/slambench -i ~/slambench/{dataset_paths[dataset]} -load ~/slambench/{algorithm_paths[algorithm]}  --log-file {dir_path}/log_file.txt -enhance {dir_path}/conf.json -img {dir_path}/image_metrics.txt"
                            print(f"        ->{dir_path}")
                            try:
                                subprocess.run(command, shell=True, capture_output=True, timeout=900)
                            except subprocess.TimeoutExpired:
                                print("                 ->timeout")
                                open(Path(f'{dir_path}/Timeout.txt'), "a").close()
                            
                            #  do reverse for brightness and contrast
                            if(filters[filter] == "brightness"):
                                setting=-setting
                                dir_path = f"{prefix}/{algorithm}/{dataset}/{filters[filter]}/frames{t}/exp{turn}_frames_val_{setting}"
                                os.makedirs(Path(dir_path), exist_ok=True)
                                conf = generate_conf(filters[filter], setting, frame_rand[turn])
                                # print(conf)
                                with open(Path(dir_path+"/conf.json"), "w") as json_file:
                                    json.dump(conf, json_file, indent=4)
                                command = f"~/slambench/build/bin/slambench -i ~/slambench/{dataset_paths[dataset]} -load ~/slambench/{algorithm_paths[algorithm]}  --log-file {dir_path}/log_file.txt -enhance {dir_path}/conf.json -img {dir_path}/image_metrics.txt"
                                print(f"        ->{dir_path}")
                                try:
                                    subprocess.run(command, shell=True, capture_output=True, timeout=900)
                                except subprocess.TimeoutExpired:
                                    print("                 ->timeout")
                                    open(Path(f'{dir_path}/Timeout.txt'), "a").close()
                                    
                            elif(filters[filter]=="contrast"):
                                setting = i*steps["contrast_inv"]
                                dir_path = f"{prefix}/{algorithm}/{dataset}/{filters[filter]}/frames{t}/exp{turn}_frames_val_{setting}"
                                os.makedirs(Path(dir_path), exist_ok=True)
                                conf = generate_conf(filters[filter], setting, frame_rand[turn])
                                # print(conf)
                                with open(Path(dir_path+"/conf.json"), "w") as json_file:
                                    json.dump(conf, json_file, indent=4)
                                command = f"~/slambench/build/bin/slambench -i ~/slambench/{dataset_paths[dataset]} -load ~/slambench/{algorithm_paths[algorithm]}  --log-file {dir_path}/log_file.txt -enhance {dir_path}/conf.json -img {dir_path}/image_metrics.txt"
                                print(f"        ->{dir_path}")
                                try:
                                    subprocess.run(command, shell=True, capture_output=True, timeout=900)
                                except subprocess.TimeoutExpired:
                                    print("                 ->timeout")
                                    open(Path(f'{dir_path}/Timeout.txt'), "a").close()
                                    
