import sys
import os
import subprocess

def make(target):
    subprocess.run(['make', target], check=True)

def bench_gui(data_path, algorithm_path, extra):
    # Install required packages (assuming Debian-based system)
    subprocess.run(['sudo', 'apt', 'install', '-y', 'vainfo', 'mesa-va-drivers'], check=True)

    # Set environment variables
    os.environ['DEBIAN_FRONTEND'] = 'noninteractive'
    os.environ['LIBVA_DRIVER_NAME'] = 'd3d12'
    os.environ['LD_LIBRARY_PATH'] = '/usr/lib/wsl/lib'

    # Find the slam_file
    slam_file = find_slam_file(data_path)
    print(slam_file)

    if os.path.isfile(algorithm_path):
        subprocess.run(['./build/bin/slambench', '-i', slam_file, '-load', algorithm_path, '--gui', 'true'], check=True)
    else:
        vol_name = os.path.basename(os.path.dirname(algorithm_path))
        os.chdir(f"/deps/{vol_name}/")
        subprocess.run(['cmake', '.'], check=True)
        subprocess.run(['make'], check=True)
        subprocess.run(['/slambench/build/bin/slambench', '-i', slam_file, '-load', algorithm_path, '--gui', 'true'], check=True)

def bench_cli(data_path, algorithm_path, extra):
    # Find the slam_file
    slam_file = find_slam_file(data_path)
    print(slam_file)

    if not os.path.isfile(algorithm_path):
        vol_name = os.path.basename(os.path.dirname(algorithm_path))
        os.chdir(f"/deps/{vol_name}/")
        subprocess.run(['cmake', '.'], check=True)
        subprocess.run(['make'], check=True)
    subprocess.run(['/slambench/build/bin/slambench', '-i', slam_file, '-load', algorithm_path], check=True)

def testing():
    make("g2o")
    make("opencv")
    make("suiteparse")
    make("lsdslam")
    make("slambench APPS=lsdslam")
    make("datasets/ICL_NUIM/living_room_traj2_loop.slam")
    subprocess.run(['./build/bin/slambench', '-i', 'datasets/ICL_NUIM/living_room_traj2_loop.slam', '-load', './build/lib/liblsdslam-cpp-library.so'], check=True)

def from_cfg():
    config_file = sys.argv[2]
    config = configparser.ConfigParser()
    config.read(config_file)

    # Read the mode from the config file
    mode = config['DEFAULT']['Mode']

    # Read dataset volume and file from the config file
    dataset_volume = config['DATASET']['Volume']
    dataset_file = config['DATASET']['Path']

    # Read algorithm volumes and implementations from the config file
    algorithm_volumes = []
    algorithm_implementations = []
    for section in config.sections():
        if section.startswith('ALGORITHM'):
            algorithm_volumes.append(config[section]['Volume'])
            algorithm_implementations.append(config[section]['Implentation'])

    # Extract any extra arguments from the config file
    extra_args = ""
    if 'EXTRA_ARGS' in config['DEFAULT']:
        extra_args = config['DEFAULT']['EXTRA_ARGS']

    

def find_slam_file(filename):
    slam_file = None
    for root, _, files in os.walk('./datasets'):
        for file in files:
            if file == filename:
                slam_file = os.path.join(root, file)
                break
    return slam_file

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Invalid argument. Usage: python script.py [--test | --dataset <dataset_name> | --bench-gui <data_path> <algorithm_path> | --bench-cli <data_path> <algorithm_path>]")
        sys.exit(1)

    argument = sys.argv[1]

    if argument == "--test":
       
    elif argument == "--dataset":
        if len(sys.argv) < 3:
            print("Missing dataset name.")
            sys.exit(1)
        dataset_name = sys.argv[2]
        make(dataset_name)

    elif argument == "--bench-gui" :
        _, data_path, algorithm_path = sys.argv
        bench_gui(data_path, algorithm_path)
    
        

    elif argument == "--interactive":
        subprocess.run(['/bin/bash'], check=True)

    elif argument == "--list_datasets":
        make("datasets")

    elif argument == "--from-config":
        


    else:
        print("Invalid argument. Usage: python script.py [--test | --dataset <dataset_name> | --bench-gui <data_path> <algorithm_path> | --bench-cli <data_path> <algorithm_path>]")
        sys.exit(1)
