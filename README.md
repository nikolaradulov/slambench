<!-- ================================================== Disclaimer ============================================================ -->
# Disclaimer
SLAMFuse is still under development and not all features presented in the paper are currently available on  main 'SLAMFuse' branch. We are actively working on improving 
the tool so have a look [here](https://github.com/nikolaradulov/SLAMFuse/issues/30) if you can't find a feature you were looking for.  
<!-- ==================================================Prerequisites============================================================ -->
# 1. ___Prerequisites___
## 1.1 Recommended System:
* Linux distribution.
* Windows WSL2

## 1.2 Docker
Install [Docker Engine](https://docs.docker.com/engine/install/). <br>
If you are using Windows10 WSL2, you can EITHER install [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/) OR run following command:
```
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh get-docker.sh
$ sudo service docker start
$ sudo chmod 777 /var/run/docker.sock
```

<!-- ==============================================Start with Docker========================================================= -->
# 2. ___Start with Docker___
## 2.1 Build SLAMFuse
```
$ sudo apt-get install git
$ git clone https://github.com/nikolaradulov/SLAMFuse.git slamfuse # NEED TO CHANGE THE LINK IF MOVES TO PAMELA-PROJECT
$ cd ~/slamfuse/
$ docker build . -t slamfuse/main
```
## 2.2 Build Algorithm
```
$ cd ~/slamfuse/
$ ./scripts/algorithm-vol.sh [algorithm name]
$ ./scripts/algorithm-vol.sh floam
```
Replace [algorithm name] by `kfusion`, `orbslam3`, `floam`, etc.
Now you have: 
* **Docker IMAGE:** [algorithm name]-img
* **Docker CONTAINER:** [algorithm name]
* **Docker VOLUME:** [algorithm name]-vol

The `[algorithm name]-vol` contains all the necessary components for  evaluating algorithm on SLAMFuse. <br>
**Hints**: If Docker images takes too much memory and you already have `[algorithm name]-vol`, you can use `docker rm [algorithm name]`, `docker rmi [algorithm name]-img` and `docker system prune` to free memory.

## 2.3 Build dataset
```
$ python3 starter.py dataset -t make -d ./datasets/KITTI/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam -v KITTI07
```
check `~/slamfuse/datasets/command.txt` for more details.

## 2.4 Run Algorithm
```
$ python3 starter.py run -t gui -dv KITTI07 -d 2011_09_30_drive_0027_sync.slam -a floam/libfloam-original-library.so
```
**If you want to have more control of the filesystem**, then you will need `-t interactive-gui`, it allows you to modify the configuration file, visualize results, and so on.
```
$ python3 starter.py run -t interactive-gui -dv KITTI07 -d 2011_09_30_drive_0027_sync.slam -a floam/libfloam-original-library.so
```
