FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y upgrade
RUN apt-get -y update 
# time to build slamfuse
# handle dependencies
RUN apt-get -y install \
    libvtk6.3 \
    libvtk6-dev \
    unzip \
    libflann-dev \
    wget \
    mercurial \
    git \
    gcc \
    g++ \
    cmake \
    python-numpy \
    freeglut3 \
    freeglut3-dev \
    libglew-dev \
    libglu1-mesa \
    libglu1-mesa-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dev \
    libxmu-dev \
    libxi-dev  \
    libboost-all-dev \
    cvs \
    libgoogle-glog-dev \
    libatlas-base-dev \
    gfortran  \
    gtk2.0 \
    libgtk2.0-dev  \
    libyaml-dev \
    build-essential \
    libyaml-cpp-dev
# download git folder
RUN git clone --branch SLAMBench-conteinarisation https://github.com/nikolaradulov/SLAMFuse.git slamfuse
WORKDIR /slamfuse
# RUN git checkout 93fa32123c8878a311ecb02acd2ad8971292a4ab
RUN apt-get install python
COPY entry.sh /slamfuse
RUN chmod +x /slamfuse/entry.sh
# COPY ./framework/tools/accuracy-tools/CMakeLists.txt /slamfuse/framework/tools/accuracy-tools
#COPY ./framework/makefiles/deps.makefile /slamfuse/framework/makefiles
# RUN	make brisk
#	+make ceres
#	+make cvd
RUN make eigen3
RUN make flann -j$(expr $(nproc) - 2)
#	+make freeimage
# RUN make g2o
#	+make gvars
RUN make opencv -j$(expr $(nproc) - 2)
#	+make opengv
#	+make opentuner
RUN make pangolin -j$(expr $(nproc) - 2)
RUN make pcl -j$(expr $(nproc) - 2)
# RUN make suitesparse
#	+make toon
#	+make Sophus
# build slambench
RUN make slambench
ENTRYPOINT [ "./entry.sh" ]