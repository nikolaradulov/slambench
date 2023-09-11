OPENCV_DIR=${DEPS_DIR}/opencv/share/OpenCV/
ANDROID_OPENCV_DIR=${ANDROID_DEPS_DIR}/opencv/share/OpenCV/
OPENCV_CONTRIB_MODULES_DIR=${REPOS_DIR}/opencv_contrib/modules

${REPOS_DIR}/opencv :
	mkdir -p ${REPOS_DIR}
	rm -rf ${REPOS_DIR}/opencv
	git clone "https://github.com/opencv/opencv.git" ${REPOS_DIR}/opencv
	git clone "https://github.com/opencv/opencv_contrib.git" ${REPOS_DIR}/opencv_contrib
	cd ${REPOS_DIR}/opencv && git checkout 3.4.3
	cd ${REPOS_DIR}/opencv_contrib && git checkout 3.4.3

# TODO: update opencv flags for opencv 3
${DEPS_DIR}/opencv : ${REPOS_DIR}/opencv
	cd ${REPOS_DIR}/opencv && mkdir -p build_dir
	cd ${REPOS_DIR}/opencv && rm -rf build_dir/*
	cd ${REPOS_DIR}/opencv/build_dir && cmake -D CMAKE_BUILD_TYPE=RELEASE  -D CMAKE_INSTALL_PREFIX=$@    -DCMAKE_CXX_FLAGS="-Wno-error=address" \
	-DWITH_GSTREAMER=OFF \
	-DWITH_FFMPEG=OFF \
	-DBUILD_PERF_TESTS=OFF \
	-DWITH_OPENCL=OFF \
	-DBUILD_WITH_DEBUG_INFO=OFF \
	-DWITH_1394=OFF  \
	-DBUILD_TESTS=OFF \
	-DWITH_TBB=OFF \
	-DWITH_V4L=OFF \
	-DWITH_OPENGL=OFF \
	-DBUILD_JAVA=OFF \
	-DWITH_CUDA=OFF \
	-DWITH_GTK=ON \
	-DBUILD_opencv_ml=ON \
	-DBUILD_opencv_video=ON \
	-DBUILD_opencv_gpu=OFF \
	-DBUILD_opencv_java=OFF \
	-DBUILD_opencv_videostab=OFF \
	-DBUILD_opencv_ts=OFF \
	-DBUILD_opencv_bioinspired=OFF  \
	-DBUILD_opencv_python=OFF \
	-DBUILD_opencv_text=OFF \
	-DBUILD_opencv_photo=OFF \
	-DBUILD_opencv_stitching=OFF \
	-DOPENCV_EXTRA_MODULES_PATH=${OPENCV_CONTRIB_MODULES_DIR} \
	-DENABLE_PRECOMPILED_HEADERS=OFF .. > ${REPOS_DIR}/opencv/build_dir/opencv_cmake.log
	cat ${REPOS_DIR}/opencv/build_dir/opencv_cmake.log
	+cd ${REPOS_DIR}/opencv/build_dir && make
	mkdir -p $@
	cd ${REPOS_DIR}/opencv/build_dir && make install

$(ANDROID_DEPS_DIR)/opencv : ${REPOS_DIR}/opencv  ${REPOS_DIR}/android-cmake
	cd ${REPOS_DIR}/opencv && mkdir -p build_dir
	cd ${REPOS_DIR}/opencv && rm -rf build_dir/*
	cd ${REPOS_DIR}/opencv/build_dir && cmake  -DBUILD_SHARED_LIBS=OFF  -DCMAKE_TOOLCHAIN_FILE=${REPOS_DIR}/android-cmake/android.toolchain.cmake -DANDROID_NDK=${ANDROID_NDK} -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI="armeabi-v7a with NEON"   -DWITH_GSTREAMER=OFF -DBUILD_PERF_TESTS=OFF  -D WITH_OPENCL=OFF -D CMAKE_INSTALL_PREFIX=$(ANDROID_DEPS_DIR) -D BUILD_WITH_DEBUG_INFO=OFF  -D WITH_1394=OFF -D BUILD_TESTS=OFF  -D WITH_TBB=OFF  -D WITH_V4L=OFF  -D WITH_OPENGL=OFF -D BUILD_opencv_gpu=OFF -D BUILD_opencv_java=OFF -D WITH_CUDA=OFF -DWITH_GTK=OFF  -D BUILD_opencv_videostab=OFF  -D BUILD_opencv_ts=OFF  -D BUILD_opencv_stitching=OFF   .. 
	+cd ${REPOS_DIR}/opencv/build_dir && make
	cd ${REPOS_DIR}/opencv/build_dir && make install

opencv : 	
	+if [ ! -d ${DEPS_DIR}/$@ ] ; then make ${DEPS_DIR}/$@ ; else echo "$@ skipped."; fi

android-opencv : ${ANDROID_DEPS_DIR}/opencv

.PHONY: opencv android-opencv
