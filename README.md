# rtmaps_zed_cameras
- Install cuda 12.6 from this link https://developer.nvidia.com/cuda-12-6-0-download-archive following the instructions for the desired operating system and architecture.
- Install ZED SDK from this link https://www.stereolabs.com/en-fr/developers/release
- in the repo directory do the following commands: 
    - mkdir build
    - cd build
    - cmake .. -D"RTMAPS_SDKDIR=rtmaps_install_dir" -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs
    - make -j6