docker build \
    --no-cache \
    --progress=plain \
    -f ./Dockerfile \
    -t samuelhrqe/ros2_realsense_x86_64:humble . 2>&1 | tee build.log