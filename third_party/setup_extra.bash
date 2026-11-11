# sudo 없이 로컬에 풀어둔 Nav2 / slam_toolbox 환경 (ROS humble setup 이후 source)
_X="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/root"
export AMENT_PREFIX_PATH="$_X/opt/ros/humble:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$_X/opt/ros/humble:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$_X/opt/ros/humble/lib:$_X/opt/ros/humble/lib/x86_64-linux-gnu:$_X/usr/lib/x86_64-linux-gnu:$_X/usr/lib/llvm-14/lib:$_X/usr/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$_X/opt/ros/humble/lib/python3.10/site-packages:$_X/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH"
export PATH="$_X/opt/ros/humble/bin:$PATH"
