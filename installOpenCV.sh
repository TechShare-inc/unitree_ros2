cd $HOME
git clone -b 4.1.1 https://github.com/opencv/opencv.git
cd opencv
# Exit immediately if a command exits with a non-zero status.
set -e

# Define the build directory
BUILD_DIR="build"
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# Configure the project
# Replace <Your_Project_Source_Dir> with the actual source directory of your project
# Specify the CMake options you need
cmake .. -DWITH_CUDA=ON -DWITH_GSTREAMER=ON

# Build the project
make

# Install the project
sudo make install