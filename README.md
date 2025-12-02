# Install dependencies
sudo apt install \
    liborocos-kdl-dev \
    liburdfdom-dev \
    libkdl-parser-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    cmake

# Copy files
mkdir -p ~/pose-validator/cpp
cp ~/Downloads/ned2_validator.cpp ~/pose-validator/cpp/
cp ~/Downloads/validate_yaml_poses.cpp ~/pose-validator/cpp/
cp ~/Downloads/CMakeLists.txt ~/pose-validator/cpp/

# Build
cd ~/pose-validator/cpp
mkdir build && cd build
cmake ..
make