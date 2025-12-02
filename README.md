# Install dependencies
```bash
sudo apt install \
    liborocos-kdl-dev \
    liburdfdom-dev \
    libkdl-parser-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    cmake
```
# Build
```bash
cd ~/pose-validator/cpp
mkdir build && cd build
cmake ..
make
```