### devel-mujoco

install mujoco

```bash
git clone https://github.com/google-deepmind/mujoco.git
git checkout 3.2.7
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

compile rl_sar

```bash
# build with catkin
./build.sh -s mujoco
# build with cmake
./build.sh -m -s mujoco
```

test with go2w

```bash
# build with catkin
rosrun rl_sar rl_sim go2w
# build with cmake
./cmake_build/bin/rl_sim go2w
```

### TODO

ROS2 `Segmentation fault (core dumped)`
