### mujoco

install mujoco

```bash
git clone https://github.com/google-deepmind/mujoco.git
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

test mujoco simulate

```bash
simulate
```

compile rl_sar

```bash
./build.sh -m
```

test go2w

```bash
./cmake_build/bin/rl_sim src/robots/go2w_description/mjcf/scene.xml
```

### TODO

can't find `lodepng`, just comment it
