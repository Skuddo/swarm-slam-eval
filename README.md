# swarm-slam-eval

if rqt_graph ever breaks delete ths file
```bash
rm ~/.config/ros.org/rqt_gui.ini
```

## Install

## Build
### First or clean Build
```bash
cd src/ 
rm -rf build/ log/ install/ (if not first build)

colcon build
```

### Dev builds
If we have alrady build the cslam repos 
```bash
colcon build --packages-select swarm_slam_eval swarm_slam_visualizer
```
