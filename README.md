# 基于 [ pointcloud_to_grid ](https://github.com/jkk-research/pointcloud_to_grid) 修改

# 效果图

![test.gif](./doc/test.gif)

![results.png](./doc/results.png)

# 使用步骤
```bash
mkdir -p ~/pointcloud_to_grid/src
cd ~/pointcloud_to_grid/src
git clone git@github.com:zylyehuo/pointcloud_to_grid.git
```
```bash
# 获取二维栅格地图的参数
# 运行脚本后播放 bag 数据
cd ~/pointcloud_to_grid/src/pointcloud_to_grid/scripts
python3 cloud_bounds.py
```
```bash
# 搭配定位程序使用，订阅定位程序输出的点云话题
cd ~/lidar2base
source install/setup.bash
ros2 launch pointcloud_to_grid demo.launch.py topic:=<定位程序输出的点云话题>
```
