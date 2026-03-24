## 实车挑战赛轨迹规划
JJLin  2026/3/23
### 目前效果
* 合并了分支
* 添加了gitignore
### TODO
编译测试
修改函数细节


### 运行方法
#### 编译
```bash
cd avdp_homework/src
catkin_init_workspace
cd ..
rm -r devel
rm -r build
catkin_make
```
#### 运行
务必退出conda，四个终端分别运行
```bash
source devel/setup.bash
roslaunch cssr cssr.launch
```