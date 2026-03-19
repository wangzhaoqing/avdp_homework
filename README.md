## 实车挑战赛轨迹规划
JJLin  2026/3/19
### 目前效果
只修改了opt.cpp和opt.h
可以规划轨迹并输出，大体沿着参考轨迹在跑
* 采样空间为横向-1m到1m，5个点；纵向速度0到10m/s，10个点，时间2s到5s，4个点。可以在Opt构造函数内修改
* 基本完成了简单采样、轨迹生成
* 需要继续完成判断碰撞和计算代价部分，目前都是随便生成的方法
### TODO
```cpp
bool Opt::checkCollision()
double Opt::calculateCost()
```