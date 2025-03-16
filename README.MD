

# RGBD GS-ICP SLAM (100FPS Gaussian Splatting SLAM)

赵逸彬正在对其进行改进

## 日历文档
### 20250307

commit了GSICP原版，增加了一些pose colmap存储

解决了PSNR下降的问题，在于training_setup中天杀的xyz_gradient_accum会累积，不能重复定义

### 20250309

改写了Mapper，可以设置render多少new和多少random

new_train_time = 10 / random_train_time = 19 的情况下（先做10位姿估计，在render19随机）：
只有17.06 还不如原版

鬼知道哪里出问题了

### 20250310

6b5a6466中检查发现问题，在mapping keyframe的时候高斯没有加入

改写后解决该问题，在1/19的情况下取消camera的优化器可以达到17.5

new_train_time = 10 / random_train_time = 19 的情况下（先做10位姿估计，在render19随机）：可以达到17.5，还是基本没有什么变化

需要注意跳转gaussian_model.py里面的位姿优化lr，不然可能会爆炸

给mapping keyframe也增加了位姿优化（只作用于高斯不更新位姿）（似乎更新位姿了会出问题，但是加不加应该问题不大，因为其实不是顺序帧的）

### 20250312
修改了camera_q/t的编写方式，现在每一个新视角都会有一个独特的q/t并保持优化，PSNR不变

### 20250316
统计了时间，原版的

render time: 0.000538 | loss cal time: 0.005996 | loss back time: 0.006944

***

我们的

trans_gaussian_time: 0.001770

render_time: 0.000487 | loss cal time: 0.003126 | loss back time: 0.030524

由于多了一些pytorch的backward，会导致速度大大降低

在增加了optimizer_camera之后测试了多轮训练，结果没有提升，与第二篇文章的结果不符，应该是有错误