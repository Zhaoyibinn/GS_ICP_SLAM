

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

