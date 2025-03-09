import numpy as np
import os
import copy
import open3d as o3d



def save_poses(Tracker_class):
    np.save(Tracker_class.dataset_path + "/posesnp",np.array(Tracker_class.trajmanager.gt_poses))
    import math
    poses = Tracker_class.poses
    img_num = Tracker_class.poses.__len__()
    dataset = Tracker_class.dataset_path
    fname2pose = {}
    if not os.path.exists(dataset + '/sparse/pose'):
        # 如果文件夹不存在，则创建它
        os.makedirs(dataset + '/sparse/pose')
    with open(dataset + '/sparse/pose/cameras.txt', 'w') as f:
        f.write(f'1 PINHOLE {Tracker_class.W} {Tracker_class.H} {Tracker_class.fx} {Tracker_class.fy} {Tracker_class.cx} {Tracker_class.cy}')
        idx = 1
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            # blend到opencv的转换：y轴和z轴方向翻转
            # pose = np.array(frame['transform_matrix']) @ blender2opencv
            pose = np.array(poses[i])
            fname2pose.update({fname: pose})

    with open(dataset + '/sparse/pose/images.txt', 'w') as f:
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            pose = fname2pose[fname]
            # 参考https://blog.csdn.net/weixin_44120025/article/details/124604229：colmap中相机坐标系和世界坐标系是相反的
            # blender中：world = R * camera + T; colmap中：camera = R * world + T
            # 因此转换公式为
            # R’ = R^-1
            # t’ = -R^-1 * t
            R = np.linalg.inv(pose[:3, :3])
            T = -np.matmul(R, pose[:3, 3])
            # R = pose[:3, :3]
            # T = pose[:3, 3]
            q0 = 0.5 * math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
            q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
            q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
            q3 = (R[1, 0] - R[0, 1]) / (4 * q0)

            f.write(f'{idx} {q0} {q1} {q2} {q3} {T[0]} {T[1]} {T[2]} 1 {fname}\n\n')
            idx += 1
    with open(dataset + '/sparse/pose/points3D.txt', 'w') as f:
        f.write('')

def save_gt_poses(Tracker_class):
    import math
    poses = copy.deepcopy(Tracker_class.trajmanager.gt_poses)
    img_num = Tracker_class.gt_poses.__len__()
    dataset = Tracker_class.dataset_path
    if not os.path.exists(dataset + '/sparse/gt'):
        # 如果文件夹不存在，则创建它
        os.makedirs(dataset + '/sparse/gt')
    fname2pose = {}
    with open(dataset + '/sparse/gt/cameras.txt', 'w') as f:
        f.write(f'1 PINHOLE {Tracker_class.W} {Tracker_class.H} {Tracker_class.fx} {Tracker_class.fy} {Tracker_class.cx} {Tracker_class.cy}')
        idx = 1
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            # blend到opencv的转换：y轴和z轴方向翻转
            # pose = np.array(frame['transform_matrix']) @ blender2opencv
            pose = np.array(poses[i])
            fname2pose.update({fname: pose})

    with open(dataset + '/sparse/gt/images.txt', 'w') as f:
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            pose = fname2pose[fname]
            # 参考https://blog.csdn.net/weixin_44120025/article/details/124604229：colmap中相机坐标系和世界坐标系是相反的
            # blender中：world = R * camera + T; colmap中：camera = R * world + T
            # 因此转换公式为
            # R’ = R^-1
            # t’ = -R^-1 * t
            R = np.linalg.inv(pose[:3, :3])
            T = -np.matmul(R, pose[:3, 3])
            # R = pose[:3, :3]
            # T = pose[:3, 3]
            q0 = 0.5 * math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
            q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
            q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
            q3 = (R[1, 0] - R[0, 1]) / (4 * q0)

            f.write(f'{idx} {q0} {q1} {q2} {q3} {T[0]} {T[1]} {T[2]} 1 {fname}\n\n')
            idx += 1
    with open(dataset + '/sparse/gt/points3D.txt', 'w') as f:
        f.write('')

def save_gt_poses_random_error(Tracker_class):
    import math
    poses = copy.deepcopy(Tracker_class.trajmanager.gt_poses)
    img_num = Tracker_class.gt_poses.__len__()
    dataset = Tracker_class.dataset_path
    if not os.path.exists(dataset + '/sparse/gt_random_error'):
        # 如果文件夹不存在，则创建它
        os.makedirs(dataset + '/sparse/gt_random_error')
    fname2pose = {}
    with open(dataset + '/sparse/gt_random_error/cameras.txt', 'w') as f:
        f.write(f'1 PINHOLE {Tracker_class.W} {Tracker_class.H} {Tracker_class.fx} {Tracker_class.fy} {Tracker_class.cx} {Tracker_class.cy}')
        idx = 1
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            # blend到opencv的转换：y轴和z轴方向翻转
            # pose = np.array(frame['transform_matrix']) @ blender2opencv
            poses_trans = poses[i]

            error = np.random.normal(loc=0.005, scale=0.002)
            # TUM



            # error = np.random.normal(loc=0.002, scale=0.0015)
            # REPLICA


            theta = np.arccos(2 * np.random.rand() - 1)  # 极角在 [0, pi] 范围内
            phi = 2 * np.pi * np.random.rand()  # 方位角在 [0, 2pi] 范围内

            # 将球坐标转换为笛卡尔坐标
            x = np.sin(theta) * np.cos(phi)
            y = np.sin(theta) * np.sin(phi)
            z = np.cos(theta)

            # 生成单位向量
            unit_vector = np.array([x, y, z])

            # 缩放向量到给定长度
            vector = error * unit_vector
            poses_trans[:3,3] = poses_trans[:3,3] + vector
            # REPLICA





            pose = np.array(poses_trans)
            fname2pose.update({fname: pose})

    with open(dataset + '/sparse/gt_random_error/images.txt', 'w') as f:
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            pose = fname2pose[fname]
            # 参考https://blog.csdn.net/weixin_44120025/article/details/124604229：colmap中相机坐标系和世界坐标系是相反的
            # blender中：world = R * camera + T; colmap中：camera = R * world + T
            # 因此转换公式为
            # R’ = R^-1
            # t’ = -R^-1 * t
            R = np.linalg.inv(pose[:3, :3])
            T = -np.matmul(R, pose[:3, 3])
            # R = pose[:3, :3]
            # T = pose[:3, 3]
            q0 = 0.5 * math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
            q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
            q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
            q3 = (R[1, 0] - R[0, 1]) / (4 * q0)

            f.write(f'{idx} {q0} {q1} {q2} {q3} {T[0]} {T[1]} {T[2]} 1 {fname}\n\n')
            idx += 1
    with open(dataset + '/sparse/gt_random_error/points3D.txt', 'w') as f:
        f.write('')

def save_gt_poses_continus_error(Tracker_class):
    import math
    poses = copy.deepcopy(Tracker_class.trajmanager.gt_poses)
    img_num = Tracker_class.gt_poses.__len__()
    dataset = Tracker_class.dataset_path
    if not os.path.exists(dataset + '/sparse/gt_continus_error'):
        # 如果文件夹不存在，则创建它
        os.makedirs(dataset + '/sparse/gt_continus_error')
    fname2pose = {}
    with open(dataset + '/sparse/gt_continus_error/cameras.txt', 'w') as f:
        f.write(f'1 PINHOLE {Tracker_class.W} {Tracker_class.H} {Tracker_class.fx} {Tracker_class.fy} {Tracker_class.cx} {Tracker_class.cy}')
        idx = 1
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            # blend到opencv的转换：y轴和z轴方向翻转
            # pose = np.array(frame['transform_matrix']) @ blender2opencv
            poses_trans = poses[i]
            # continues_error = 0.027 * 2 * i / img_num
            continues_error = 0.0057 * 2 * i / img_num
            



            random_error = np.random.normal(loc=0.001, scale=0.001)
            # random_error = np.random.normal(loc=0.0005, scale=0.0001)
            # TUM
            # random_error = np.random.normal(loc=0.006, scale=0.003)
            # REPLICA


            theta = np.arccos(2 * np.random.rand() - 1)  # 极角在 [0, pi] 范围内
            phi = 2 * np.pi * np.random.rand()  # 方位角在 [0, 2pi] 范围内
            # 将球坐标转换为笛卡尔坐标
            x = np.sin(theta) * np.cos(phi)
            y = np.sin(theta) * np.sin(phi)
            z = np.cos(theta)
            # 生成单位向量
            unit_vector = np.array([x, y, z])
            # 缩放向量到给定长度
            random_error_vector = random_error * unit_vector


            continues_error_vector = np.zeros_like(random_error_vector)

            continues_error_vector[0] = continues_error


            poses_trans[:3,3] = poses_trans[:3,3] + random_error_vector + continues_error_vector
            # REPLICA

            



            pose = np.array(poses_trans)
            fname2pose.update({fname: pose})

    with open(dataset + '/sparse/gt_continus_error/images.txt', 'w') as f:
        for i in range(img_num):
            fname = str(i)
            if not (fname.endswith('.png') or fname.endswith('.jpg')):
                fname += '.png'
            pose = fname2pose[fname]
            # 参考https://blog.csdn.net/weixin_44120025/article/details/124604229：colmap中相机坐标系和世界坐标系是相反的
            # blender中：world = R * camera + T; colmap中：camera = R * world + T
            # 因此转换公式为
            # R’ = R^-1
            # t’ = -R^-1 * t
            R = np.linalg.inv(pose[:3, :3])
            T = -np.matmul(R, pose[:3, 3])
            # R = pose[:3, :3]
            # T = pose[:3, 3]
            q0 = 0.5 * math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
            q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
            q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
            q3 = (R[1, 0] - R[0, 1]) / (4 * q0)

            f.write(f'{idx} {q0} {q1} {q2} {q3} {T[0]} {T[1]} {T[2]} 1 {fname}\n\n')
            idx += 1
    with open(dataset + '/sparse/gt_continus_error/points3D.txt', 'w') as f:
        f.write('')

def save_images(Tracker_class):
    images_path = Tracker_class.trajmanager.color_paths
    depths_path = Tracker_class.trajmanager.depth_paths
    # saved_path =
    import shutil
    idx =0
    idx_depth = 0
    if not os.path.exists(os.path.join(Tracker_class.dataset_path,"images_colmap")):
        # 如果文件夹不存在，则创建它
        os.makedirs(os.path.join(Tracker_class.dataset_path,"images_colmap"))
    if not os.path.exists(os.path.join(Tracker_class.dataset_path,"depth_colmap")):
        # 如果文件夹不存在，则创建它
        os.makedirs(os.path.join(Tracker_class.dataset_path,"depth_colmap"))
    for path in images_path:
        shutil.copyfile(path, os.path.join(Tracker_class.dataset_path,"images_colmap",str(idx)+".png"))
        idx += 1
    for path in depths_path: 
        shutil.copyfile(path, os.path.join(Tracker_class.dataset_path,"depth_colmap",str(idx_depth)+"_depth.png"))
        idx_depth += 1

def save_frame_points(Tracker_class,points,idx):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if not os.path.exists(Tracker_class.dataset_path + "/pcd/"):
        # 如果文件夹不存在，则创建它
        os.makedirs(Tracker_class.dataset_path + "/pcd/")
    if not os.path.exists(Tracker_class.dataset_path + "/pcd/" + str(idx) + ".pcd"):
        o3d.io.write_point_cloud(Tracker_class.dataset_path + "/pcd/" + str(idx) + ".pcd", pcd)