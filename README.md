## FAST-LIO-SAM-Localization-SC-QN

本仓库包含两部分：
- **FAST-LIO-SAM-SC-QN**：基于 FAST-LIO2 的里程计 + iSAM2 位姿图优化 + ScanContext 回环检测，结合 Quatro 与 Nano-GICP 做回环配准的 SLAM。
- **FAST-LIO-Localization-SC-QN**：基于保存好的地图（.bag）进行定位的系统，使用 ScanContext 进行候选匹配，Quatro 提供全局粗配准初值，Nano-GICP 进行精配准。

两者可单独使用，也可联动：SLAM 节点在线发布 `odom -> base_link` 与矫正点云，定位节点据此估计并发布 `map -> odom`。

## 目录结构
- `FAST-LIO-SAM-SC-QN/fast_lio_sam_sc_qn`：SLAM 节点
- `FAST-LIO-Localization-SC-QN/fast_lio_localization_sc_qn`：定位节点
- `third_party/`：Quatro、Nano-GICP、ScanContext、FAST_LIO 配置等

## 依赖
- C++ >= 17, OpenMP >= 4.5, CMake >= 3.10, Eigen >= 3.2, Boost >= 1.54
- ROS
- GTSAM >= 4.1.1
- TEASER++
- TBB

安装要点（示例）：
```bash
# GTSAM（SLAM 需要）
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
unzip gtsam.zip && cd gtsam-4.1.1 && mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j$(nproc)

# TEASER++（两包共享）
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
sudo make install -j$(nproc) && sudo ldconfig

# TBB
sudo apt install -y libtbb-dev
```

## 编译
在已配置好的 catkin 工作空间中：
```bash
cd ~/your_workspace/src
# 若你不是通过 --recursive 获取本仓库，请先初始化子模块
# git submodule update --init --recursive

# 将本仓库放入 src 后回到工作空间根目录
cd ~/your_workspace

# 先编译依赖模块（建议 Release）
catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON

# 再编译整个工作空间
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 运行方式
### 1) 运行 SLAM（FAST-LIO-SAM-SC-QN）
- 修改 `FAST-LIO-SAM-SC-QN/fast_lio_sam_sc_qn/config/config.yaml` 参数与 `third_party/FAST_LIO` 的传感器配置。
- 启动：
```bash
roslaunch fast_lio_sam_sc_qn run.launch lidar:=ouster
# 或 lidar:=velodyne / livox / kitti / mulran / newer-college2020 等预设
```
- 关键话题（输出）：
  - `/ori_odom`、`/ori_path`：原始点轨迹
  - `/corrected_odom`、`/corrected_path`：图优化后轨迹
  - `/corrected_current_pcd`：实时矫正点云
  - `/odom_to_baselink`：发布 odom->base_link（nav_msgs/Odometry）
  - `/loop_detection`：回环可视化
- 保存地图：向 `/save_dir`（std_msgs/String）发送保存目录，可将关键帧点云与位姿保存为 `.bag`、`.pcd`，并可输出 KITTI/TUM 格式位姿。

### 2) 运行定位（FAST-LIO-Localization-SC-QN）
- 将 SLAM 产生的 `.bag` 地图路径填入 `FAST-LIO-Localization-SC-QN/fast_lio_localization_sc_qn/config/config.yaml` 的 `basic.saved_map`。
- 启动：
```bash
roslaunch fast_lio_localization_sc_qn run.launch lidar:=livox
```
- 该 launch 默认包含 SLAM 的 launch（用于提供话题与可视化）。若需纯定位，可在定位包 `run.launch` 中移除 `<include>` 或改为订阅外部里程计/点云。
- 关键话题（输出）：
  - `/pose_stamped_localization`：实时定位位姿（map 框架）
  - `/map_to_odom`：估计的 map->odom（nav_msgs/Odometry）
  - `/corrected_current_pcd_localization`：矫正后的当前帧点云
  - `/saved_map_localization`：加载的地图点云（按需发布）

## 数据流与坐标系
- SLAM：
  - 输入：`/Odometry`（来自 FAST_LIO）、`/cloud_registered`（点云）
  - 输出：`/odom_to_baselink`（odom->base_link）、`/corrected_current_pcd` 等
- 定位：
  - 订阅 SLAM 输出的 `/odom_to_baselink` 与 `/corrected_current_pcd`
  - 基于地图匹配估计 `map->odom`，并以 nav_msgs/Odometry 形式发布 `/map_to_odom`
  - 实时位姿为：`map->odom * odom->base_link`

## 常用参数（示例）
- SLAM：
  - `basic.loop_update_hz`、`basic.vis_hz`、`keyframe.keyframe_threshold`
  - `scancontext_max_correspondence_distance`
  - Quatro/Nano-GICP 参数（迭代次数、对应数量、阈值等）
- 定位：
  - `basic.saved_map`（.bag 地图路径）
  - `basic.map_match_hz`、`basic.visualize_voxel_size`、`keyframe.keyframe_threshold`
  - `match.scancontext_max_correspondence_distance`、`match.quatro_nano_gicp_voxel_resolution`

参数示例与默认值参见各包 `config/config.yaml`。

## 地图制作与使用流程建议
1. 使用 SLAM 运行数据集，确认轨迹闭环并稳定。
2. 向 `/save_dir` 发布保存目录（或在配置中开启保存标志），导出 `.bag` 地图。
3. 在定位包配置中填写 `basic.saved_map` 为上述 `.bag` 路径。
4. 启动定位，检查 `/map_to_odom` 与 `/pose_stamped_localization` 是否稳定。

## RViz 预设
- SLAM：`fast_lio_sam_sc_qn/config/sam_rviz.rviz`
- 定位：`fast_lio_localization_sc_qn/config/localization_rviz.rviz`

## 许可
- 子仓库各自遵循其原始许可证。
- 若用于学术研究，请引用原作者与相关论文/项目。
