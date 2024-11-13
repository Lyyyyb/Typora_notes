# **深入解析 ORB-SLAM2 中地图点的坐标系及其表示**

ORB-SLAM（Oriented FAST and Rotated BRIEF SLAM）作为一种先进的实时视觉SLAM系统，广泛应用于机器人导航、增强现实（AR）等领域。理解其坐标系及地图点的表示方式，对于优化和应用该系统至关重要。本文将详细、逻辑清晰、专业且严谨地解析 ORB-SLAM 中地图点的坐标系，包括初始化过程、尺度设定、轨迹文件格式等关键内容。

---

### 1. **ORB-SLAM2 中的坐标系概述**

#### 1.1 坐标系的基本定义

在 SLAM 系统中，坐标系的选择和定义至关重要，因为所有地图点和相机位姿都是相对于某一固定坐标系进行表示的。ORB-SLAM 中的坐标系主要涉及以下几个部分：

- **世界坐标系（World Coordinate System）**：系统初始化时选定的基准坐标系，通常与第一帧相机位姿对齐。
- **相机坐标系（Camera Coordinate System）**：每个关键帧（KeyFrame）都有自己的相机坐标系，描述该帧相对于世界坐标系的位姿。

#### 1.2 地图点的表示

地图点是环境中被观测到的特征点，通过多视图几何（如三角化）在三维空间中重建。ORB-SLAM 中，地图点的坐标是相对于世界坐标系的三维坐标，表示为 \( \mathbf{P}_w = (X, Y, Z)^T \)。

---

### 2. **初始化过程与坐标系的确定**

初始化是 SLAM 系统中关键的一步，决定了后续所有地图点和位姿的坐标基准。在 ORB-SLAM 的初始化过程中，主要涉及以下步骤：

#### 2.1 单目 SLAM 的尺度模糊性

单目 SLAM（Monocular SLAM）由于仅依赖单个摄像头，无法直接测量真实世界的绝对尺度。这导致了所谓的尺度模糊性（Scale Ambiguity），即无法确定地图和相机运动的真实尺度。因此，初始化阶段需要采用特定方法来设定一个相对尺度。

#### 2.2 初始化步骤

初始化过程通常包括以下步骤：

1. **特征匹配与基础矩阵估计**：
   - 从第一帧和第二帧中提取特征点，并进行匹配。
   - 估计基础矩阵（Fundamental Matrix）或本质矩阵（Essential Matrix），用于恢复相对位姿。

2. **本质矩阵分解**：
   - 使用 SVD（奇异值分解）将本质矩阵分解为旋转矩阵 \( \mathbf{R} \) 和平移向量 \( \mathbf{t} \)。
   - 代码片段：
     ```cpp
     void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t) {
         cv::Mat u, w, vt;
         cv::SVD::compute(E, w, u, vt);
         
         u.col(2).copyTo(t);
         t = t / cv::norm(t); // 归一化平移向量
         // 分解得到两个可能的旋转矩阵 R1 和 R2
         // ...
     }
     ```
   - **归一化平移向量**：`t = t / cv::norm(t);` 将平移向量 \( \mathbf{t} \) 归一化，确保其模长为1。这一步骤解决了单目 SLAM 的尺度模糊性，设定了一个相对单位尺度。

3. **三角化与地图初始化**：
   - 使用初始关键帧对的位姿，进行特征点的三角化，生成初始地图点。
   - 初始地图点的坐标相对于世界坐标系，且初始平移向量的模长被设定为1个单位。

4. **全局捆绑调整（Global Bundle Adjustment, BA）**：
   - 对初始地图点和相机位姿进行优化，最小化重投影误差，进一步提升初始化的准确性。

#### 2.3 坐标系的确定

初始化完成后，世界坐标系由第一帧的相机位姿定义，所有地图点的坐标均相对于该坐标系表示。由于单目 SLAM 的尺度模糊性，初始平移向量被归一化为单位长度，地图的尺度是相对的，依赖于场景中的特征点分布和移动。

---

### 3. **尺度归一化与中位深度调整**

尽管初始化阶段设定了一个相对尺度，但为了使地图的尺度更符合实际场景，ORB-SLAM 在初始化后进行了尺度归一化处理。

#### 3.1 计算中位深度

在初始化后，通过计算初始关键帧中地图点的中位深度（Median Depth），即所有地图点到相机的距离的中位数，用于估计场景的整体尺度。

```cpp
float medianDepth = pKFini->ComputeSceneMedianDepth(2);
float invMedianDepth = 1.0f / medianDepth;
```

#### 3.2 尺度调整

1. **平移向量缩放**：
   - 将当前关键帧的平移向量乘以 \( \text{invMedianDepth} \)，即 \( \mathbf{t}_{\text{scaled}} = \mathbf{t} \times \text{invMedianDepth} \)。
   - 这一步骤将相机的位姿从相对单位尺度调整为基于场景中位深度的实际尺度。

2. **地图点坐标缩放**：
   - 将所有地图点的坐标乘以 \( \text{invMedianDepth} \)，即 \( \mathbf{P}_{\text{scaled}} = \mathbf{P} \times \text{invMedianDepth} \)。
   - 使得地图点的分布更加符合真实场景的尺度。

```cpp
// 缩放当前关键帧的位姿
cv::Mat Tc2w = pKFcur->GetPose();
Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3) * invMedianDepth;
pKFcur->SetPose(Tc2w);

// 缩放所有地图点的坐标
vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
        MapPoint* pMP = vpAllMapPoints[iMP];
        pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
    }
}
```

#### 3.3 目的与意义

通过上述尺度归一化，ORB-SLAM 能够将地图的尺度调整为更接近实际环境的尺度，尽管单目 SLAM 无法直接测量绝对尺度。这一过程利用了场景中点的分布信息，提升了系统在实际应用中的准确性和鲁棒性。

---

### 4. **轨迹文件格式与含义**

ORB-SLAM 支持多种轨迹文件格式，用于保存和输出相机的运动轨迹。主要包括 TUM 格式和 KITTI 格式，两者在列数和含义上有所不同。

#### 4.1 TUM 格式（Trajectory TUM）

TUM 格式的轨迹文件通常用于 TUM RGB-D 数据集，与该数据集的评估工具兼容。每一行包含时间戳、位置坐标和四元数表示的姿态。

**列含义**：

1. **时间戳（Timestamp）**：浮点数，表示当前位姿的时间点。
2. **位置坐标（x, y, z）**：相机在世界坐标系中的位置。
3. **四元数（q1, q2, q3, q4）**：相机的旋转部分，表示为四元数 \( q = (q_1, q_2, q_3, q_4) \)。

**示例**：
```
timestamp x y z q1 q2 q3 q4
```

**代码实现参考**：

```cpp
// System::SaveTrajectoryTUM
cv::Mat R = pKF->GetRotation().t();
vector<float> q = Converter::toQuaternion(R);
cv::Mat t = pKF->GetCameraCenter();
f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " 
  << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
```

#### 4.2 KITTI 格式（Trajectory KITTI）

KITTI 格式主要用于 KITTI 数据集，与其评估工具兼容。每一行包含12个浮点数，表示3x4的位姿变换矩阵。

**列含义**：

1. **R00, R01, R02**：旋转矩阵的第一行。
2. **t0**：平移向量的第一个分量。
3. **R10, R11, R12**：旋转矩阵的第二行。
4. **t1**：平移向量的第二个分量。
5. **R20, R21, R22**：旋转矩阵的第三行。
6. **t2**：平移向量的第三个分量。

**示例**：
```
R00 R01 R02 t0 R10 R11 R12 t1 R20 R21 R22 t2
```

**代码实现参考**：

```cpp
// System::SaveTrajectoryKITTI
cv::Mat Tcw = (*lit) * Trw;
cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
cv::Mat twc = -Rwc * Tcw.rowRange(0,3).col(3);
f << setprecision(9)
  << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1) << " " << Rwc.at<float>(0,2) << " " << twc.at<float>(0) << " " 
  << Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1) << " " << Rwc.at<float>(1,2) << " " << twc.at<float>(1) << " " 
  << Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1) << " " << Rwc.at<float>(2,2) << " " << twc.at<float>(2) << endl;
```

#### 4.3 轨迹文件的用途

轨迹文件主要用于以下目的：

- **性能评估**：通过与地面真值轨迹对比，评估 SLAM 系统的精度。
- **后处理与可视化**：利用轨迹文件重建相机的运动路径，进行可视化展示。
- **后续算法应用**：为后续的路径规划、地图构建等算法提供位姿信息。

---

### 5. **详细分析初始化函数 `CreateInitialMapMonocular`**

`CreateInitialMapMonocular` 是 ORB-SLAM 中单目 SLAM 初始化的核心函数，负责从初始关键帧对创建地图点并进行尺度归一化。以下是该函数的详细解析：

#### 5.1 关键步骤分解

1. **创建关键帧（Create KeyFrames）**：
   - 将初始帧和相邻帧作为关键帧，提取并匹配特征点。
   
2. **插入关键帧到地图（Insert KFs in the Map）**：
   - 将关键帧添加到地图管理模块，建立关键帧之间的关联。

3. **创建地图点并关联到关键帧（Create MapPoints and Associate to KeyFrames）**：
   - 通过三角化生成地图点，并将其关联到对应的关键帧。

4. **更新连接关系（Update Connections）**：
   - 更新关键帧之间的连接关系，以构建位姿图。

5. **全局捆绑调整（Global Bundle Adjustment）**：
   - 优化地图点和相机位姿，减少重投影误差。
   
6. **尺度归一化（Scale Normalization）**：
   - 计算场景的中位深度，并根据中位深度调整地图的尺度。

#### 5.2 代码详解

以下是 `CreateInitialMapMonocular` 函数中关键代码段的详细解析：

```cpp
void Tracking::CreateInitialMapMonocular() {
    // 创建关键帧并插入地图
    // ...

    // 全局捆绑调整
    Optimizer::GlobalBundleAdjustemnt(mpMap, 20); // 迭代次数为20

    // 计算场景的中位深度
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;

    // 检查初始化是否正确
    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
        cout << "Wrong initialization, resetting..." << endl;
        Reset();
        return;
    }

    // 缩放初始基线
    cv::Mat Tc2w = pKFcur->GetPose(); // 从世界坐标系到当前关键帧的变换矩阵
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // 缩放所有地图点的坐标
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if (vpAllMapPoints[iMP]) {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
        }
    }

    // 后续处理
    // ...
}
```

**关键步骤解析**：

1. **全局捆绑调整（Global Bundle Adjustment）**：
   - 调用 `Optimizer::GlobalBundleAdjustemnt` 对地图和位姿进行优化，确保地图点和相机位姿的一致性和准确性。

2. **中位深度计算与尺度归一化**：
   - 使用 `ComputeSceneMedianDepth(2)` 计算初始场景的中位深度，参数 `2` 表示深度的计算方式（具体实现可能基于不同的深度计算策略）。
   - 计算中位深度的倒数 `invMedianDepth`，用于后续的尺度调整。

3. **初始化校验**：
   - 检查中位深度是否合理（`medianDepth >= 0`）以及跟踪到的地图点数量是否足够（`TrackedMapPoints(1) >= 100`），确保初始化的有效性。
   - 若初始化失败，调用 `Reset()` 重新初始化系统。

4. **相机位姿的尺度调整**：
   - 获取当前关键帧的位姿矩阵 `Tc2w`（从世界坐标系到相机坐标系的变换）。
   - 将位姿中的平移部分 `Tc2w.col(3).rowRange(0, 3)` 乘以 `invMedianDepth`，实现相机位姿的尺度归一化。
   - 更新关键帧的位姿。

5. **地图点坐标的尺度调整**：
   - 遍历所有初始地图点 `vpAllMapPoints`，将其世界坐标 `pMP->GetWorldPos()` 乘以 `invMedianDepth`，实现地图点坐标的尺度归一化。

**逻辑流程图**：

1. **初始化关键帧对** → 2. **提取与匹配特征点** → 3. **本质矩阵分解与位姿恢复** → 4. **三角化生成初始地图点** → 5. **全局捆绑调整优化** → 6. **计算中位深度** → 7. **尺度归一化** → 8. **完成初始化**

---

### 6. **总结与关键要点**

通过以上详细解析，我们可以总结出 ORB-SLAM 中地图点的坐标系及其表示方式的关键要点：

1. **坐标系基准**：
   - 世界坐标系由初始化关键帧的相机位姿定义，所有地图点和后续关键帧的位姿均相对于该坐标系表示。

2. **尺度设定**：
   - 在单目 SLAM 中，由于尺度模糊性，初始平移向量被归一化为单位长度。
   - 初始化后，通过计算场景的中位深度，进一步调整地图和位姿的尺度，使其更符合实际环境。

3. **轨迹文件格式**：
   - 支持 TUM 和 KITTI 两种轨迹文件格式，分别包含时间戳、位置坐标、四元数或旋转矩阵与平移向量。
   - 这些文件用于轨迹的评估、可视化及后续处理。

4. **初始化流程的稳健性**：
   - 通过特征匹配、位姿恢复、三角化和全局捆绑调整等步骤，确保初始化的准确性。
   - 尺度归一化步骤利用中位深度信息，解决了单目 SLAM 的尺度模糊性问题。

5. **代码实现细节**：
   - 关键函数如 `Initializer::DecomposeE` 和 `Tracking::CreateInitialMapMonocular` 通过归一化和平移缩放实现尺度设定和坐标系的确定。
   - 轨迹文件的输出函数明确了各列的含义，方便用户进行后续处理和分析。

理解这些细节不仅有助于更好地应用 ORB-SLAM，还为进一步的优化和定制提供了基础。例如，在实际应用中，用户可以根据特定需求调整初始化过程中的尺度设定策略，或根据不同的轨迹文件格式进行数据的解析与可视化。

---

### 7. **附录：术语解释与相关概念**

- **关键帧（KeyFrame）**：在 SLAM 系统中，选定的具有代表性和稳定性的帧，用于地图构建和位姿优化。
- **三角化（Triangulation）**：通过多视图几何方法，利用多帧的观测信息重建三维点的位置。
- **本质矩阵（Essential Matrix）**：描述两个相机之间相对旋转和平移的矩阵，用于恢复相机位姿。
- **四元数（Quaternion）**：一种用于表示三维空间中旋转的数学工具，避免了万向节锁（Gimbal Lock）问题。
- **捆绑调整（Bundle Adjustment, BA）**：一种全局优化方法，通过最小化重投影误差优化地图点和相机位姿。

---

通过以上全面、详细且严谨的解析，相信您对 ORB-SLAM 中地图点的坐标系及其表示方式有了深入的理解。如有进一步的问题或需要更具体的技术细节，欢迎继续交流探讨。