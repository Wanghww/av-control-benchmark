# 第十五至十六周：MPC 控制器实现

## 当前进度

第四阶段第十五至十六周完成：MPC 控制器实现，全部单元测试通过，S 弯三控制器对比完成。

---

## 实现方案

### MPC 类型选择

采用**误差状态横向 MPC**，纵向仍用 PID：

- 状态量：`[e_y, e_theta]`（横向误差、航向误差）
- 控制量：`delta`（前轮转角）
- 纵向速度：复用 PID 控制器

这是真实工业实现的常见架构（如 Apollo 的 MPC 控制器），横纵向解耦简化了 QP 规模。

### 线性化误差模型

在当前状态处线性化车辆运动学模型：

```
e_{k+1} = A e_k + B u_k

A = [[1, v·dt],    B = [    0    ]
     [0,   1 ]]         [v/L·dt  ]
```

### QP 问题结构

```
决策变量：z = [x_1,...,x_N, u_0,...,u_{N-1}]，共 3N 个（N=10 时为 30 个）

代价函数（OSQP 形式 min 0.5 z^T P z + q^T z）：
  P = diag(2Q, 2Q, ..., 2Qf, 2R, 2R, ..., 2R)
  q = [0,...,0, -2R·ff, ..., -2R·ff]    （ff 为曲率前馈转角）

约束：
  等式（动力学）：x_{k+1} = A x_k + B u_k    共 2N 个
  不等式（转角）：|u_k| ≤ 0.52 rad (±30°)   共 N 个
```

### 前馈项

前馈转角 `ff = atan(L × kappa)` 写入代价函数的线性项，引导优化在曲率段提前打方向。

---

## 新增文件

```
cpp_controllers/include/mpc_controller.hpp   ← MPC 控制器头文件
cpp_controllers/src/mpc_controller.cpp       ← QP 构建与 OSQP 求解
cpp_controllers/tests/test_mpc.cpp           ← 单元测试
cpp_controllers/build/results/mpc_output.csv ← 仿真轨迹数据
cpp_controllers/build/results/lqr_output.csv ← LQR 仿真轨迹数据
```

### CMakeLists.txt 变更

```cmake
find_package(OsqpEigen REQUIRED)
target_link_libraries(controllers Eigen3::Eigen OsqpEigen::OsqpEigen)
```

---

## 踩坑记录

**Waypoint 类型不匹配**

`generateSCurveLQR()` 返回 `vector<LQRController::Waypoint>`，但 `MPCController::compute()` 需要 `vector<MPCController::Waypoint>`。两者字段完全相同但 C++ 视为不同类型，需做显式转换：

```cpp
for (auto& wp : lqr_traj)
    traj.push_back({wp.x, wp.y, wp.yaw, wp.v_ref, wp.curvature});
```

---

## 验证结果

```bash
./test_controllers    # 6 个测试全部通过
./run_simulation
```

| 控制器 | S 弯 RMSE | 说明 |
|--------|-----------|------|
| Pure Pursuit | 0.0650 m | 几何法，无优化 |
| LQR | 0.0635 m | 当前步最优 |
| MPC | 0.0637 m | 预测 10 步最优 |

S 弯低速场景三者 RMSE 接近，符合预期。MPC 的预测优势在**高速急弯**场景才能体现，这是第十七周双移线测试的核心验证点。

---

## 下一阶段

**第十七周：三个测试场景**

1. 直线变速（纵向 PID 验证）
2. S 弯对比（已完成，见本周数据）
3. **ISO 3888 双移线**（核心场景，预期 MPC 比 Pure Pursuit 低 30% 以上）
