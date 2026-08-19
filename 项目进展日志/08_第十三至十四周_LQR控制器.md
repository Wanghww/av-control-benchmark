# 第十三至十四周：LQR 理论与实现

## 当前进度

第四阶段第十三至十四周完成：LQR 理论学习 + OSQP 环境安装 + LQR 控制器实现。

---

## 第十三周：理论学习

### LQR 核心思想

把路径跟踪问题转化为最优控制问题，最小化代价函数：

```
J = Σ (e_y² × q₁ + e_θ² × q₂ + δ² × r)

e_y = 横向误差，e_θ = 航向误差，δ = 转角
q₁、q₂ 越大 → 越在乎误差，跟踪精度高但可能抖动
r 越大 → 越在乎平顺，转向柔和但误差可能变大
```

误差状态方程（简化为 2 状态：横向误差 + 航向误差）：

```
e_{k+1} = A·e_k + B·δ

A = [[1, v·dt], [0, 1]]
B = [0, v/L·dt]
```

求解流程：黎卡提方程迭代 → 最优增益矩阵 K → 控制律 δ = -K·e（加曲率前馈项）。

### MPC 核心思想（对比预习）

MPC 是 LQR 的升级版：每步预测未来 N 步，求解一个二次规划（QP）问题找最优控制序列，只执行第一步（滚动优化）。比 LQR 多两个能力：**预测未来** + **处理约束**。求解需要专门的 QP 求解器 → OSQP。

---

## 第十四周：环境安装 + LQR 实现

### OSQP 环境安装

在第二台电脑 WSL2 编译安装：

```bash
git clone https://github.com/osqp/osqp.git
# submodule update 跳过（新版OSQP用CMake FetchContent，不用submodule）
cmake .. -DBUILD_SHARED_LIBS=ON && make -j4 && sudo make install

git clone https://github.com/robotology/osqp-eigen.git
cmake .. && make -j4 && sudo make install
```

验证：CMake config 文件存在于 `/usr/local/lib/cmake/osqp/` 和 `/usr/local/lib/cmake/OsqpEigen/`（新版不生成 pkg-config 文件，属正常现象）。

### 新增文件

```
cpp_controllers/include/lqr_controller.hpp   ← LQR 控制器头文件
cpp_controllers/src/lqr_controller.cpp       ← LQR 实现（黎卡提迭代求解）
cpp_controllers/tests/test_lqr.cpp           ← 单元测试
```

### 踩坑记录

**GCC 编译错误：`default member initializer required before the end of its enclosing class`**

原因：嵌套 `struct Params` 不能直接作为构造函数的默认参数值（`= Params{}`）写在类内声明里。

解决：参照 `bicycle_model.hpp` 的写法，拆成两个构造函数：
```cpp
LQRController();                         // 无参，内部用 Params{}
explicit LQRController(const Params&);   // 有参，不设默认值
```

---

## 验证结果

```bash
./test_controllers   # 5 个测试全部通过（PID 3个 + Pure Pursuit 1个 + LQR 1个）
./run_simulation
```

| 控制器 | S 弯 RMSE |
|--------|-----------|
| Pure Pursuit | 0.0650 m |
| LQR | 0.0635 m |

LQR 略优于 Pure Pursuit。当前测试场景（低速 S 弯）区分度不大，符合预期——LQR 相比几何法的优势要在更复杂场景（弯道曲率变化大、高速）才能明显体现，这也是后续双移线场景要验证的点。

---

## 下一阶段

**第十五至十六周：实现 MPC 控制器**

- 用 OSQP 求解带约束的二次规划问题
- 支持横纵向联合控制（同时优化转角和加速度）
- 处理转角约束、转角变化率约束、加速度约束
