# 第一阶段：环境搭建 + Python 算法验证（第 1-4 周）

## 本阶段目标

用 Python 实现动态自行车模型仿真器，并实现 PID 纵向控制器和 Pure Pursuit 横向控制器，让仿真车辆跟踪给定轨迹。

**里程碑：CARLA 仿真窗口中看到车辆沿预设轨迹自动行驶，终端打印 RMSE 数据。**

> **开发流程：** 先用动态自行车模型快速验证控制器逻辑（无需启动 CARLA），再接入 CARLA 做真实仿真测试。控制器代码两种场景完全复用，这与工业界（Apollo、Autoware）的控制器开发流程一致。

---

## 第一周：双机环境搭建

### 1.0 双机工作方式说明

本项目使用两台电脑协作开发：

```
开发电脑（主机）                         第二台电脑（Windows 11）
  · VS Code + Remote-SSH 写代码          · WSL2 Ubuntu 22.04
  · Claude AI 辅助编程          ←SSH→    · 编译 C++ 代码
  · git push / pull                      · 运行 Python 仿真器
                                         · 运行 ROS2 节点
```

**日常工作流：**
1. 当前电脑打开 VS Code，通过 Remote-SSH 连接第二台电脑的 WSL2
2. 在 VS Code 里写代码（文件实际存在第二台电脑上）
3. 在 VS Code 终端（即第二台电脑的 Ubuntu 终端）里编译运行
4. 仿真结果（PNG）通过 `git push` 同步到 GitHub，当前电脑 `git pull` 后查看

---

### 1.1 在第二台电脑上安装 WSL2 + Ubuntu 22.04

**在第二台电脑（Windows 11）操作。以管理员身份打开 PowerShell：**

```powershell
# 指定 Ubuntu 22.04 版本（Windows 11 新版默认会装 24.04，必须显式指定）
wsl --install -d Ubuntu-22.04
```

安装完成后**重启电脑**。重启后自动弹出 Ubuntu 设置窗口，创建用户名（全小写英文，不含空格）和密码，记住密码后续要用。

**验证安装（在 Ubuntu 终端里运行）：**
```bash
lsb_release -a
# 预期输出：Ubuntu 22.04.x LTS
```

**预期产出：** 第二台电脑能打开 Ubuntu 22.04 终端，输出正确版本号。

---

### 1.2 在第二台电脑上开启 SSH 服务

**Step 1：安装 Windows OpenSSH Server（第二台电脑，Windows 设置里操作）**

```
设置 → 系统 → 可选功能 → 添加功能 → 搜索 "OpenSSH 服务器" → 安装
```

**Step 2：启动并设置开机自启（管理员 PowerShell）：**

```powershell
Start-Service sshd
Set-Service -Name sshd -StartupType 'Automatic'
```

**Step 3：查看第二台电脑的局域网 IP：**

```powershell
ipconfig
# 找 "WLAN" 或 "以太网适配器" 下的 IPv4 地址，如 192.168.1.xxx
```

记下这个 IP（后面用 `<第二台IP>` 代替）。

**预期产出：** `Start-Service sshd` 没有报错，记下 IP 地址。

---

### 1.3 从当前电脑连接第二台电脑

**安装 VS Code Remote-SSH 插件（当前电脑操作）：**

VS Code → 扩展（Ctrl+Shift+X）→ 搜索 `Remote - SSH` → 安装

**测试 SSH 连接（当前电脑，PowerShell 或终端）：**

```powershell
ssh 第二台用户名@<第二台IP>
# 例如：ssh wang@192.168.1.5
```

输入密码后，出现 `C:\Users\...>` 说明进入了第二台的 Windows。然后：

```
wsl
```

出现 `wang@PC:~$` 形式的提示符，说明成功进入 Ubuntu。`exit` 退出 WSL，`exit` 再退出 SSH。

**通过 VS Code 连接（日常推荐方式）：**

1. VS Code 左下角点蓝色 `><` 图标 → "连接到主机"
2. 输入 `第二台用户名@<第二台IP>` → 回车 → 输入密码
3. 连接成功后左下角显示 `SSH: <IP>`，此时已进入第二台 Windows
4. 打开 VS Code 终端（Ctrl+` ），输入 `wsl` 进入 Ubuntu
5. 在 Ubuntu 终端里执行 `code .`，VS Code 会弹出提示安装 WSL 扩展，安装后**左下角变为 `WSL: Ubuntu-22.04`**，此时文件编辑和终端均直接在 Ubuntu 里运行

> **注意：** 步骤 5 的 `code .` 首次运行需要下载约 100MB 的 VS Code Server，确保第二台电脑网络正常（无 VPN 限制，可直接下载）。若下载失败，参考 [VS Code 官方离线安装说明](https://code.visualstudio.com/docs/remote/vscode-server)。

**预期产出：** VS Code 左下角显示 `WSL: Ubuntu-22.04`，终端可直接运行 Ubuntu 命令。

---

### 1.4 安装 Python 依赖

**在第二台电脑的 Ubuntu 终端里运行（通过 VS Code 终端或 SSH 都可以）：**

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-dev python3-tk -y
pip3 install numpy matplotlib scipy
```

**验证安装：**

```bash
python3 -c "import numpy; print('NumPy OK'); import matplotlib; print('Matplotlib OK'); import scipy; print('SciPy OK')"
```

**预期输出：** 三行 OK，没有报错。

---

### 1.5 在第二台电脑上安装 CARLA

CARLA 作为**仿真服务器**运行在 Windows 上，WSL2 里的 Python 作为**客户端**通过 localhost 连接它。

**Step 1：下载 CARLA 0.9.15（Windows，约 10GB）**

浏览器打开：`https://github.com/carla-simulator/carla/releases/tag/0.9.15`

下载 `CARLA_0.9.15.zip`，解压到 `C:\CARLA_0.9.15\`。

**Step 2：测试 CARLA 能否启动（Windows，双击或 PowerShell）**

```powershell
cd C:\CARLA_0.9.15
.\CarlaUE4.exe
```

正常情况会弹出仿真窗口（城市街道场景），画面出现即成功。按 `Alt+F4` 关闭。

> **注意：** 第一次启动需要编译 Shader，等待 2-3 分钟是正常的。

**Step 3：在 WSL2 Ubuntu 里安装 CARLA Python 客户端**

```bash
pip3 install carla==0.9.15
```

**Step 4：验证 Python 能连上 CARLA**

先启动 CARLA（Windows 运行 `CarlaUE4.exe`），然后在 WSL2 终端：

```bash
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
print('CARLA 版本:', client.get_server_version())
print('连接成功！')
"
```

**预期输出：**
```
CARLA 版本: 0.9.15
连接成功！
```

> **如果 localhost 连接超时：** 少数 WSL2 配置下 localhost 不能直接转发到 Windows。改用 Windows 侧 IP：
> ```bash
> # 在 WSL2 终端获取 Windows 主机 IP
> cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
> # 把上面输出的 IP 替换掉 'localhost'
> # 例如：carla.Client('172.18.176.1', 2000)
> ```

**预期产出：** WSL2 Python 能连上 Windows 上运行的 CARLA，打印出版本号。

---

## 第二周：车辆模型 + 轨迹生成

本项目使用**动态自行车模型**作为仿真器的物理模型，对应真实四轮车的动力学行为。

**两种模型的关系：**

| 模型 | 用途 | 包含哪些物理量 |
|------|------|---------------|
| 运动学自行车模型 | 控制器设计基础（低速、小转角近似） | 几何约束，忽略惯性和轮胎力 |
| **动态自行车模型** | **仿真器（本项目使用）** | 轮胎侧偏力、质心侧偏角、横摆角速度 |

控制器仍基于运动学假设设计（工业界标准做法），仿真器使用动态模型体现真实车辆的欠转向/过转向特性。这与工业界的控制器开发流程一致：在动力学模型上测试通过后再上车。

**动态模型示意：**

```
        前轮（转向轮）
         /← 转角 δ
        /
  后轮 ──── 质心 ──── 航向角 ψ
              ↑
         侧偏角 β（车辆实际运动方向与车身方向之差）
         横摆角速度 r = ψ̇
```

**核心状态方程（线性轮胎模型）：**

```
纵向位置：  Ẋ = v · cos(ψ + β)
横向位置：  Ẏ = v · sin(ψ + β)
航向角：    ψ̇ = r

侧偏角：    β̇ = -(Cf+Cr)/(mv) · β  +  (-(Cf·lf-Cr·lr)/(mv²) - 1) · r  +  Cf/(mv) · δ
横摆角速度：ṙ = -(Cf·lf-Cr·lr)/Iz · β  -  (Cf·lf²+Cr·lr²)/(Iz·v) · r  +  Cf·lf/Iz · δ
纵向速度：  v̇ = a
```

**关键参数（参考普通轿车）：**
- `m = 1500 kg`：整备质量
- `Iz = 2500 kg·m²`：绕竖轴转动惯量（影响横摆响应速度）
- `lf = 1.2 m`：质心到前轴距离
- `lr = 1.5 m`：质心到后轴距离
- `Cf = Cr = 80000 N/rad`：前/后轮侧偏刚度

**侧偏刚度是什么？** 轮胎每偏转 1 弧度产生的横向力（牛顿）。侧偏刚度越大，轮胎越"硬"，横向响应越灵敏。运动学模型假设侧偏刚度无穷大（轮胎不打滑），动态模型则真实模拟了有限刚度下的轮胎特性。

---

### 2.2 初始化 Git 仓库并建立项目结构

**先在 GitHub 创建仓库（在 Windows 浏览器操作）：**
```
1. 登录 GitHub → New repository
2. 仓库名：av_control_benchmark
3. 选 Private（开发期间不公开）
4. 不要勾选 Initialize with README（后面手动加）
5. 创建完成，复制仓库 URL
```

**在第二台电脑的 Ubuntu 终端里初始化（通过 VS Code 终端或 SSH）：**

```bash
# 配置 git 身份（只需执行一次）
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"

# 创建项目目录并初始化
mkdir -p ~/av_control_benchmark/python_prototype/{models,controllers,simulation,utils}
cd ~/av_control_benchmark
git init
git remote add origin https://github.com/你的用户名/av_control_benchmark.git

touch python_prototype/__init__.py

# 创建 .gitignore
cat > .gitignore << 'EOF'
build/
__pycache__/
*.pyc
*.o
results/*.csv
*.egg-info/
EOF

git add .
git commit -m "init: project structure"
git push -u origin main
```

**在 VS Code 里打开项目（推荐方式）：**
```bash
# 在第二台电脑的 Ubuntu 终端里执行，VS Code 会自动切换到 WSL 模式
cd ~/av_control_benchmark
code .
```

---

### 2.3 实现动态自行车模型

创建文件 `python_prototype/models/dynamic_bicycle_model.py`：

```python
import numpy as np

class DynamicBicycleModel:
    """
    动态自行车模型（线性轮胎模型）
    对应真实四轮车的横向动力学，包含轮胎侧偏力和横摆动态

    状态量: [x, y, psi, v]（对控制器可见）
    内部状态: beta（质心侧偏角）, r（横摆角速度）
    控制量: [delta（前轮转角 rad）, a（加速度 m/s²）]
    """

    def __init__(self, dt=0.05):
        self.dt = dt

        # 车辆参数（参考普通轿车）
        self.m  = 1500.0      # 整备质量 (kg)
        self.Iz = 2500.0      # 绕 z 轴转动惯量 (kg·m²)
        self.lf = 1.2         # 质心到前轴距离 (m)
        self.lr = 1.5         # 质心到后轴距离 (m)
        self.L  = self.lf + self.lr   # 轴距

        # 轮胎参数（线性侧偏刚度）
        self.Cf = 80000.0     # 前轮侧偏刚度 (N/rad)
        self.Cr = 80000.0     # 后轮侧偏刚度 (N/rad)

        # 约束
        self.max_steer = np.radians(30)
        self.max_speed = 20.0
        self.max_accel = 3.0

        # 状态
        self.x    = 0.0
        self.y    = 0.0
        self.psi  = 0.0    # 航向角 (rad)
        self.v    = 0.0    # 纵向速度 (m/s)
        self.beta = 0.0    # 质心侧偏角 (rad)
        self.r    = 0.0    # 横摆角速度 (rad/s)

    def update(self, delta, a):
        """
        更新车辆状态一个时间步
        delta: 前轮转角 (rad)
        a:     加速度 (m/s²)
        """
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        a     = np.clip(a,    -self.max_accel,  self.max_accel)

        # 低速时用运动学近似，避免线性化在 v≈0 时的数值问题
        if self.v < 0.5:
            self.x    += self.v * np.cos(self.psi) * self.dt
            self.y    += self.v * np.sin(self.psi) * self.dt
            self.psi  += self.v / self.L * np.tan(delta) * self.dt
            self.psi   = self._normalize_angle(self.psi)
            self.v     = np.clip(self.v + a * self.dt, 0.0, self.max_speed)
            return

        # RK4 积分（v 在一步内视为常数，标准工程近似）
        state = np.array([self.x, self.y, self.psi, self.beta, self.r])
        v = self.v
        dt = self.dt

        k1 = self._derivatives(state,              delta, v)
        k2 = self._derivatives(state + dt/2 * k1,  delta, v)
        k3 = self._derivatives(state + dt/2 * k2,  delta, v)
        k4 = self._derivatives(state + dt    * k3,  delta, v)

        state_new = state + dt / 6 * (k1 + 2*k2 + 2*k3 + k4)

        self.x, self.y, self.psi, self.beta, self.r = state_new
        self.psi = self._normalize_angle(self.psi)
        self.v   = np.clip(self.v + a * self.dt, 0.0, self.max_speed)

    def _derivatives(self, state, delta, v):
        """计算状态导数，供 RK4 使用。state = [x, y, psi, beta, r]"""
        x, y, psi, beta, r = state
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr = self.Cf, self.Cr

        dx    = v * np.cos(psi + beta)
        dy    = v * np.sin(psi + beta)
        dpsi  = r
        dbeta = (-(Cf + Cr) / (m * v) * beta
                + (-(Cf * lf - Cr * lr) / (m * v**2) - 1.0) * r
                + Cf / (m * v) * delta)
        dr    = (-(Cf * lf - Cr * lr) / Iz * beta
                - (Cf * lf**2 + Cr * lr**2) / (Iz * v) * r
                + Cf * lf / Iz * delta)

        return np.array([dx, dy, dpsi, dbeta, dr])

    def get_state(self):
        """返回控制器兼容的状态量 [x, y, psi, v]"""
        return np.array([self.x, self.y, self.psi, self.v])

    def set_state(self, x, y, psi, v):
        self.x, self.y, self.psi, self.v = x, y, psi, v
        self.beta = 0.0
        self.r    = 0.0

    @staticmethod
    def _normalize_angle(angle):
        while angle >  np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle
```

**测试这个模型（在第二台电脑的 Ubuntu 终端运行）：**

```bash
cd ~/av_control_benchmark/python_prototype
python3 -c "
from models.dynamic_bicycle_model import DynamicBicycleModel
import numpy as np

car = DynamicBicycleModel()
car.set_state(0, 0, 0, 5.0)

# 模拟直行 1 秒
for _ in range(20):
    car.update(delta=0.0, a=0.0)

print(f'直行1秒后位置: x={car.x:.2f}m, y={car.y:.2f}m')
# 预期输出：x≈5.00m, y≈0.00m
"
```

---

### 2.4 实现轨迹生成器

创建文件 `python_prototype/utils/trajectory_generator.py`：

```python
import numpy as np

def generate_straight(length=100, step=1.0, speed=8.0):
    """生成直线轨迹"""
    x = np.arange(0, length, step)
    y = np.zeros_like(x)
    yaw = np.zeros_like(x)
    v = np.full_like(x, speed)
    return np.column_stack([x, y, yaw, v])

def generate_circle(radius=20, n_points=200, speed=5.0):
    """生成圆形轨迹"""
    theta = np.linspace(0, 2 * np.pi, n_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    yaw = theta + np.pi / 2
    v = np.full(n_points, speed)
    return np.column_stack([x, y, yaw, v])

def generate_s_curve(length=100, amplitude=10, speed=6.0):
    """生成 S 形弯道轨迹"""
    x = np.linspace(0, length, 300)
    y = amplitude * np.sin(2 * np.pi * x / length)
    dx = np.gradient(x)
    dy = np.gradient(y)
    yaw = np.arctan2(dy, dx)
    v = np.full_like(x, speed)
    return np.column_stack([x, y, yaw, v])

def generate_double_lane_change(speed=8.0):
    """
    生成 ISO 3888 双移线轨迹
    用于测试车辆极限操控性能，MPC vs 其他算法差异最明显
    """
    points = []
    for x in np.linspace(0, 30, 60):
        points.append([x, 0, 0, speed])
    for t in np.linspace(0, np.pi, 40):
        x = 30 + t / np.pi * 20
        y = 3.5 * (1 - np.cos(t)) / 2
        points.append([x, y, 0, speed])
    for x in np.linspace(50, 80, 30):
        points.append([x, 3.5, 0, speed])
    for t in np.linspace(0, np.pi, 40):
        x = 80 + t / np.pi * 20
        y = 3.5 - 3.5 * (1 - np.cos(t)) / 2
        points.append([x, y, 0, speed])
    for x in np.linspace(100, 130, 30):
        points.append([x, 0, 0, speed])

    traj = np.array(points)
    dx = np.gradient(traj[:, 0])
    dy = np.gradient(traj[:, 1])
    traj[:, 2] = np.arctan2(dy, dx)
    return traj
```

---

## 第三周：PID + Pure Pursuit 控制器

### 3.1 纵向 PID 控制器

**原理：**

PID 控制是最基础的控制器。纵向控制的目标是让车速跟踪参考速度。

```
误差 e = 目标速度 v_ref - 当前速度 v

加速度输出 a = Kp·e + Ki·∫e·dt + Kd·(de/dt)

Kp (比例)：误差越大，加速越猛
Ki (积分)：消除稳态误差（长期偏慢/偏快）
Kd (微分)：误差变化越快，越提前抑制（防超调）
```

创建文件 `python_prototype/controllers/pid_controller.py`：

```python
class PIDController:
    """纵向速度 PID 控制器"""

    def __init__(self, kp=1.0, ki=0.1, kd=0.05, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self._error_sum = 0.0
        self._last_error = 0.0

    def compute(self, v_ref, v_current):
        """
        v_ref:     目标速度 (m/s)
        v_current: 当前速度 (m/s)
        返回:      加速度指令 (m/s²)
        """
        error = v_ref - v_current

        self._error_sum += error * self.dt
        error_diff = (error - self._last_error) / self.dt

        output = self.kp * error + self.ki * self._error_sum + self.kd * error_diff

        self._last_error = error
        return output

    def reset(self):
        self._error_sum = 0.0
        self._last_error = 0.0
```

---

### 3.2 横向 Pure Pursuit 控制器

**原理：**

Pure Pursuit（纯追踪）是自动驾驶中最经典的路径跟踪算法。

核心思想：在参考轨迹上找一个"前视点"（lookahead point），计算让车头指向该点所需的转角。

```
前视距离 ld = k · v + ld_min
（速度越快，看得越远，越稳定）

转角公式：δ = arctan(2·L·sin(α) / ld)

其中：
α = 前视点方向 - 当前航向角
L = 轴距
```

创建文件 `python_prototype/controllers/pure_pursuit.py`：

```python
import numpy as np

class PurePursuitController:
    """横向 Pure Pursuit 路径跟踪控制器"""

    def __init__(self, wheelbase=2.7, k=0.1, ld_min=2.0):
        self.L = wheelbase
        self.k = k
        self.ld_min = ld_min

    def compute(self, state, trajectory):
        """
        state:      [x, y, theta, v]
        trajectory: N×4 数组，每行 [x, y, yaw, v_ref]
        返回:       (转角 delta, 目标速度 v_ref)
        """
        x, y, theta, v = state

        ld = max(self.k * v + self.ld_min, self.ld_min)

        traj_xy = trajectory[:, :2]
        dists = np.linalg.norm(traj_xy - np.array([x, y]), axis=1)
        nearest_idx = np.argmin(dists)

        target_idx = nearest_idx
        for i in range(nearest_idx, len(trajectory)):
            dist = np.linalg.norm(trajectory[i, :2] - np.array([x, y]))
            if dist >= ld:
                target_idx = i
                break

        tx, ty = trajectory[target_idx, :2]
        alpha = np.arctan2(ty - y, tx - x) - theta
        alpha = self._normalize_angle(alpha)

        delta = np.arctan2(2 * self.L * np.sin(alpha), ld)

        v_ref = trajectory[target_idx, 3]
        return delta, v_ref

    @staticmethod
    def _normalize_angle(angle):
        while angle >  np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle
```

---

## 第四周：接入 CARLA 仿真

前三周的控制器（PID + Pure Pursuit）已在动态自行车模型上验证，本周把它们接入 CARLA，让真实 3D 仿真中的车辆跟踪轨迹。

### 4.1 CARLA 环境桥接

**理解连接架构：**

```
Windows 上的 CARLA（仿真服务器）
        ↕ TCP localhost:2000
WSL2 Ubuntu 里的 Python（控制客户端）
  · 读取车辆状态 (x, y, psi, v)
  · 运行 PID + Pure Pursuit
  · 发送控制指令 (steer, throttle, brake)
```

**先创建目录和包文件：**

```bash
mkdir -p ~/av_control_benchmark/python_prototype/carla_bridge
touch ~/av_control_benchmark/python_prototype/carla_bridge/__init__.py
```

创建文件 `python_prototype/carla_bridge/carla_env.py`：

```python
import carla
import numpy as np


class CarlaEnv:
    """CARLA 环境桥接：负责连接、生成车辆、读状态、发控制指令"""

    # CARLA 最大方向盘转角（弧度），Tesla Model 3 约 70°
    MAX_STEER_ANGLE = np.radians(70)

    def __init__(self, host='localhost', port=2000, town='Town03'):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.load_world(town)

        # 关闭随机交通，方便调试
        settings = self.world.get_settings()
        settings.synchronous_mode = True   # 同步模式：每步需手动 tick
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

        self.vehicle = None
        self.history_x = []
        self.history_y = []

    def spawn_vehicle(self, spawn_index=0):
        """在指定起点生成 Tesla Model 3"""
        bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        spawn_point = self.world.get_map().get_spawn_points()[spawn_index]
        self.vehicle = self.world.spawn_actor(bp, spawn_point)
        self.vehicle.set_autopilot(False)
        # 让物理引擎稳定几帧
        for _ in range(10):
            self.world.tick()
        return spawn_point

    def get_state(self):
        """返回控制器需要的状态量 [x, y, psi, v]"""
        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()

        x   = tf.location.x
        y   = tf.location.y
        # CARLA yaw 单位为度，顺时针，转换成弧度逆时针（标准数学方向）
        psi = -np.radians(tf.rotation.yaw)
        v   = np.sqrt(vel.x**2 + vel.y**2)

        self.history_x.append(x)
        self.history_y.append(y)
        return np.array([x, y, psi, v])

    def apply_control(self, delta, accel):
        """
        delta: 前轮转角 (rad)，正值向左
        accel: 加速度 (m/s²)，正值加速，负值制动
        """
        steer = float(np.clip(delta / self.MAX_STEER_ANGLE, -1.0, 1.0))

        ctrl = carla.VehicleControl()
        ctrl.steer = steer
        if accel >= 0:
            ctrl.throttle = float(np.clip(accel / 3.0, 0.0, 1.0))
            ctrl.brake    = 0.0
        else:
            ctrl.throttle = 0.0
            ctrl.brake    = float(np.clip(-accel / 5.0, 0.0, 1.0))

        self.vehicle.apply_control(ctrl)
        self.world.tick()   # 推进一个仿真时间步

    def destroy(self):
        if self.vehicle:
            self.vehicle.destroy()
        # 恢复异步模式
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
```

---

### 4.2 生成 CARLA 参考轨迹

CARLA 的地图提供了路点（Waypoint）系统，直接从地图提取前方路点作为参考轨迹：

创建文件 `python_prototype/carla_bridge/waypoint_trajectory.py`：

```python
import carla
import numpy as np


def get_waypoint_trajectory(world, start_transform, length=150, step=2.0, speed=8.0):
    """
    从 CARLA 地图沿道路方向提取路点，生成参考轨迹
    返回 N×4 数组，每行 [x, y, yaw_rad, v_ref]
    """
    carla_map = world.get_map()
    wp = carla_map.get_waypoint(start_transform.location)

    points = []
    for _ in range(int(length / step)):
        loc = wp.transform.location
        # CARLA yaw 转标准弧度
        yaw = -np.radians(wp.transform.rotation.yaw)
        points.append([loc.x, loc.y, yaw, speed])

        nexts = wp.next(step)
        if not nexts:
            break
        wp = nexts[0]

    return np.array(points)
```

---

### 4.3 CARLA 完整仿真主程序

创建文件 `python_prototype/carla_bridge/run_carla_sim.py`：

```python
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# 把 python_prototype 加入路径，以便导入控制器
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from controllers.pid_controller import PIDController
from controllers.pure_pursuit import PurePursuitController
from carla_bridge.carla_env import CarlaEnv
from carla_bridge.waypoint_trajectory import get_waypoint_trajectory


def run():
    env = CarlaEnv(host='localhost', port=2000, town='Town03')

    try:
        print('生成车辆...')
        spawn_tf = env.spawn_vehicle(spawn_index=1)

        print('生成参考轨迹...')
        trajectory = get_waypoint_trajectory(env.world, spawn_tf, length=150, speed=8.0)

        pid         = PIDController(kp=1.5, ki=0.1, kd=0.05)
        pure_pursuit = PurePursuitController(wheelbase=2.87)  # Model 3 轴距

        lateral_errors = []
        print('开始仿真...')

        for step in range(600):   # 30 秒（0.05s × 600）
            state = env.get_state()
            delta, v_ref = pure_pursuit.compute(state, trajectory)
            accel        = pid.compute(v_ref, state[3])
            env.apply_control(delta, accel)

            # 计算横向误差（到轨迹最近点的距离）
            dists = np.linalg.norm(trajectory[:, :2] - state[:2], axis=1)
            lateral_errors.append(np.min(dists))

            if step % 50 == 0:
                print(f'  步骤 {step:3d}: v={state[3]:.1f} m/s  横向误差={lateral_errors[-1]:.3f} m')

    finally:
        env.destroy()

    # 绘制结果
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    axes[0].plot(trajectory[:, 0], trajectory[:, 1], 'b--', lw=2, label='参考轨迹')
    axes[0].plot(env.history_x, env.history_y, 'r-', lw=1.5, label='实际轨迹')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('CARLA 轨迹跟踪（Town03）')
    axes[0].legend()
    axes[0].set_aspect('equal')
    axes[0].grid(True, alpha=0.3)

    rmse = np.sqrt(np.mean(np.array(lateral_errors) ** 2))
    axes[1].plot(lateral_errors, 'g-', lw=1.2)
    axes[1].axhline(rmse, color='r', linestyle='--', label=f'RMSE: {rmse:.3f} m')
    axes[1].set_xlabel('时间步')
    axes[1].set_ylabel('横向误差 (m)')
    axes[1].set_title('横向跟踪误差')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs('results', exist_ok=True)
    plt.savefig('results/carla_pure_pursuit.png', dpi=150)
    print(f'\nRMSE: {rmse:.4f} m')
    print(f'最大误差: {max(lateral_errors):.4f} m')
    print('图表已保存到 results/carla_pure_pursuit.png')


if __name__ == '__main__':
    run()
```

**运行流程（每次跑仿真的步骤）：**

1. **第二台电脑 Windows** 上双击启动 `C:\CARLA_0.9.15\CarlaUE4.exe`，等到仿真窗口出现
2. **WSL2 终端**（VS Code 里）运行：

```bash
cd ~/av_control_benchmark/python_prototype
python3 carla_bridge/run_carla_sim.py
```

**预期产出：** CARLA 窗口中可以看到白色 Tesla 沿道路行驶，终端打印每步误差，结束后生成 `results/carla_pure_pursuit.png`。

---

## 第一阶段总结

完成本阶段后，你应该拥有：

| 产出 | 验证方式 |
|------|----------|
| 第二台电脑 WSL2 Ubuntu 22.04 | `lsb_release -a` 输出正确版本 |
| SSH 从当前电脑连接成功 | VS Code 左下角显示 `SSH: <IP>` |
| CARLA 安装并能启动 | 弹出城市仿真窗口 |
| WSL2 连接 CARLA 成功 | Python 打印出版本号 |
| 动态自行车模型（算法验证用） | 直行测试输出 x≈5.00m, y≈0.00m |
| PID 控制器 | 速度误差 < 0.2 m/s |
| Pure Pursuit 控制器 | S 弯 RMSE < 0.5m |
| CARLA 仿真跑通 | 车辆在 CARLA 中跟踪轨迹，RMSE < 1.0m |

**遇到问题时：**
- SSH 连接失败 → 检查第二台防火墙是否放行 22 端口
- `carla` 模块找不到 → 重新运行 `pip3 install carla==0.9.15`
- 连接 CARLA 超时 → 确认 `CarlaUE4.exe` 已启动且窗口已出现（等 Shader 编译完）
- 车辆失控 → 调小 Pure Pursuit 的 `k` 参数（前视增益）或调大 `ld_min`
- CARLA 坐标 y 方向反了 → 检查 `get_waypoint_trajectory` 里的 yaw 转换
