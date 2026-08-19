# Git 使用指南

## 核心概念

```
你的电脑（本地）                      远端（GitHub）
┌────────────────────┐               ┌──────────────┐
│  工作区             │  git push ──→  │              │
│  （你写代码的地方）   │               │  GitHub 仓库  │
│        ↓ add       │  ←── git pull  │              │
│  暂存区             │               └──────────────┘
│  （选好要存档的文件） │
│        ↓ commit    │
│  本地仓库           │
│  （本地存档历史）    │
└────────────────────┘
```

**三个区域：**
- **工作区**：你正在编辑的文件
- **暂存区**：选好"这次要存档哪些文件"
- **本地仓库**：正式存档，保存完整历史记录

---

## 第一次使用前的配置

在每台机器上只需配置一次：

```bash
git config --global user.name "你的名字"
git config --global user.email "你的GitHub邮箱"
```

---

## 基本命令速查

### 初始化 / 克隆

```bash
# 全新项目，在项目目录内初始化
git init

# 从 GitHub 把仓库下载到本地
git clone https://github.com/用户名/仓库名.git
```

### 查看状态（随时可敲，不会改变任何东西）

```bash
git status
```

输出说明：
- `Untracked files` — 新文件，Git 还不认识它
- `Changes not staged` — 已有文件被修改，但还没 add
- `Changes to be committed` — 已 add，等待 commit

### 暂存文件

```bash
# 暂存单个文件
git add 文件名

# 暂存所有改动（最常用）
git add .
```

### 提交（本地存档）

```bash
git commit -m "说明这次改了什么"

# 示例
git commit -m "实现 PID 控制器基本框架"
git commit -m "修复 Pure Pursuit 预瞄距离计算错误"
```

> commit 信息写清楚，以后看历史记录时能一眼知道每次做了什么。

### 推送到 GitHub

```bash
# 第一次推送（需要指定远端分支）
git push -u origin main

# 之后每次直接
git push
```

### 从 GitHub 拉取最新代码

```bash
git pull
```

### 查看历史记录

```bash
# 简洁模式，每次提交一行
git log --oneline

# 查看某次提交的具体改动
git show 提交ID
```

### 查看改动内容

```bash
# 查看工作区和暂存区的差异（还没 add 的改动）
git diff

# 查看已 add 但还没 commit 的改动
git diff --staged
```

---

## 本项目的标准工作流

### 双设备协作示意

```
Windows 主机（写代码）
  │
  │  git push
  ▼
GitHub 仓库
  │
  │  git pull
  ▼
Ubuntu 机器（编译 + 运行 CARLA + ROS2）
  │
  │  运行结果（CSV / PNG）git push 回来
  ▼
Windows 主机（查看图表 / 分析结果）
```

### Windows 上写完代码后

```bash
git status              # 先确认改了哪些文件
git add .               # 暂存所有改动
git commit -m "说明"    # 本地存档
git push                # 推到 GitHub
```

### Ubuntu 上拉取并运行

```bash
git pull                # 拉取最新代码
# 然后编译运行
```

### Ubuntu 上产出结果后

```bash
git add results/        # 暂存结果文件
git commit -m "添加 PID benchmark 结果数据"
git push                # 推回 GitHub
```

---

## 常用 commit 信息参考格式

```
feat: 新增 Stanley 控制器实现
fix:  修复 LQR 矩阵维度不匹配问题
test: 添加 MPC 单元测试
data: 更新 benchmark 结果数据
docs: 补充 README 安装说明
```

---

## 常见问题

**Q：push 时报错 "rejected"**
```bash
# 先 pull 把远端最新代码合并进来，再 push
git pull
git push
```

**Q：不小心 add 了不想要的文件**
```bash
# 从暂存区移除（文件本身不受影响）
git restore --staged 文件名
```

**Q：想撤销最近一次 commit（还没 push）**
```bash
# 撤销 commit，但保留文件改动
git reset --soft HEAD~1
```

**Q：想看某个文件是谁改的、什么时候改的**
```bash
git log --oneline 文件名
```

**Q：.gitignore 是什么**

`.gitignore` 文件里列出不想被 Git 追踪的文件，比如编译产物、系统文件：

```
# 示例 .gitignore
build/
*.o
*.pyc
__pycache__/
.vscode/
```
