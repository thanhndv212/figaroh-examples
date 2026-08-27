# KITT1_5V3 右臂动力学辨识(MuJoCo 仿真优先)

本目录对 KITT1_5V3 双足双臂机器人的**右臂**(7-DOF)做动力学参数辨识,采用
**MuJoCo 仿真优先**方案:先从整机 URDF 提取右臂子模型,用 IPOPT 生成最优激励
轨迹,在 MuJoCo 中做逆动力学采样(叠加测量噪声),再用 FIGAROH 的 UR10 辨识模式
完成参数辨识。

整个流程复用了 `examples/ur10` 的 `identification.py` / `optimal_trajectory.py`
结构,并针对 KITT1_5 做了子模型提取与 MuJoCo 一致性处理。

**逐步操作（仿真 → 辨识 URDF → 实机数据契约与 AI 提示词）**见
[`docs/运行指南_仿真到实机.md`](docs/运行指南_仿真到实机.md)。

## 关键约定

- **末端执行器(tool0)= `right_flange_link`(flange)**:子模型保留到 flange,
  flange 之后的夹爪/相机一律剔除。
- **仅辨识右臂 7 个关节**:`right_arm_joint_1..7`。左臂、腿、头部、移动底盘
  均不参与。
- **子模型根**:`right_arm_joint`(固定关节,连接 `base_link` 与 `Base_R`)。
- **辨识数据来源**:`data/identification_q_simulation.csv` 与
  `data/identification_tau_simulation.csv`(MuJoCo 生成,带/不带噪声)。

## 目录结构

```
kitt_1_5/
├── extract_right_arm.py         # 从整机 URDF 提取右臂 7-DOF 子模型
├── urdf_to_mjcf.py              # URDF 子模型 -> MuJoCo MJCF(手动转换,保证动力学一致)
├── optimal_trajectory.py        # IPOPT 最优激励轨迹生成
├── generate_mujoco_data.py      # MuJoCo 逆动力学采样,生成 q/tau CSV + 真值 npz
├── plot_identification_data.py  # 绘制 q/dq/ddq/tau 曲线(噪声 vs 真值)
├── identification.py            # 动力学参数辨识(复用 UR10 模式)
├── update_model.py              # 将 soft-SDP 完整惯量写回 URDF
├── utils/
│   └── kitt_1_5_tools.py        # KITT1_5Identification / OptimalTrajectory 子类
├── config/
│   └── kitt_1_5_unified_config.yaml
├── urdf/
│   ├── right_arm_robot.urdf     # 右臂子模型(提取产物)
│   └── right_arm_robot.mjcf     # 右臂子模型 MJCF(urdf_to_mjcf.py 产物)
├── data/
│   ├── identification_q_simulation.csv
│   ├── identification_tau_simulation.csv
│   ├── identification_truth.npz # 无噪声真值(用于绘图/交叉验证)
│   ├── validation/              # 独立验证集(同一轨迹,不同噪声)
│   └── trajectories/right_arm_optimal_trajectory.npz  # IPOPT 轨迹(拼接)
└── results/
    ├── plots/                   # 数据可视化 PNG
    ├── runs/                    # identification.py 归档结果
    ├── urdf/                    # update_model.py 写出的辨识 URDF + SIP CSV
    └── trajectories/            # IPOPT 轨迹图/数据
```

## 运行流程

所有脚本都要求工作目录是 `examples/kitt_1_5/`,且使用 `figaroh-dev` conda 环境:

```bash
conda activate figaroh-dev
cd examples/kitt_1_5
```

### 0.(一次性)提取右臂子模型

从整机 KITT1_5V3 URDF 提取右臂,保留到 `right_flange_link`,并复制相关 mesh:

```bash
python extract_right_arm.py
```

脚本需要源整机 URDF(默认 `data/urdf/kitt_1_5_v3.urdf` 等路径,见脚本内常量)。
提取产物 `urdf/right_arm_robot.urdf` 与 mesh 已随本目录提供,通常无需重跑。

### 0b.(一次性)URDF -> MJCF 转换

MuJoCo 自带的 URDF 解析器会丢掉挂在**固定根关节**下的体的质量(这里即
`Base_R`),导致 `mj_inverse` 与 Pinocchio RNEA 不一致。因此手动转换:

```bash
python urdf_to_mjcf.py
```

转换要点:

- 每个 URDF link 保留为独立 `<body>`,`inertial` 使用 `fullinertia`(与 URDF
  相同的 `ixx iyy izz ixy ixz iyz` 约定)写入 body 坐标系惯量,避免
  `diaginertia + quat` 特征分解重建不保真。
- 关节 `axis` 直接沿用 URDF 值(子体 frame 即关节 frame),`pos="0 0 0"`。
- 生成的 MJCF 经校验:FK、质量矩阵、bias(重力+科氏+离心)与 Pinocchio 一致
  (误差 ~1e-9)。

### 1. 生成最优激励轨迹(IPOPT)

```bash
python optimal_trajectory.py
```

单段激励轨迹(验证用,数据量小),输出
`data/trajectories/right_arm_optimal_trajectory.npz`(拼接后的
`t/q/dq/ddq`,约 100 采样点)。若该文件缺失,`generate_mujoco_data.py` 会回退
到内置的 Fourier 激励轨迹(`--trajectory fourier`),不影响链路跑通。

> **验证阶段默认用 Fourier 轨迹**(见下方步骤 2 的 `--trajectory fourier`),
> 秒级生成。IPOPT 最优轨迹极慢的根因与修复建议见 [`issue.md`](issue.md)
> (ISSUE-001)。正式辨识再启用 IPOPT,并恢复更大的 `waypoints` /
> `segment_duration` / `max_iterations`。

### 2. 生成 MuJoCo 仿真数据

```bash
# 无噪声(校验用)
python generate_mujoco_data.py --noise-scale 0.0
# 带测量噪声(实际辨识用):q 噪声 1e-4 rad, tau 噪声 1e-2 N·m
python generate_mujoco_data.py --noise-scale 1.0
```

`run_inverse_dynamics` 对每个样本设置 `qpos/qvel/qacc` 后调用
`mj_inverse`(先 `mj_forward` 再设 `qacc`)。**加载模型后强制
`jnt_limited[:] = False`**:由于 URDF 中关节 4/6 的限位是不对称的窄限位
(`[-2.5307, 1.0472]`、`[-1.0472, 1.0472]`),激励轨迹可能轻微越界,若不禁用
限位,`mj_inverse` 会把**关节限位约束力**混入力矩,与辨识所用的理想 RNEA
模型不一致,导致拟合相关性极差。

输出:

- `data/identification_q_simulation.csv`(q, 含噪声)
- `data/identification_tau_simulation.csv`(τ, 含噪声)
- `data/identification_truth.npz`(无噪声真值 q/dq/ddq/tau + 噪声配置,绘图用)
- `data/validation/` 下同结构独立验证集(不同随机种子)

### 3. 数据可视化

```bash
python plot_identification_data.py
```

在 `results/plots/` 生成训练/验证两套 `q/dq/ddq/tau` 曲线(真值 vs 含噪采样)。

### 4. 动力学辨识

```bash
python identification.py
```

复用 UR10 的 `BaseIdentification.solve()` 流程:列消元 -> QR 分解求基参数 ->
最小二乘 -> 质量指标,并输出 HTML 报告与验证裁定(`--verify`)。

默认无噪声/带噪声两轮都通过:

| 指标 | 无噪声 | 带噪声(q 1e-4, τ 1e-2) |
| --- | --- | --- |
| 基参数数量 | 45 | 45 |
| Condition number | ~42 | ~42 |
| RMSE (N·m) | 0.082 | 0.116 |
| Correlation | 0.9999 | 0.9999 |
| 验证集 Correlation | 0.9999 | 0.9998 |

注意:`validation_improvement_pct` 的库默认阈值是 50%,但本场景数据由模型本身
生成,标称参数几乎就是真值,该指标天然只有几个百分点;本脚本 `verify()` 只把
这一项阈值放宽到 1%(其余项保持库默认),并在代码中注释了原因。

### 5. 写回辨识 URDF

将 soft-SDP 重建的完整标准参数写回子模型 URDF(`Joint1_R`..`Joint7_R`;
`Base_R` 在 Pinocchio 中并入 universe,保持原样):

```bash
python update_model.py
# 默认输出:
#   results/urdf/right_arm_robot_identified.urdf
#   results/urdf/full_parameters_identified.csv
```

脚本会打印名义 / 基参数 / SDP θ_r / 回读 URDF 四套力矩 RMSE。若还需 MJCF,
把输出 URDF 拷到 `urdf/right_arm_robot.urdf` 后重跑 `urdf_to_mjcf.py`,或按该
脚本内路径改一下再转。

## 已知问题与踩坑记录

1. **MuJoCo 限位约束力污染力矩**:见上文步骤 2。症状是辨识相关性暴跌到 ~0.5,
   `mj_inverse` 与 RNEA 在超限样本处相差几十到几百 N·m。
2. **`scipy.linalg.qr` 默认 `mode='full'` 内存爆炸**:84k 行回归器会分配约
   56 GB 的 Q 矩阵导致进程被 OOM(SIGKILL)。已把
   `figaroh/tools/qrdecomposition.py` 的 QR 调用改为 `mode='economic'`
   (改在 `../figaroh` 源码与已安装包两处)。
3. **`get_standard_parameters` 索引错位**:`figaroh/identification/parameter.py`
   中 `model.inertias` 是 1 索引(0 为固定基座),修复前按 `link_idx` 取惯性会把
   所有 link 的参数整体错位一位,导致 `W @ phi_std` 与 RNEA 不一致。
4. **`filter_type` 映射**:统一配置写 `butterworth`,而
   `DataProcessor.apply_filter` 需要 `lowpass`,在
   `KITT1_5Identification.load_trajectory_data` 中做了映射。
5. **环境退出时 `double free` / `SIGABRT`**:已知的 pinocchio/numpy 兼容问题,
   发生在脚本正常结束后的进程清理阶段,不影响结果。可用
   `PYTHONMALLOC=malloc` 规避。
6. **IPOPT 慢**:最优轨迹生成依赖 IPOPT(conda 的 `cyipopt`),可跑 5+ 分钟,
   `validate.py --quick` 会跳过该脚本。

## 验证

从仓库根目录:

```bash
python validate.py --robot kitt_1_5   # 该机器人全部测试+脚本
python validate.py --quick            # 跳过 IPOPT 慢脚本
```

## 复现示例

```bash
conda activate figaroh-dev
cd examples/kitt_1_5

# 1) 最优轨迹(约 5 分钟)
python optimal_trajectory.py

# 2) 生成带噪声训练/验证数据
python generate_mujoco_data.py --noise-scale 1.0

# 3) 可视化
python plot_identification_data.py

# 4) 辨识 + 验证裁定
python identification.py
```
