# Issues — KITT1_5V3 右臂动力学辨识

按发现时间倒序记录。状态: `open` / `workaround` / `fixed`。

---

## ISSUE-001: IPOPT 最优激励轨迹生成极慢

| 字段 | 内容 |
|---|---|
| 状态 | **open** (验证阶段用 Fourier 轨迹绕过) |
| 发现日期 | 2026-08-27 |
| 影响脚本 | `optimal_trajectory.py` |
| 严重度 | 高(阻塞正式激励轨迹生成;验证阶段可绕过) |

### 现象

`python optimal_trajectory.py` 单段优化跑满 200 次迭代约需 **15 分钟**,即便缩减规模
后(`waypoints=5`、`segment_duration=1.0`、`max_iterations=60`)仍需数分钟。日志频繁出现
`Joint vel idx_v * limits violated!` 与
`Could not find feasible initial trajectory after 500 attempts`。

对比:仓库里 UR10 的 `identification.py` 几分钟即可完成,但 UR10 **从未真正跑过**
`optimal_trajectory.py`(其 `results/` 无轨迹产物),因此"UR10 很快"是把辨识与 IPOPT
轨迹生成两个环节混淆了。

### 已排除的误判

| 假设 | 结论 |
|---|---|
| 是否误把整机无关关节(左臂/腿/头)塞进求解? | **否**。子模型实测 `nq=7, nv=7`,碰撞对=0;IPOPT 变量数 28 = (5−1)×7,与右臂 7 关节完全吻合 |
| 是否 URDF mesh/几何过多导致慢? | **否**。`pinocchio.computeJointTorqueRegressor` 单次 < 0.01 ms,400 样本循环也只是毫秒级 |
| 是否可以用 pytorch + RTX 3090 GPU 加速? | **收益有限**。瓶颈不在矩阵算力,而在数值差分的评估次数;IPOPT 内点法本身严格串行 |

### 根因(实测)

主求解器是 **IPOPT**(通过 `cyipopt`,C++ 内点法),不是 numpy 求解器。numpy / pinocchio
只负责每次目标/约束评估里的矩阵运算。

figaroh 未提供解析梯度,默认走 `numdifftools` 数值差分
(`figaroh/tools/robotipopt.py::IPOPTProblem.gradient` / `.jacobian`):

```text
每次 IPOPT 迭代成本 ≈
  2 × n_var 次 objective  (中心差分梯度)
+ 1 × n_var 次 constraint (前向差分 Jacobian)
```

缩小后的实测(Ns=400, n_var=28):

| 环节 | 耗时 |
|---|---|
| 单次 `computeJointTorqueRegressor` | < 0.01 ms |
| 单次 objective(样条 + 回归器 + QR + SVD cond) | **39 ms** |
| 单次 constraint | **39 ms** |
| **一次完整 IPOPT 迭代**(56×obj + 28×con) | **~3.2 s** |
| 满 200 次迭代 | ~11–15 min(与实测 892 s 吻合) |

次要因素:KITT 右臂末端力矩余量小(18 Nm)、关节行程窄且不对称(尤其关节 4/6),
随机初始点 500 次几乎全部违反速度/力矩约束,IPOPT 要从不可行点做可行性恢复,
进一步拉长迭代。

### 临时绕过(验证阶段)

`generate_mujoco_data.py --trajectory fourier` 用内置 Fourier 激励轨迹,秒级生成,
足够跑通「仿真采样 → 可视化 → 辨识 → 验证」全链路。IPOPT 轨迹留到正式实验再启用。

### 建议的正式修复(按收益排序)

1. **解析/自动微分梯度替代 `numdifftools`**(预期 20–30×)
   - 基参数列选择 `idx_b` 是机器人结构量,对轨迹不变,可预计算固定列选择;
   - 目标 `κ(W_b)` = `cond(W[:, idx_b])`,可用 SVD 对矩阵条目的解析导数反传到
     样条路点,或用 JAX/CasADi 对整条 `X → κ` 做 AD;
   - 接入点:`BaseTrajectoryIPOPTProblem.gradient` / `.jacobian`
     (覆盖 `robotipopt.IPOPTProblem` 默认实现)。
2. **并行数值差分**(短期,约线性加速):56 次独立 objective 评估可用
   `multiprocessing` / `joblib` 并行。
3. **降低采样密度**:验证阶段 `freq` 从 100 Hz 降到 50/25 Hz,每次评估成本近似线性下降。
4. **放宽初始可行域**:增大 `soft_lim`、或放宽腕部力矩上限做初始采样,提高
   `_generate_feasible_initial_guess` 命中率,减少可行性恢复迭代。
5. **不建议优先做 GPU**:单次评估矩阵太小(400×7×70),H2D 搬运开销抵消计算收益;
   且 pinocchio 无 GPU/批量 API,要自写 torch 批量 RNEA,工程量大。

### 相关修改记录

- 配置已为验证缩小:`waypoints=5`、`segment_duration=1.0`、`max_iterations=60`、
  `sampling_frequency=100 Hz`(见 `config/kitt_1_5_unified_config.yaml`)。
- `optimal_trajectory.py` 已去掉对有 bug 的 `traj.save_results()` 调用
  (`CubicSpline` 无 `identif_config` 属性 → `AttributeError`),改为只写
  `data/trajectories/right_arm_optimal_trajectory.npz`。
- 库侧补丁(site-packages,待回灌到 `../figaroh` 源码):
  - `figaroh/optimal/config.py`: `trajectory_config` 支持 `max_iterations`;
  - `figaroh/optimal/base_optimal_trajectory.py`: 从 `trajectory_config` 读
    `max_iterations`。

### 复现

```bash
conda activate figaroh-dev   # 或 figaroh-examples
cd examples/kitt_1_5
PYTHONMALLOC=malloc MPLBACKEND=Agg python -u optimal_trajectory.py
```
