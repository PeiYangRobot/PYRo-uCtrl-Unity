# 六轴 VQF

`pyro::vqf_eskf_t` 将 VQF 近惯性系加速度低通与六维全局误差状态 EKF（ESKF）结合，提供陀螺积分、加速度可信度评估、倾斜校正、静止检测和三轴零偏估计。
这是本项目的定制融合实现，采用真实时间戳和固定存储。

## 公开接口

| 接口 | 用途 |
|---|---|
| `configure(config)` | 首次初始化，以及运行中统一修改配置 |
| `reset(bias, sigma)` | 重置全部状态，可指定初始零偏及标准差 |
| `update(data)` | 接收 `const imu_data_t &`，按时间戳合并处理两路新样本 |
| `get_state()` | 取得姿态、联合误差协方差、零偏、消费帧数、观测可信度及 KF 诊断的值快照 |

配置由 `vqf_eskf_t::config_t` 保存，包含六轴融合参数。更新间隔由输入时间戳决定。
成员注释标明单位。结果由 `vqf_eskf_t::state_t` 保存，内部低通与积分状态由实例管理。
公开方法统一为上表四项。静止调试使用快照中的 `rest_t / rest_gyr_lp / rest_acc_lp`。

## 基本使用

```cpp
#include "imu_base_drv.h"
#include "pyro_algo_vqf_eskf.h"

pyro::vqf_eskf_t attitude;
pyro::vqf_eskf_t::config_t config;

pyro::status_t attitude_init()
{
    config.tau_acc = 3.0f;
    return attitude.configure(config);
}

pyro::status_t attitude_update(const imu_data_t &data)
{
    return attitude.update(data);
}

// 控制循环读取一次快照。
// const auto state = attitude.get_state();
// state.quat / state.bias / state.rest_detected
```

`imu_data_t` 及两路样本类型使用本工程 `Component/IMU/imu_base_drv.h` 中的定义。
`gyro_sample_t::Gyro` 使用 rad/s，`acc_sample_t::Accel` 使用 m/s²，两者采用相同的右手坐标系。
输出 `quat` 按 `[w,x,y,z]` 存储，将传感器系向量旋转到 Z 向上的六轴参考系。
航向由初始条件确定。零偏和协方差分别使用 rad/s 与 (rad/s)²。
`bias_sigma` 为零偏标准差上界，单位 rad/s。

## 时间戳合并与异频批次

```cpp
attitude.configure(config);

// 驱动成功取到新数据后，直接交给 VQF。
imu.read();
const auto status = attitude.update(imu.read_imu_data());
const auto state = attitude.get_state();
imu.assume_gyro_point(state.consumed_gyr);
imu.assume_acc_point(state.consumed_acc);
// 根据 status 使用 state，剩余样本保留在两路流中。
(void)status;
```

VQF 在批首通过 `gyro.available()` 和 `acc.available()` 获取有效帧数，
用 `gyro.at(i) / acc.at(i)` 取样本（i=0 最旧，内部处理环形回绕），
两个游标按 timestamp 线性合并。
每次处理时间较早的样本；时间戳相同时先处理角速度，再处理加速度。
支持两路独立采样率、不同帧数、单路数据和空批次。

两路各自按采样时间排列，共用时间基准，各批按采样时间先后提交。
比较采用 uint32 模运算，支持计数器回绕，本批时间跨度应小于 2^31 tick。
每路独立保存上一帧时间戳，跨批次延续，更新周期为
`dt = uint32_t(timestamp - previous_timestamp) * data.timestamp_tick_s`。
陀螺积分、所有低通、静止计时、零偏过程噪声与测量权重均使用对应的实际周期。
各路首帧初始化低通并建立时间基准，该帧的积分/计时为零。
同一路重复时间戳的帧消费后跳过，批次返回 `PYRO_WARNING`。
倒退时间戳或间隔达到 `2^31` tick 的帧返回 `PYRO_PARAM_ERROR`，标记该帧已消费，保留其后的样本。
设备重启或时间戳计数器被清零后调用 `reset()` 建立新的时间基准。

`imu_data_t::timestamp_tick_s` 是每个 tick 的秒数，通用默认值为 `1e-6`。
IMU_Module 的 LSM6DSV16X 驱动根据数据手册 DS13510 第 9.52 节，在初始化时读取
有符号 `INTERNAL_FREQ_FINE`，设置
`timestamp_tick_s = 1 / (46080 * (1 + 0.0013 * FREQ_FINE))`。
其他数据源须填写自身时间戳单位。批次必须携带逐帧采样时间戳。
运行中修改 `timestamp_tick_s` 返回 `PYRO_PARAM_ERROR`，更换时间基准前调用 `reset()`。
原 VQF 配置项 `gyr_freq / acc_freq` 已移除，器件 ODR 配置继续由 IMU 驱动管理。
VQF 通过 `state.consumed_gyr / state.consumed_acc` 报告最近一次 update 的实际消费前缀。
调用方按这两个计数归还样本，避免重复处理已提交帧或移除尚未处理的尾部。

## 参数修改与重置

修改调用方持有的 config，再调用 `configure(config)`。
参数更新保留当前姿态和零偏，降低 `bias_clip` 会立即对已有零偏限幅。
`tau_acc` 或可信度判据变化时清除加速度低通历史，重新确认可信观测，当前姿态继续传播。
静止判据变化重启静止检测，运动零偏开关控制倾斜观测对零偏状态的修正。
`bias_clip` 同时参与陀螺静止判据，修改时也会重启静止检测。
`bias_sigma_init` 和 `attitude_sigma_init` 在 `reset()` 时设置先验协方差。
`bias_forgetting_time` 设置零偏随机游走强度，过程噪声在每个陀螺样本上按 dt 积分。

`reset()` 恢复单位四元数、零偏零值及配置的初始不确定度。
`reset(bias, sigma)` 可提供初始零偏，sigma=0 使用配置的初始不确定度。
输入非法配置或重置参数时，当前状态保留。

初始化前更新/重置返回 `PYRO_NOT_FOUND`。
非空流需要有效的样本缓冲，容量必须是大于等于 2 的 2 的幂，否则返回 `PYRO_PARAM_ERROR`。
非有限样本、非法 tick 单位或倒退时间戳同样返回 `PYRO_PARAM_ERROR`。
已初始化时，空批次返回 `PYRO_OK`，帧数为零的流允许传入空指针。
零加速度或低通向量无法确定方向时，该帧跳过校正，批次继续处理并最终返回 `PYRO_WARNING`。
零加速度会中断连续静止计时。静止确认需要陀螺仪与加速度计两路判据均满足。
其他错误结束批次，此前完成的样本及步骤保留。每次 update 开始时消费计数清零。
非法输入帧在提交前被拒绝，消费前缀停在该帧之前，调用方可修复输入后继续。
加速度帧推进低通历史后计入消费量。后续 KF 或误差注入失败时保留此前名义姿态及该次修正前的协方差，避免重试同一帧时重复推进低通。
WARNING 帧同样计入消费量，批次继续处理后续样本。
`state.initialized` 表示已完成可信重力方向初始化，启动默认连续确认 0.2 s。
`state.bias_diagnostics` 保留字段名，内容为最近一次联合误差滤波操作的诊断。
`state.acc_confidence / acc_nis / acc_rejected` 分别为最近样本可信度、最近倾斜观测 NIS、门控或恢复等待标记。
`state.acc_corrections / acc_rejections` 为实际倾斜观测提交数、受门控或恢复等待的加速度帧数。
同一实例的配置、更新与读取由调用方同步。

## 算法依赖

ESKF 使用现有 `kf_t<6,3,true>` 计算线性化误差状态，采用 Cholesky 求解、Joseph 协方差更新和数值检查。
名义四元数使用非线性旋转传播，观测模型在当前名义状态处线性化。
矩阵使用 PYRo Matrix，四元数运算使用 CMSIS-DSP。
`Algorithm/Butterworth` 独立提供 `configure / reset / process` 三个入口，
低通使用二阶连续 Butterworth 的状态转移，区间内输入按当前样本保持。
可变周期调用 `process(input, output, dt)`，支持零间隔初始化及长间隔。
启动阶段按实际 dt 计算时间加权均值，之后使用精确状态转移，周期相同时复用系数。
巴特沃斯模块统一为 `configure(tau) / reset() / process(input, output, dt)`，逐次显式提供周期。
低通系数和递推状态采用 double，输入输出采用 float。
所有模块使用固定存储。VQF 算法版权及 MIT 许可保留在本目录。

## 固定存储与块结构协方差传播

实例内部除滤波状态外还持有一组矩阵暂存（`_jacobian`/`_skew`/`_skew_sq`/`_rot`/
`_avg_rot`/`_avg_rot_t`/`_avg_rot_at`/`_q`/`_p_prop`/`_m`/`_m_t`/`_p00`/`_p01`/`_p10`/
`_p11`/`_t1`/`_t2`/`_t3`），供 `_update_gyr` 与 `_inject_error` 复用，避免每样本构造并
清零 `matrix_t` 临时量（原实现的 `memset` 占 11% CPU、约 4 KB/ms；该版 memset 是逐字节循环）。

误差状态转移与误差注入都是块结构，协方差传播不再构造 6×6 的 F/G，而是直接做 3×3 块运算：

- 传播 `F = [[I, M], [0, I]]`（`M = -A·dt`）：由 `P` 对称得 `X10 = X01ᵀ`，只需
  `X01 = P01 + M·P11`、`r00 = P00 + M·P10 + X01·Mᵀ`、`r11 = P11`、`r10 = X01ᵀ`，
  即 **3 次 3×3 乘法（81 MAC）**，通用路径是 2 次 6×6（432 MAC）加一次 6×6 转置。
- 注入 `G = [[J_l, 0], [0, I]]`：`(0,0) = J·P00·Jᵀ`、`(0,1) = J·P01`、
  `(1,0) = (J·P01)ᵀ`、`(1,1) = P11`，同样 3 次 3×3 乘法。

算好的 `F·P·Fᵀ` 交给 `kf_t::predict_covariance(x_pred, P_propagated, Q_d)`：
通用 KF 仍负责 **+Q_d、数值检查、对称化与方差下限、`result_t::PREDICTED` 提交语义**，
只是省掉两次 N×N 乘法（`predict(x_pred,F,Q_d)` 保持原样给其他使用者）。

不变量：`_q` 每拍由 `_update_gyr` 完整写入（含显式清零的右下 3×3 非对角块）；各块暂存
每次使用前都被完整写满，因此不需要初始化。矩阵运算的返回码全部会被检查——原实现走
`matrix_t::operator*`，失败被静默吞掉，会用残缺的 A/Aᵀ 继续传播协方差。

块结构改变了浮点分组，因此数值与通用路径**不逐位相同**：主机回归中只有
`motion_bias` 第 6 位有效数字和 `tilt 0.040563°→0.040582°` 有变化，其余逐字一致。

## 联合估计与观测门控

名义状态为四元数和三轴陀螺零偏。误差顺序为 `[δθ_global, δb_body]`，定义为 `R_true = Exp(δθ) R_nom`。
连续误差模型为 `δθ_dot = -R_nom δb - R_nom n_g`，`δb_dot = n_b`。
恒角速度区间使用 `R_start J_l(ω dt)` 作为平均旋转计算偏置耦合，偏置随机游走噪声采用该平均旋转的近似积分。
倾斜观测取朝向 +Z 的最短旋转向量的水平两维，NIS 门限按两维设置。
每次修正通过调整后的增益一起更新状态和 Joseph 协方差，随后向名义状态注入误差，并用 SO(3) 左雅可比重置误差协方差。
`error_covariance` 的前 3 维单位为 rad，后 3 维为 rad/s，交叉项单位为二者乘积。

加速度进入倾斜低通前，使用模长偏差、近惯性系高频残差能量和预测倾斜不确定度进行检查。
50 ms 一阶能量窗口跟踪高频扰动。预检查拒绝样本时清空倾斜低通历史，重新等待连续可信数据。
接受的低通观测根据可信度增大 R，使用两维 NIS 进一步门控。
为限制相邻低通样本重复提供的信息，R 乘以 `max(1, tau_acc / observation_interval)`。
静止陀螺观测对应使用 `max(1, 2 * rest_filter_tau / observation_interval)`。
上述相关时间缩放是保守的工程近似，包含陀螺积分误差的滤波观测统计需要通过标定和回放进一步验证。

陀螺按全部样本传播，倾斜和静止观测按 `acc_update_period` 限制频率，观测间隔使用实际累积时间。
`motion_bias_est_enabled` 允许可信倾斜观测同时修正零偏，静止阶段使用低通角速度观测。
IMU_Module 启用运动零偏估计，ESKF 观测最小间隔为 2 ms。1920 Hz 名义采样下按四个样本对齐，约为 480 Hz，实际频率由时间戳及门控状态决定。

| 新增参数 | 默认值 | 用途 |
|---|---|---|
| `attitude_sigma_init` | 30 ° | 初始姿态误差标准差，重力对齐后保留航向不确定度 |
| `gyro_noise_density` | 0.05 °/s/√Hz | 陀螺白噪声密度 |
| `acc_noise` | 0.3 m/s² | 加速度单轴噪声标准差，用于方向噪声和预检查 |
| `acc_norm_tolerance` | 1.0 m/s² | 模长偏差降权尺度，超过三倍时拒绝 |
| `acc_vibration_tolerance` | 1.0 m/s² | 近惯性系高频残差 RMS 降权尺度 |
| `acc_update_period` | 0.002 s | ESKF 观测最小间隔 |
| `acc_recovery_t` | 0.2 s | 启动和受扰后的可信确认时间 |
| `acc_nis_threshold` | 9.21034 | χ²(2) 的名义 99% 分位值 |

原 `bias_sigma_motion` 和 `bias_vertical_forgetting_factor` 随原三状态运动零偏观测模型一并移除。
新模型的运动零偏收敛由联合协方差、陀螺/偏置过程噪声、加速度观测噪声和可信度共同决定。
这些默认值为工程初始值，目标硬件噪声及振动条件的回放用于确定最终参数。

## 联调检查

产品通过 `imu_service_err_bus / imu_service_err_data / imu_service_err_timeout` 观察采集异常，
通过 `imu_service_vqf_us_max` 观察单批解算耗时。
`imu_service_acc_confidence / imu_service_acc_nis / imu_service_acc_rejected` 用于观察降权、门控和恢复等待。
`imu_service_attitude().valid` 在可信重力方向确认后置位。
未及时消费时，24 帧环形缓冲会覆盖最旧样本并计数。
时间戳提供丢帧区间的时长，区间运动按当前角速度样本保持来近似。
异频批次按共同时间轴提交，已经提交较新陀螺帧后到达的旧加速度帧需要由上层重新建立共同批次。
可变步长低通在 dt 改变时重新计算系数，高采样率实时性通过硬件测量确认。

纯六轴的绝对航向需要外部方向参考，持续单向加速的进一步分离需要运动约束或外部观测。
饱和与混叠在量程、采样及器件前端滤波层处理。

## 主机回放回归

在仓库根目录执行，需要 Python 3 与本机 gcc/g++，使用仓库内 CMSIS-DSP 标量实现，产物写入临时目录。

```sh
python <备份目录>/PYRo_tests/run_vqf_eskf.py   # 测试已移出发布包，见仓库外层 test/
```

覆盖平移加速、振动、冲击、倒置初始化、真实三轴转动、静止/运动零偏、带噪声动态、异频流、环形批次、时间戳回绕、非法参数及数值失败回滚。
可通过 `--baseline-dir` 指定保存旧版 `pyro_algo_vqf_eskf.h/.cpp` 的目录，运行相同的三项干扰回放作对比。
合成回放使用已知四元数轨迹生成理想传感器输入，并叠加给定噪声及扰动。真实硬件误差、时间同步及 CPU 最坏耗时由上板验证补充。

## 模型参考

近惯性系加速度预处理和静止检测参考 VQF，保留原版权及许可。[VQF 论文](https://arxiv.org/abs/2203.17024)

全局姿态误差动力学、误差注入和协方差重置采用 SO(3) 误差状态表达。[Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)
