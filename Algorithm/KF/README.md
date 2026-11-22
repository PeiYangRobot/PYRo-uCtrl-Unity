# 通用 KF 与独立 EKF / LKF / ESKF

依赖方向从应用指向算法，四元数模型属于 INS：

```text
Component/INS/pyro_ins.h/.cpp
  └─ pyro_ins_attitude.h/.cpp          IMU 预处理、四元数模型、门控、姿态输出
       └─ Algorithm/EKF/pyro_algo_ekf.h   ekf_t<N,Z>：非线性观测适配
            └─ Algorithm/KF/pyro_algo_kf.h/.tpp   kf_t<N,Z>：数值核心
                 └─ Algorithm/Matrix/pyro_algo_matrix.h/.tpp/.cpp   通用矩阵运算

任意线性应用
  └─ Algorithm/KF/pyro_algo_lkf.h   可选线性适配器
       └─ 相同数值核心

VQF + 误差状态滤波（本工程 IMU 产品使用）
  └─ Algorithm/VQF/pyro_algo_vqf_eskf.h/.cpp    姿态传播、倾斜/静止观测、块结构传播
       └─ Algorithm/KF/pyro_algo_eskf.h    eskf_t<N,Z>：误差状态注入适配
            └─ 相同数值核心
```

三个适配器的分工：

| 类 | 文件 | 相对核心新增 |
|---|---|---|
| `ekf_t<N,Z>` | `pyro_algo_ekf.h` | `correct(z, z_pred, H, R, …)`：先算残差 z−h(x) 再交给核心 |
| `lkf_t<N,Z,U>` | `pyro_algo_lkf.h` | `predict(F, Q)` / `predict(F, Q, B, u)` 与 `correct(z, H, R, …)` |
| `eskf_t<N,Z>` | `pyro_algo_eskf.h` | `correct(y, H, R, dx, …)` 多回传误差增量；`reset_after_inject(P)` |

`eskf_t` 与 `ekf_t` 的唯一区别是 `correct` 多一个 `dx` 出参：误差状态滤波要求把
δθ/δb 注入回名义状态，而协方差仍由核心按最终 K 更新，二者必须用同一份增量。
`reset_after_inject` 对应注入后的协方差替换（G·P·Gᵀ）。

核心仅提供 `init/reset/predict/correct` 四种操作及只读查询；没有 `*_from_model`、
外部 prepare/apply/reject，也不读取传感器或时钟。状态和矩阵使用固定尺寸的
`vector_t` / `matrix_t`，内部为行优先 float32 `std::array`，不分配堆内存。
N 为状态维度，Z 为最大观测维度。
KF 专用类型统一放在 `kf_t<N,Z,CheckNumerics>` 的类作用域：
`diagnostics_t` 为操作诊断，`result_t` 为操作结果，`covariance_update_t` 为协方差
修正策略，`solver_t` 为残差协方差求解策略。`options_t`、`correction_t`、`accept_t`
及矩阵/向量别名同样属于类作用域；EKF 与线性适配器继承这些公开类型。

```cpp
using filter_t = pyro::kf_t<6, 3>;
filter_t::options_t options;
options.solver = filter_t::solver_t::CHOLESKY;
options.covariance_update = filter_t::covariance_update_t::JOSEPH;
filter_t filter;
const filter_t::diagnostics_t &diagnostics = filter.get_diagnostics();
bool corrected = diagnostics.result == filter_t::result_t::CORRECTED;
```

矩阵存储与运算由 [Matrix 模块](../Matrix/README.md) 提供；KF 保留滤波公式、
协方差约束和提交语义，不再实现 CMSIS 描述符、求逆或分解封装。
矩阵支持 `F*x`、`F*P*F.transpose()+Q_d` 等表达式，并在编译期检查形状；
KF 求解路径继续使用工作区和带状态码接口。矩阵不再是数组别名，旧数组引用接口
通过 `as_array()` 适配，`P_prior` 指针使用 `covariance_t` 对象。
矩阵用 `F/H/Q_d/R/P/B/K/S`，向量用 `x/z/u/y`；`_pred` 表示预测值，
尾部 `_` 标识私有数学成员。完整含义、维度和单位保留在双语注释中。

`y` 称为**观测残差（新息） / Measurement residual (innovation)**，
即实际观测减预测观测：`y=z-z_pred`。`S` 是观测残差协方差；
`NIS=yᵀS⁻¹y` 表示归一化观测残差平方，标准名称为“归一化新息平方”。

默认关闭数值扫描，调用方保证输入有限、协方差合法及方差下限有效。
`CheckNumerics` 是编译期模板参数：关闭时 `if constexpr` 移除扫描代码及其分支，
不额外遍历 x/F/P/Q_d/H/R/K 或候选结果；缺测通过 `mask` 指定。
初始化状态、策略结果、矩阵形状和 CMSIS 求解错误仍检查。

```cpp
pyro::kf_t<6, 3> kf;            // 默认不扫描数值
pyro::ekf_t<6, 3, true> checked_ekf;      // 显式开启数值检查
pyro::lkf_t<2, 1, 1, true> checked_linear;
```

开启后检查 NaN/Inf、协方差近似对称/非负对角及结果溢出；
EKF/线性适配器还会将非有限原始观测自动作为缺测。
`matrix::is_finite` 保留为主动调用工具；矩阵运算和运算符不会自动扫描数值。

## 1. 通用数值接口

```cpp
#include "pyro_algo_kf.h"
using filter_t = pyro::kf_t<2, 1>;
filter_t filter;
filter.init({0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 1.0f});
filter_t::state_t x_pred = {0.0f, 0.1f}; // 外部状态模型的计算结果
filter_t::transition_t F = {1.0f, 0.001f, 0.0f, 1.0f};
filter_t::covariance_t Q_d = {1.0e-6f, 0.0f, 0.0f, 1.0e-3f};
filter_t::observation_t H = {0.0f, 1.0f};
filter_t::measurement_noise_t R = {0.01f};
filter_t::measurement_t y{};
if (filter.predict(x_pred, F, Q_d) == pyro::PYRO_OK)
{
    y = {0.2f - filter.get_state()[1]};
    filter.correct(y, H, R); // 融合观测残差 y / Fuse measurement residual y.
}
```

`predict(x_pred,F,Q_d)` 提交外部预测状态，计算 `P- = F*P*Fᵀ + Q_d`。
F 只用于误差传播；外部可采用线性、非线性或其他状态模型。
第四个可选参数 `P_prior` 是输入 P 的借用指针，可用于外部渐消，默认使用当前 P。

`predict_covariance(x_pred,P_propagated,Q_d)` 是同一提交语义的变体：调用方按状态转移的
结构自己算出 `F*P*Fᵀ`（例如 F 为块三角、可用若干次小矩阵乘法），核心只负责 `+Q_d`、
数值检查、对称化/方差下限与提交。语义与 `predict(...,F,Q_d)` 一致（含 `PREDICTED`
结果码），只是省掉通用的两次 N×N 乘法。INS 的误差状态模型即用此入口传播协方差。

`correct(y,H,R)` 直接融合观测残差。线性模型 `y=z-H*x`，非线性模型 `y=z-h(x)`；
角度环绕等残差适配同样由外部完成。缺测使用 `measurement_mask_t`，false 行不参与
计算，对应 H 行/R 行列无需有效数据。未提供 mask 时启用全部行，有效 y/H/R 必须有限。
核心不将 NaN 残差自动作为缺测；关闭检查时不检测它，开启时报告输入错误。
Q_d 是状态空间离散过程协方差；调用方完成 `G*Qw*Gᵀ` 映射和时间离散化，核心不隐式乘 dt。

## 2. 独立 EKF

```cpp
#include "pyro_algo_ekf.h"
pyro::ekf_t<1, 1> ekf;
ekf.init({2.0f}, {1.0f});
// f(x)=x²，F=2x；h(x)=x²，H=2x，在各自阶段线性化。
if (ekf.predict({4.0f}, {4.0f}, {0.01f}) == pyro::PYRO_OK)
    ekf.correct({17.0f}, {16.0f}, {8.0f}, {0.1f});
// 参数依次为 z、z_pred=h(x)、H、R。
```

`ekf_t<N,Z>` 不含 IMU、四元数、角度或重力常量。它沿用通用 `predict` 接口；
`correct(z,z_pred,H,R)` 构造 `y=z-z_pred` 后调用相同核心。
F/H/Q_d/R 每次传入，可实时变化。默认仅按 mask 排除缺测；开启检查时非有限原始 z
自动排除，并检查 z_pred 和残差溢出。状态函数 f 和观测函数 h 的计算由外部模型负责。

## 3. 可选线性适配器

```cpp
#include "pyro_algo_lkf.h"
pyro::lkf_t<2, 1, 1> kf; // N=2，Z=1，U=1
kf.init({0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 1.0f});
if (kf.predict({1, 0.001f, 0, 1}, {1e-6f, 0, 0, 1e-3f},
               {0.0000005f, 0.001f}, {0.3f}) == pyro::PYRO_OK)
    kf.correct({0.0f}, {0, 1}, {0.01f});
```

适配器负责 `x_pred=F*x+B*u` 和 `y=z-H*x`，随后调用统一核心。
U=0 用 `predict(F,Q_d)`，U>0 可用 `predict(F,Q_d,B,u)`。
零量测合法；默认使用 mask 表达缺测，开启检查时非有限原始 z 自动作为缺测。
适配器不增加另一套状态、协方差或求解实现。

## 4. 单次修正中的通用策略

```cpp
filter.correct(y, H, R, nullptr, [](auto &correction)
{
    if (correction.diagnostics.nis > 9.0f)
        return pyro::PYRO_WARNING; // 拒绝量测，保留当前 x/P
    for (float &value : correction.K)
        value *= 0.8f;
    return pyro::PYRO_OK; // 核心用调整后的 K 计算 dx=K*y 和协方差
});
```

EKF/线性适配器传递相同策略，区别仅在观测参数。`correction_t` 借用本次
y/S/NIS/K/dx，仅在调用期间有效，不得重入该实例。返回 PYRO_OK 接受、
PYRO_WARNING 拒绝，其他值报告 POLICY_FAILURE。输入/求解失败或无观测时不调用策略。
准备、决策和提交在同一调用中结束，没有外部待提交状态，也没有旧 User_Func/SkipEq 链。
高级模型可填充 `correction.dx` 并置 `use_dx=true` 覆盖增量；P 仍由 K 计算，限幅/投影
与协方差的一致性由模型负责。INS 零偏限幅及 qz 增量策略保留在应用层。

## 5. 数值策略与失败语义

默认 Cholesky 解 S、Joseph 更新 P、对称化 P；`options_t::min_variance` 在
初始化/重置、预测和修正后施加方差下限。开启 CheckNumerics 时，Q_d/P0/R 检查
有限性、近似对称和非负对角，不证明完整半正定性；默认由调用方保证这些约束。
求逆使用 S 副本，公共 S/K/y 保持原观测索引，缺测对应元素为零。

| 结果 | 返回值 / result | x/P 行为 |
|---|---|---|
| 预测成功 | PYRO_OK / PREDICTED | 提交预测 |
| 修正成功 | PYRO_OK / CORRECTED | 提交修正 |
| 无有效观测 | PYRO_WARNING / NO_MEASUREMENT | 保留当前值 |
| 策略拒绝 | PYRO_WARNING / REJECTED | 保留当前值 |
| 策略失败 | 策略返回码 / POLICY_FAILURE | 保留当前值 |
| 输入无效 | PYRO_PARAM_ERROR / INVALID_INPUT | 保留当前值 |
| CMSIS 失败 | PYRO_ERROR / MATH_FAILURE | 保留当前值，诊断含 math_status |
| 运算溢出（开启数值检查时） | PYRO_ERROR / NON_FINITE_RESULT | 不提交候选值 |

预测失败保留上一 x/P；修正失败/拒绝保留已完成的预测。
重复 init 返回 PYRO_ALREADY_INIT；`reset(x0,P0)` 重新播种并保留 options。
同一预测后可依次融合独立量测，相关观测需在 R 中体现；避免重复融合。
同一实例并发访问由调用方同步，不同实例状态和工作区独立。

## 6. INS 接入与验证

姿态模型在 `Component/INS/pyro_ins_attitude.h/.cpp`。`ins_drv_t` 持有
`ins_attitude_t`，后者使用通用 `ekf_t<6,3>`，无全局姿态实例。
已移除算法层 QuaternionEKF/pyro_algo_quaternion_ekf 文件和旧全局入口。
IMU 预处理、四元数积分/归一化、重力观测、渐消、门控、增益调整、零偏限幅和欧拉角输出
均属于 INS。为保持调参行为，应用显式选择 INVERSE、SIMPLIFIED、不对称化；
保留归一化前构造 F 耦合、归一化后构造 H 的时序。
异常陀螺/dt 保持姿态，异常/零加速度仅预测且不污染低通历史；求解失败保留预测并可恢复。
infantry1 前向滤波器使用 `lkf_t<4,2>`，按 KF_ok 实测版本融合 IMU 前向加速度和轮速，
保留 INVERSE、Joseph、方差下限和连续滑移权重。每拍先 predict，再由预测 x/P
计算单条轮速 NIS 和滑移评分，调整本拍 Rwheel 后 correct。失效轮速使用 mask，
合法零轮速正常参与；轮速失效或评分达到 0.2 时，Policy 将 K 的零偏行清零，
状态和 Joseph 协方差使用相同增益。模型加速度不参与观测或滑移评分。
底盘模型、IMU 输入映射及控制时序保持原设计。
旧 kalman_filter.cpp/.h 已移至 tests/forward_kf/reference，仅作为冻结参考；
无调用的 pyro_kalman_filter.cpp/.h 已删除。Infantry2 仍使用 kf.cpp/.h，暂时保留。

```powershell
cmake --build build/forward-kf-tests --parallel 4
ctest --test-dir build/forward-kf-tests --output-on-failure
cmake --build cmake-build-debug-stm32 --parallel 4
```

测试覆盖独立 double 参考、动态矩阵/控制、相关 R、缺测、非线性观测残差、策略拒绝/失败、
增益调整、增量覆盖、数值失败/恢复、无堆分配、多实例、重置及前向 KF/底盘回归。
另验证默认无扫描路径与开启检查路径的一致性，以及 -O3、unsafe-math、finite-math
下显式开启的异常输入保护。INS 保留传感器边界检查，不在通用 KF 中重复扫描矩阵。
两组姿态轨迹各 20,000 拍，比较 q、bias、角度、6 维状态及 6×6 P，共 52 个 float32/拍。
旧 QEKF 和前向滤波器仅作为 tests/forward_kf/reference 冻结参考，不链接进固件；
前向 normal/stress 各 30,000 拍对照 x/v/a/bias/slip/P/K/R，并验证独立 double 参考、
轮速失效及恢复、O3/unsafe-math/finite-math 和底盘控制接线。
前向基线为 KF_ok / 19ea989，参数单独冻结；差异和复现见 docs/forward_kf_kf_ok_alignment.md。
最初三量测迁移的历史记录保存在 docs/forward_kf_migration.md。位级对照仅适用于
相同主机编译器和浮点选项。尚未刷机或测量板上执行时间、任务栈高水位与动态闭环。
