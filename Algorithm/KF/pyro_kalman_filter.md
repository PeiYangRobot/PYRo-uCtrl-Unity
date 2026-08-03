# PYRo 卡尔曼滤波器 (`kf_t`) 技术说明文档

> **目标平台**: STM32H723 (Cortex-M7, FPv5 硬浮点)  
> **依赖库**: ARM CMSIS-DSP (C API + dsppp C++ 扩展)  
> **命名空间**: `pyro`  
> **源文件**: `kf.h` / `kf.cpp`

---

## 目录

1. [卡尔曼滤波器数学原理](#1-卡尔曼滤波器数学原理)  
   1.1 [离散线性系统模型](#11-离散线性系统模型)  
   1.2 [预测步骤 (Predict)](#12-预测步骤-predict)  
   1.3 [更新步骤 (Update)](#13-更新步骤-update)  
   1.4 [Joseph 形式协方差更新](#14-joseph-形式协方差更新)  
2. [CMSIS-DSP 与 dsppp C++ 扩展](#2-cmsis-dsp-与-dsppp-c-扩展)  
   2.1 [dsppp 是什么](#21-dsppp-是什么)  
   2.2 [本项目中的关键用法](#22-本项目中的关键用法)  
   2.3 [dsppp 的限制与规避](#23-dsppp-的限制与规避)  
3. [类设计概览](#3-类设计概览)  
   3.1 [类型别名](#31-类型别名)  
   3.2 [构造函数与内存策略](#32-构造函数与内存策略)  
   3.3 [成员变量一览](#33-成员变量一览)  
4. [公有 API 详解](#4-公有-api-详解)  
   4.1 [构造函数](#41-构造函数)  
   4.2 [init() 四重载](#42-init-四重载)  
   4.3 [update()](#43-update)  
5. [私有辅助函数详解](#5-私有辅助函数详解)  
6. [update() 逐步执行流程](#6-update-逐步执行流程)  
7. [初始化流程 (init_impl)](#7-初始化流程-init_impl)  
8. [P₀ 运行时校验](#8-p₀-运行时校验)  
9. [使用示例](#9-使用示例)  
10. [设计考量与工程细节](#10-设计考量与工程细节)  

---

## 1. 卡尔曼滤波器数学原理

### 1.1 离散线性系统模型

标准离散线性时不变 (LTI) 系统可描述为：

$$
\begin{aligned}
\mathbf{x}_k &= \mathbf{A}\,\mathbf{x}_{k-1} + \mathbf{B}\,\mathbf{u}_{k-1} + \mathbf{G}\,\mathbf{w}_{k-1} \\
\mathbf{z}_k &= \mathbf{H}\,\mathbf{x}_k + \mathbf{v}_k
\end{aligned}
$$

| 符号 | 含义 | 维度 |
|------|------|------|
| $\mathbf{x}_k$ | 状态向量 | $n \times 1$ |
| $\mathbf{u}_k$ | 控制输入向量 | $m \times 1$ |
| $\mathbf{z}_k$ | 观测（量测）向量 | $p \times 1$ |
| $\mathbf{w}_k$ | 过程噪声向量，$\mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q})$ | $w \times 1$ |
| $\mathbf{v}_k$ | 量测噪声，$\mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R})$ | $p \times 1$ |
| $\mathbf{A}$ | 状态转移矩阵 | $n \times n$ |
| $\mathbf{B}$ | 控制输入矩阵 | $n \times m$ |
| $\mathbf{H}$ | 观测矩阵 | $p \times n$ |
| $\mathbf{G}$ | 噪声传递矩阵（噪声输入矩阵） | $n \times w$ |
| $\mathbf{Q}$ | 过程噪声协方差 | $w \times w$ |
| $\mathbf{R}$ | 量测噪声协方差 | $p \times p$ |

代码中的维度参数对应：

- `x_size` → $n$ (状态维度)
- `u_size` → $m$ (控制维度)
- `z_size` → $p$ (量测维度)
- `w_size` → $w$ (过程噪声维度)

### 1.2 预测步骤 (Predict)

**先验状态估计**：将上一时刻的后验估计通过系统模型向前推演。

$$
\hat{\mathbf{x}}_k^- = \mathbf{A}\,\hat{\mathbf{x}}_{k-1} + \mathbf{B}\,\mathbf{u}_{k-1}
$$

- 对应代码变量：`_vec_xhat_minus = A * _vec_xhat + B * _vec_u`

**先验误差协方差**：描述先验估计值的不确定性。噪声传递矩阵 $\mathbf{G}$ 将过程噪声从噪声空间 $\mathbb{R}^w$ 映射到状态空间 $\mathbb{R}^n$。

$$
\mathbf{P}_k^- = \mathbf{A}\,\mathbf{P}_{k-1}\,\mathbf{A}^T + \mathbf{G}\,\mathbf{Q}\,\mathbf{G}^T
$$

- 对应代码变量：`_mat_P_minus = A * P * A^T + G * Q * G^T`

> **为何引入 G？** 在许多实际系统中，过程噪声维度 $w$ 与状态维度 $n$ 不同。例如对于位置-速度系统 ($n=2$)，噪声可能只有加速度噪声 ($w=1$)，此时 $\mathbf{G} = [\Delta t^2/2;\; \Delta t]$（$2 \times 1$），$\mathbf{Q} = [\sigma_a^2]$（$1 \times 1$）。若令 $\mathbf{G} = \mathbf{I}_n$，则退化为旧公式 $\mathbf{P}_k^- = \mathbf{A}\mathbf{P}_{k-1}\mathbf{A}^T + \mathbf{Q}$。

### 1.3 更新步骤 (Update)

**新息协方差**（Innovation Covariance）：将先验不确定性投影到量测空间并加上量测噪声。

$$
\mathbf{S}_k = \mathbf{H}\,\mathbf{P}_k^-\,\mathbf{H}^T + \mathbf{R}
$$

**卡尔曼增益**：最优加权因子，平衡模型预测与量测信息的可信度。

$$
\mathbf{K}_k = \mathbf{P}_k^-\,\mathbf{H}^T\,\mathbf{S}_k^{-1}
$$

**后验状态估计**：融合量测残差（新息）修正先验估计。

$$
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k\,(\mathbf{z}_k - \mathbf{H}\,\hat{\mathbf{x}}_k^-)
$$

其中 $(\mathbf{z}_k - \mathbf{H}\,\hat{\mathbf{x}}_k^-)$ 即为**量测残差 / 新息**（innovation）。

### 1.4 Joseph 形式协方差更新

经典简化更新公式为：

$$
\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k\,\mathbf{H})\,\mathbf{P}_k^-
$$

该公式虽然简洁，但在浮点运算环境下容易因数值误差导致 $\mathbf{P}_k$ 丢失**对称性**和**正半定性**（positive semi-definiteness）。当 $\mathbf{P}_k$ 失去正半定性时，后续迭代将产生不物理的负方差，滤波器可能发散。

本代码采用 **Joseph 形式**（Joseph Stabilized Form）：

$$
\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k\,\mathbf{H})\,\mathbf{P}_k^-\,(\mathbf{I} - \mathbf{K}_k\,\mathbf{H})^T + \mathbf{K}_k\,\mathbf{R}\,\mathbf{K}_k^T
$$

令 $\mathbf{A}_j = \mathbf{I} - \mathbf{K}_k\,\mathbf{H}$，则可改写为：

$$
\mathbf{P}_k = \mathbf{A}_j\,\mathbf{P}_k^-\,\mathbf{A}_j^T + \mathbf{K}_k\,\mathbf{R}\,\mathbf{K}_k^T
$$

**优势**：

- $\mathbf{A}_j\,\mathbf{P}_k^-\,\mathbf{A}_j^T$ 结构自然保持对称性（$\mathbf{M}\mathbf{M}^T$ 恒对称、正半定）
- $\mathbf{K}_k\,\mathbf{R}\,\mathbf{K}_k^T$ 同理
- 两个正半定矩阵之和仍为正半定
- 即使 $\mathbf{K}_k$ 因数值不精确偏离最优值，Joseph 形式也能保证 $\mathbf{P}_k$ 的数学性质

**代价**：相较简化形式多消耗若干矩阵乘法和转置，但对嵌入式控制系统而言更鲁棒。

---

## 2. CMSIS-DSP 与 dsppp C++ 扩展

### 2.1 dsppp 是什么

CMSIS-DSP 库提供了经过 SIMD / 汇编优化的 C 语言矩阵运算 API（如 `arm_mat_mult_f32`, `arm_mat_inverse_f32`）。在此基础上，ARM 官方还提供了一套 **C++ 模板扩展库**，内部代号 **dsppp**（位于 `dsppp/` 头文件目录），其核心目标是：

- **表达式模板**（Expression Templates）：利用 C++ 模板元编程延迟矩阵表达式求值，将链式运算编译期合并，减少中间临时对象的堆分配
- **运算符重载**：可像 MATLAB / Eigen 般书写 `C = A * B + Q`，编译器自动分派到底层 CMSIS-DSP 优化函数
- **动态维度支持**：通过 `arm_cmsis_dsp::DYNAMIC` 标记在**运行时**确定行列数，适合卡尔曼滤波器这类维度可变的算法

### 2.2 本项目中的关键用法

#### 类型别名

```cpp
using mat = arm_cmsis_dsp::Matrix<float, arm_cmsis_dsp::DYNAMIC, arm_cmsis_dsp::DYNAMIC>;
```

将 dsppp 动态矩阵实例化为 `float` 精度、运行时确定维度的类型 `mat`。所有的系统矩阵 (A, B, H, Q, R)、状态向量、中间变量均以 `mat` 声明。

#### 运算符重载

dsppp 重载了 `*`（矩阵乘）、`+`（矩阵加）、`-`（矩阵减），使得卡尔曼更新公式可以直接书写：

```cpp
// 先验状态预测
_tmp_x_1 = _mat_A * _vec_xhat;      // A * x_{k-1}
_tmp_x_2 = _mat_B * _vec_u;          // B * u_k
_vec_xhat_minus = _tmp_x_1 + _tmp_x_2;
```

编译器通过表达式模板将上述运算映射至 CMSIS-DSP 底层优化函数（如 `arm_mat_mult_f32`），最终执行的是 SIMD 加速的汇编级矩阵乘法。

#### `.transpose()` 方法

dsppp 矩阵提供 `.transpose()` 成员函数返回转置矩阵：

```cpp
_tmp_xx_2 = _mat_A.transpose();   // A^T
_mat_Ht   = _mat_H.transpose();   // H^T
```

#### 元素访问

- `matrix(r, c)`：二维索引，按行列访问
- `matrix[i]`：一维线性索引，按 row-major 顺序访问底层连续存储

```cpp
matrix(r, c) = (r == c) ? 1.0f : 0.0f;   // 单位阵设置
matrix[i] = data[i];                       // 线性填充
```

#### `.rows()` / `.columns()`

运行时查询矩阵维度，返回 `int`。在辅助函数中用于循环边界。

### 2.3 dsppp 的限制与规避

| 问题 | 描述 | 本代码的解决方案 |
|------|------|-----------------|
| **`Vector_Base` 拷贝构造函数 `= delete`** | dsppp 内部 `Vector_Base` 禁止拷贝构造，直接赋值同尺寸向量到另一向量可能触发编译错误 | 实现 `copy_vector()` 逐元素复制以规避 |
| **无矩阵求逆** | dsppp C++ 层未包装矩阵求逆函数 | 编写 `inverse_matrix()` 桥接函数，将 dsppp 矩阵的底层指针传给 C API `arm_mat_inverse_f32` |
| **无标量填充 / 单位阵** | dsppp 不提供 `fill` 或 `setIdentity` 方法 | 手动实现 `fill_scalar()`, `set_identity()`, `fill_mat()` 等辅助函数 |
| **无 `memcpy` 风格初始化** | 构造后无法批量从 `float*` 初始化 | 用 `fill_mat()` 逐元素写入 |

#### C/C++ 桥接：`inverse_matrix`

```cpp
arm_status kf_t::inverse_matrix(mat &src, mat &dst)
{
    arm_matrix_instance_f32 src_mat, dst_mat;
    arm_mat_init_f32(&src_mat, src.rows(), src.columns(), &src[0]);
    arm_mat_init_f32(&dst_mat, dst.rows(), dst.columns(), &dst[0]);
    return arm_mat_inverse_f32(&src_mat, &dst_mat);
}
```

**原理**：  
- `&src[0]` 取出 dsppp 矩阵底层 `float*` 数据指针
- 通过 `arm_mat_init_f32` 构造 C 结构体 `arm_matrix_instance_f32`，与 dsppp 共享同一块内存
- 调用 C API `arm_mat_inverse_f32` 执行基于 LU 分解的原地求逆
- 若矩阵奇异（行列式为零）返回 `ARM_MATH_SINGULAR`

这种方法享受了 dsppp C++ 层的运算符便利性，同时在需要时能无缝回退到 CMSIS-DSP C 层的高性能实现。

---

## 3. 类设计概览

### 3.1 类型别名

```cpp
using mat = arm_cmsis_dsp::Matrix<float, arm_cmsis_dsp::DYNAMIC, arm_cmsis_dsp::DYNAMIC>;
```

在 `pyro` 命名空间中定义，供滤波器内部及外部使用。

### 3.2 构造函数与内存策略

```cpp
kf_t::kf_t(uint8_t x_size, uint8_t u_size, uint8_t z_size)
    : _x_size(x_size), _u_size(u_size), _z_size(z_size), _is_init(false),
      _mat_A(x_size, x_size), _mat_B(x_size, u_size), ...
```

**策略**：所有矩阵在**成员初始化列表**中分配：

- dsppp `DYNAMIC` 矩阵在构造时从堆（或 memory pool）分配内存来存储矩阵数据
- 矩阵尺寸在构造后**不再变化**，避免嵌入式系统运行时反复 `malloc` / `free` 导致的堆碎片化
- 构造函数本身不填充数据，数据填充延迟到 `init()` 调用

### 3.3 成员变量一览

| 变量 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `_x_size`, `_u_size`, `_z_size`, `_w_size` | `uint8_t` | — | 维度参数 |
| `_is_init` | `bool` | — | 初始化标志位 |
| `_mat_A` | `mat` | $n \times n$ | 状态转移矩阵 |
| `_mat_B` | `mat` | $n \times m$ | 控制输入矩阵 |
| `_mat_H` | `mat` | $p \times n$ | 观测矩阵 |
| `_mat_G` | `mat` | $n \times w$ | 噪声传递矩阵 |
| `_mat_Q` | `mat` | $w \times w$ | 过程噪声协方差 |
| `_mat_R` | `mat` | $p \times p$ | 量测噪声协方差 |
| `_mat_K` | `mat` | $n \times p$ | 卡尔曼增益 |
| `_mat_P` | `mat` | $n \times n$ | 后验误差协方差 |
| `_mat_P_minus` | `mat` | $n \times n$ | 先验误差协方差 |
| `_vec_xhat` | `mat` | $n \times 1$ | 后验状态估计 |
| `_vec_xhat_minus` | `mat` | $n \times 1$ | 先验状态估计 |
| `_vec_z` | `mat` | $p \times 1$ | 量测向量缓存 |
| `_vec_u` | `mat` | $m \times 1$ | 控制输入缓存 |
| `_mat_I` | `mat` | $n \times n$ | 单位阵（Joseph 公式用） |
| `_tmp_x_1`, `_tmp_x_2` | `mat` | $n \times 1$ | 状态向量中间变量 |
| `_tmp_z_1` | `mat` | $p \times 1$ | 量测空间中间变量 |
| `_tmp_xx_1`, `_tmp_xx_2`, `_tmp_xx_3` | `mat` | $n \times n$ | 协方差运算中间变量 |
| `_mat_Ht` | `mat` | $n \times p$ | $H^T$ |
| `_mat_Gt` | `mat` | $w \times n$ | $G^T$ |
| `_mat_Kt` | `mat` | $p \times n$ | $K^T$ |
| `_mat_S` | `mat` | $p \times p$ | 新息协方差 |
| `_mat_S_inv` | `mat` | $p \times p$ | 新息协方差逆 |
| `_tmp_xz_1`, `_tmp_xz_2` | `mat` | $n \times p$ | 增益计算 / Joseph 公式中间变量 |
| `_tmp_xw_1` | `mat` | $n \times w$ | $G \cdot Q$ 协方差预测中间变量 |

> **为何预分配临时变量？**  
> 嵌入式系统中，`update()` 每控制周期调用一次（如 1kHz）。若每次在栈上构造 dsppp 矩阵则频繁分配/释放堆内存，不符合实时性要求。将所有临时矩阵作为成员变量一次性分配，更新时仅写入数据。

---

## 4. 公有 API 详解

### 4.1 构造函数

```cpp
kf_t(uint8_t x_size, uint8_t u_size, uint8_t z_size, uint8_t w_size);
```

| 参数 | 含义 |
|------|------|
| `x_size` | 状态向量维度 $n$ |
| `u_size` | 控制输入维度 $m$ |
| `z_size` | 量测向量维度 $p$ |
| `w_size` | 过程噪声向量维度 $w$ |

构造后滤波器处于**未初始化**状态（`_is_init = false`），不能调用 `update()`。

### 4.2 init() 四重载

提供四种初始化方式，覆盖不同场景：

```cpp
// 重载 1：默认 x₀ = 0, P₀ = I
status_t init(float *A, float *B, float *H, float *G, float *Q, float *R);

// 重载 2：自定义 x₀, 默认 P₀ = I
status_t init(float *A, float *B, float *H, float *G, float *Q, float *R, float *x0);

// 重载 3：默认 x₀ = 0, 自定义 P₀
status_t init(float *A, float *B, float *H, float *G, float *Q, float *R, std::nullptr_t, float *P0);

// 重载 4：自定义 x₀ 和 P₀
status_t init(float *A, float *B, float *H, float *G, float *Q, float *R, float *x0, float *P0);
```

**重载消歧设计**：重载 3 使用 `std::nullptr_t` 占位参数来表达「x₀ 使用默认值（零向量）」，避免 `init(A,B,H,Q,R, nullptr, P0)` 与重载 2 产生歧义（因为 `nullptr` 可隐式转换为 `float*`）。

所有重载最终委托给私有方法 `init_impl()`。

**返回值**：

| 返回值 | 含义 |
|--------|------|
| `PYRO_OK` | 初始化成功 |
| `PYRO_PARAM_ERROR` | 必须参数为空指针，或 P₀ 校验未通过 |
| `PYRO_ALREADY_INIT` | 滤波器已初始化，禁止重复调用 |

### 4.3 update()

```cpp
status_t update(float *measure_vec, float *control_vec, float *estimated_ret);
```

| 参数 | 含义 |
|------|------|
| `measure_vec` | 当前时刻量测向量 $\mathbf{z}_k$，长度 `z_size` |
| `control_vec` | 当前时刻控制输入 $\mathbf{u}_k$，长度 `u_size` |
| `estimated_ret` | 输出缓冲区，写入后验状态估计 $\hat{\mathbf{x}}_k$，长度 `x_size` |

每个控制周期调用一次，完成一轮预测 + 更新。

### 4.4 get_state()

```cpp
status_t get_state(float *out) const;
```

| 参数 | 含义 |
|------|------|
| `out` | 输出缓冲区，写入当前后验状态估计 $\hat{\mathbf{x}}_{k|k}$，长度 `x_size` |

随时可调用（无需等待下一次 `update()`），用于在控制周期之外读取最新滤波结果。

| 返回值 | 含义 |
|--------|------|
| `PYRO_OK` | 成功 |
| `PYRO_PARAM_ERROR` | `out` 为空指针 |
| `PYRO_NOT_FOUND` | 滤波器未初始化 |

---

### 4.5 reset()

提供三个重载，用于在运行时将滤波器状态重置到指定值，同时清零 K、P⁻、u、z。

#### 重载 1 — 归零重置

```cpp
status_t reset();
```

将状态向量清零，协方差矩阵恢复为单位阵。适用于完全重新开始的场景。

#### 重载 2 — 指定 x₀ 与 P₀

```cpp
status_t reset(float *x0_data, float *P0_data);
```

| 参数 | 含义 |
|------|------|
| `x0_data` | 新状态向量数据，长度 `x_size` |
| `P0_data` | 新协方差矩阵数据（row-major），shape `x_size × x_size`，需通过对称/非负/有限校验 |

#### 重载 3 — 仅重置状态向量

```cpp
status_t reset(float *x0_data);
```

仅更新状态向量，保留当前协方差矩阵 P。

| 参数 | 含义 |
|------|------|
| `x0_data` | 新状态向量数据，长度 `x_size` |

#### 公共返回值

| 返回值 | 含义 |
|--------|------|
| `PYRO_OK` | 成功 |
| `PYRO_PARAM_ERROR` | 指针为空 / P₀ 校验不通过（重载 2） |
| `PYRO_NOT_FOUND` | 滤波器未初始化 |

---

## 5. 私有辅助函数详解

| 函数 | 功能 | 说明 |
|------|------|------|
| `fill_mat(mat&, const float*)` | 将 row-major 浮点数组逐元素填入矩阵 | 利用 `matrix[i]` 线性寻址 |
| `fill_scalar(mat&, float)` | 将矩阵所有元素设为同一标量值 | 典型用途：清零 K 和 P⁻ |
| `assign_vector(mat&, const float*)` | 将外部 `float*` 数据写入列向量 | 用 `(i, 0)` 二维索引按行写入 |
| `copy_vector(mat&, const mat&)` | 逐元素复制列向量 | **规避 dsppp `Vector_Base` 拷贝构造删除** |
| `clear_vector(mat&)` | 清零列向量 | 初始化 xhat, u, z 等 |
| `set_identity(mat&)` | 将方阵设为单位阵 | 对角=1, 非对角=0 |
| `validate_covariance_data(const float*, uint8_t)` | 校验 P₀ 数据是否合法 | 检查有限性、对称性、对角非负 |
| `inverse_matrix(mat&, mat&)` | 矩阵求逆（桥接 CMSIS C API） | LU 分解实现，奇异时返回错误码 |

---

## 6. update() 逐步执行流程

以下对应代码中注释的 **Step 1 ~ 11**。

### Step 1-2：输入校验

```cpp
if (measure_vec == nullptr || ...) return PYRO_PARAM_ERROR;
if (!_is_init) return PYRO_NOT_FOUND;
```

### Step 3：拷贝输入

```cpp
assign_vector(_vec_z, measure_vec);   // z_k
assign_vector(_vec_u, control_vec);   // u_k
```

将外部 `float*` 数据写入内部 dsppp 列向量。

### Step 4：先验状态预测

$$\hat{\mathbf{x}}_k^- = \mathbf{A}\,\hat{\mathbf{x}}_{k-1} + \mathbf{B}\,\mathbf{u}_k$$

```cpp
_tmp_x_1 = _mat_A * _vec_xhat;       // n×n · n×1 → n×1
_tmp_x_2 = _mat_B * _vec_u;           // n×m · m×1 → n×1
_vec_xhat_minus = _tmp_x_1 + _tmp_x_2;
```

> 注意：dsppp 表达式 `A * xhat` 会调用底层 `arm_mat_mult_f32`，在 Cortex-M7 上利用双发射流水线加速。

### Step 5：先验协方差预测

$$\mathbf{P}_k^- = \mathbf{A}\,\mathbf{P}_{k-1}\,\mathbf{A}^T + \mathbf{G}\,\mathbf{Q}\,\mathbf{G}^T$$

```cpp
_tmp_xx_1 = _mat_A * _mat_P;         // n×n · n×n → n×n
_tmp_xx_2 = _mat_A.transpose();       // n×n
_mat_P_minus = _tmp_xx_1 * _tmp_xx_2; // A * P * A^T
_tmp_xw_1 = _mat_G * _mat_Q;         // n×w · w×w → n×w
_mat_Gt = _mat_G.transpose();         // w×n
_tmp_xx_1 = _tmp_xw_1 * _mat_Gt;     // n×w · w×n → n×n  (G*Q*G^T)
_mat_P_minus = _mat_P_minus + _tmp_xx_1;
```

### Step 6：新息协方差

$$\mathbf{S}_k = \mathbf{H}\,\mathbf{P}_k^-\,\mathbf{H}^T + \mathbf{R}$$

```cpp
_mat_Ht   = _mat_H.transpose();              // n×p
_tmp_z_1  = _mat_H * _vec_xhat_minus;        // p×n · n×1 → p×1 (预测量测)
_tmp_xz_1 = _mat_P_minus * _mat_Ht;          // n×n · n×p → n×p
_mat_S    = _mat_H * _tmp_xz_1 + _mat_R;     // p×n · n×p → p×p
```

> `_tmp_z_1` 在此计算 $\mathbf{H}\hat{\mathbf{x}}_k^-$ 以便后续 Step 9 使用，避免重复计算。

### Step 7：新息协方差求逆

$$\mathbf{S}_k^{-1}$$

```cpp
arm_status math_ret = inverse_matrix(_mat_S, _mat_S_inv);
CHECK_ARM_MATH_RET(math_ret);  // 若奇异则返回 PYRO_ERROR
```

通过 C 桥接调用 `arm_mat_inverse_f32`，执行 LU 分解。

### Step 8：卡尔曼增益

$$\mathbf{K}_k = \mathbf{P}_k^-\,\mathbf{H}^T\,\mathbf{S}_k^{-1}$$

```cpp
_mat_K = _tmp_xz_1 * _mat_S_inv;   // n×p · p×p → n×p
```

> `_tmp_xz_1` 在 Step 6 已计算为 $\mathbf{P}_k^- \mathbf{H}^T$，此处直接复用。

### Step 9：后验状态修正

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k\,(\mathbf{z}_k - \mathbf{H}\,\hat{\mathbf{x}}_k^-)$$

```cpp
_vec_xhat = _vec_xhat_minus + _mat_K * (_vec_z - _tmp_z_1);
```

- `_vec_z - _tmp_z_1` 即新息 $\boldsymbol{\nu}_k = \mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_k^-$
- `_mat_K * ν` 将新息加权映射回状态空间

### Step 10：Joseph 形式协方差更新

$$
\mathbf{A}_j = \mathbf{I} - \mathbf{K}_k\,\mathbf{H}
$$
$$
\mathbf{P}_k = \mathbf{A}_j\,\mathbf{P}_k^-\,\mathbf{A}_j^T + \mathbf{K}_k\,\mathbf{R}\,\mathbf{K}_k^T
$$

```cpp
// Part A: A_j * P_k^- * A_j^T
_tmp_xx_3 = _mat_I - _mat_K * _mat_H;       // A_j
_tmp_xx_1 = _tmp_xx_3 * _mat_P_minus;       // A_j * P^-
_tmp_xx_2 = _tmp_xx_3.transpose();            // A_j^T
_mat_P    = _tmp_xx_1 * _tmp_xx_2;           // A_j * P^- * A_j^T

// Part B: K * R * K^T
_tmp_xz_2 = _mat_K * _mat_R;                 // n×p · p×p → n×p
_mat_Kt   = _mat_K.transpose();              // p×n
_tmp_xx_1 = _tmp_xz_2 * _mat_Kt;            // n×p · p×n → n×n

// Sum
_mat_P = _mat_P + _tmp_xx_1;
```

### Step 11：输出

```cpp
for (int i = 0; i < _x_size; ++i)
    estimated_ret[i] = _vec_xhat(i, 0);
```

将后验状态逐元素写入调用者提供的缓冲区。

---

## 7. 初始化流程 (init_impl)

```
init_impl(A, B, H, G, Q, R, x0, P0)
    │
    ├─ Step 1: 检查 A/B/H/G/Q/R 指针非空
    │
    ├─ Step 2: 检查 _is_init 防止重复初始化
    │
    ├─ Step 2.1: 若 P0 ≠ nullptr，调用 validate_covariance_data() 校验
    │
    ├─ Step 3: fill_mat() 填充 A, B, H, G, Q, R
    │
    ├─ Step 4: 初始化后验状态 x̂₀
    │   ├─ x0 == nullptr → clear_vector(_vec_xhat)    // 零向量
    │   └─ x0 != nullptr → assign_vector(_vec_xhat, x0)
    │   └─ copy_vector(_vec_xhat_minus, _vec_xhat)    // 同步先验缓存
    │
    ├─ Step 5: 清零 u, z 向量
    │
    ├─ Step 6: 初始化工作矩阵
    │   ├─ K = 0, P⁻ = 0
    │   ├─ I = 单位阵
    │   ├─ P0 == nullptr → P = I
    │   └─ P0 != nullptr → P = P0
    │
    └─ Step 7: _is_init = true
```

---

## 8. P₀ 运行时校验

当用户通过重载 3 或 4 传入自定义 $\mathbf{P}_0$ 时，`validate_covariance_data()` 执行三项运行时检查：

### 检查 1：有限性

```cpp
if (!std::isfinite(a)) return false;
```

排除 `NaN`、`±Inf` 等非法浮点值。

### 检查 2：对称性

协方差矩阵必须满足 $P_{ij} = P_{ji}$。使用相对容差检查：

```cpp
float scale = fmax(1.0f, fmax(|a|, |b|));
if (|a - b| > 1e-5 * scale) return false;
```

- `scale` 归一化避免小值情况下的假阳性
- 容差 `1e-5` 在 `float32` 精度下合理（float32 有效精度约 7 位十进制）

### 检查 3：对角非负

协方差矩阵对角元素 $P_{ii} = \text{Var}(x_i) \geq 0$：

```cpp
if (diag < 0.0f) return false;
```

> **注意**：上述三项检查是**必要条件**但非充分条件。完整的正半定性验证需要 Cholesky 分解或特征值分解，在嵌入式实时场景中计算成本过高，因此仅做以上轻量级检查。

---

## 9. 使用示例

```cpp
#include "kf.h"

// 2 维状态, 1 维控制, 1 维量测, 1 维过程噪声
pyro::kf_t kf(2, 1, 1, 1);

// 系统矩阵 (row-major float 数组)
float A[] = {1.0f, 0.01f, 0.0f, 1.0f};  // 2x2
float B[] = {0.0f, 0.01f};               // 2x1
float H[] = {1.0f, 0.0f};                // 1x2
float G[] = {0.005f, 0.01f};             // 2x1  (dt²/2, dt) 加速度噪声映射
float Q[] = {1.0f};                       // 1x1  加速度噪声方差 σ²_a
float R[] = {0.1f};

// --- 方式 1: 默认初始化 (x₀=0, P₀=I) ---
kf.init(A, B, H, G, Q, R);

// --- 方式 2: 自定义初始状态 ---
float x0[] = {1.0f, 0.0f};
kf.init(A, B, H, G, Q, R, x0);

// --- 方式 3: 自定义初始协方差 ---
float P0[] = {10.0f, 0.0f, 0.0f, 10.0f};
kf.init(A, B, H, G, Q, R, nullptr, P0);

// --- 方式 4: 同时自定义 x₀ 和 P₀ ---
kf.init(A, B, H, G, Q, R, x0, P0);

// 周期调用 update()
float z[1], u[1], x_est[2];
while (true)
{
    z[0] = read_sensor();
    u[0] = get_control_input();
    kf.update(z, u, x_est);
    // x_est[0] = 位置估计, x_est[1] = 速度估计
}
```

---

## 10. 设计考量与工程细节

### 10.1 零动态分配策略

所有矩阵在构造函数中**一次性**分配内存，之后的 `init()` 和 `update()` 仅做数据写入，不触发堆操作。这对 FreeRTOS 实时任务至关重要——在 1kHz 控制循环中频繁的 malloc/free 会导致不确定的延迟和堆碎片化。

### 10.2 命名空间隔离

所有类型和实现位于 `pyro` 命名空间内，避免与 CMSIS 全局符号或其他模块冲突。类型别名 `mat` 也限定在 `pyro::` 下。

### 10.3 错误码体系

| 错误码 | 含义 |
|--------|------|
| `PYRO_OK` | 成功 |
| `PYRO_PARAM_ERROR` | 参数校验失败（空指针 / P₀ 数据非法） |
| `PYRO_ALREADY_INIT` | 重复初始化 |
| `PYRO_NOT_FOUND` | 未初始化就调用 update |
| `PYRO_ERROR` | 矩阵求逆失败（S 奇异） |

`CHECK_ARM_MATH_RET(math_ret)` 宏将 CMSIS `arm_status` 映射为 `status_t`。

### 10.4 表达式模板的内存效率

dsppp 使用 C++ 表达式模板链式延迟求值。例如：

```cpp
_vec_xhat = _vec_xhat_minus + _mat_K * (_vec_z - _tmp_z_1);
```

编译器不会为 `_vec_z - _tmp_z_1` 和 `_mat_K * (...)` 分别创建临时矩阵对象，而是将整条表达式融合为一次遍历计算，写入 `_vec_xhat` 的存储空间。这在内存受限的 MCU 上显著减少了堆压力。

### 10.5 可扩展性

- 如需实现 **扩展卡尔曼滤波 (EKF)**，可在子类中覆盖 `update()` 并在每步重新计算 Jacobian $\mathbf{A}_k$, $\mathbf{H}_k$
- 如需 **自适应滤波**，可在 `update()` 内动态调整 Q / R
- 矩阵维度为运行时参数（`DYNAMIC`），同一编译产物可支持不同维度的滤波实例

---

*文档生成日期: 2026-04-07*  
*适用代码版本: kf.h / kf.cpp (噪声传递矩阵 G, Joseph 形式, 四重载 init, P₀ 校验)*
