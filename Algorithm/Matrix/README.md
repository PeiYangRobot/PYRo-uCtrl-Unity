# Matrix 矩阵模块

`pyro_algo_matrix.h/.tpp/.cpp` 提供通用 float32 矩阵存储和运算，乘法、转置和求解使用 CMSIS-DSP 后端。
模块不依赖 KF、EKF、INS 或硬件驱动；KF 和其他算法通过矩阵接口使用它。

`pyro::matrix_t<Rows,Cols>` 是拥有行列信息的固定矩阵类型，内部仅持有
`std::array<float,Rows*Cols>`。`vector_t<Size>` 是 `matrix_t<Size,1>` 的别名。
元素 `(row,col)` 位于 `row*Cols+col`；支持行优先初始化列表、`[]`、`(row,col)`、
`data()`、`fill()`、`size()`、迭代和值复制，不分配堆内存。
列表未给出的元素初始化为零，元素过多时编译失败。

```cpp
#include "pyro_algo_matrix.h"

pyro::matrix_t<2, 2> F{1.0f, 0.001f, 0.0f, 1.0f};
pyro::vector_t<2> x{0.0f, 1.0f};
auto x_pred = F * x;
pyro::matrix_t<2, 2> P{1, 0, 0, 1}, Q_d{1e-6f, 0, 0, 1e-3f};
auto P_pred = F * P * F.transpose() + Q_d;
P += Q_d;
P *= 0.95f;
float variance = P(1, 1);
```

支持 `A+B`、`A-B`、`-A`、`A*B`、`scalar*A`、`A*scalar`、`A/scalar`，
以及 `+=`、`-=`、标量 `*=` / `/=` 和右乘方阵的 `*=`。
加减必须同形状，乘法必须满足左列数等于右行数；不匹配时无法通过编译。
转置使用 `A.transpose()`，返回行列交换后的独立矩阵。
`==` / `!=` 为精确逐元素比较，不用于浮点近似判断。

运算符立即计算并返回独立固定存储结果，安全支持 `A=A*A`、`A*=A`，
不会保存表达式引用。复合表达式会产生固定大小临时对象；KF 核心继续复用
已有工作区和带状态码的接口，避免增加求解路径的临时矩阵。
运算符不抛异常、不返回状态码，数值溢出通过 `matrix::is_finite(result)` 检查；
标量除零填充 NaN。求逆、分解和三角求解继续使用下方带状态码的接口。

矩阵不再是 `std::array` 别名。可从同容量数组构造或赋值；向要求数组引用的
旧接口传入 `A.as_array()`。形状不同的矩阵不能因元素个数相同而互换。

`matrix::view(A)` 自动推导矩阵形状；`matrix::view<Rows,Cols>(array_or_matrix)`
在编译期检查容量，可选择较小紧凑工作区形状。视图仅借用存储，
不复制、不保存数据；调用方保证存储的生命周期和同步。
对于 KF 缺测压缩等运行期尺寸，可直接传入 `{data,rows,cols}`；
调用方保证至少有 `rows*cols` 个连续元素，无行尾填充。

| 接口 | 计算 / Operation | 缓冲区规则 |
|---|---|---|
| `multiply(A,B,out)` | `out=A*B` | 输出与输入不重叠 |
| `transpose(A,out)` | `out=Aᵀ` | 输出与输入不重叠 |
| `add(A,B,out)` | `out=A+B` | 支持完全原地操作，拒绝部分重叠 |
| `subtract(A,B,out)` | `out=A-B` | 同上 |
| `inverse(A,out,workspace)` | `out=A⁻¹` | 三个缓冲区互不重叠、同形状；原始 A 保持不变 |
| `cholesky(A,L)` | `A=L*Lᵀ` | A 对称正定；L 上三角清零；不重叠 |
| `solve_lower(L,B,X)` | `L*X=B` | 下三角、多列右端项；输出与输入不重叠 |
| `solve_upper(U,B,X)` | `U*X=B` | 上三角，其余同上 |
| `is_finite(value/array)` | 检查 NaN/Inf | 按位判断，支持 finite-math 优化 |

以上带输出视图的运算返回 `arm_status`，便于上层保留 CMSIS 诊断。矩阵维度必须在
`1..65535` 内，空指针、非法维度及禁止的重叠返回 `ARM_MATH_ARGUMENT_ERROR`，
形状不匹配返回 `ARM_MATH_SIZE_MISMATCH`。结构检查失败时不写输出；
求逆、分解或三角求解的数值失败可能留下部分结果，上层应检查状态后再提交。
运算接口不隐式检查全部输入输出的有限性、对称性或半正定性，业务约束由调用方负责。
`is_finite` 仅在主动调用时遍历数据，不是运算或运算符的自动步骤。

KF 保留状态预测、残差构造、协方差公式、NIS、门控、方差下限及提交语义；
矩阵模块负责矩阵运算及后端描述符、工作区处理。KF 的维度别名保持原名，
初始化列表和矩阵参数的 `predict/correct` 调用保持原写法；同容量数组可按值适配，
传入 `P_prior` 指针时使用 `covariance_t` 对象。
