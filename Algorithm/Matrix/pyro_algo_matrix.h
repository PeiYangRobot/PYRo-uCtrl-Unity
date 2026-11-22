/**
 * @file pyro_algo_matrix.h
 * @brief 固定存储矩阵、借用视图及 float32 矩阵运算。
 *        Fixed-storage matrices, borrowed views and float32 operations.
 */
#ifndef PYRO_ALGO_MATRIX_H
#define PYRO_ALGO_MATRIX_H

// 不向使用方泄漏 CMSIS 的 PI 宏 / Do not leak the CMSIS PI macro.
#pragma push_macro("PI")
#include "arm_math.h"
#pragma pop_macro("PI")
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace pyro
{

/** @brief 固定尺寸行优先矩阵，元素 (r,c) 位于 r*Cols+c。
 *         Fixed-shape row-major matrix; element (r,c) is at r*Cols+c.
 * 持有 std::array 存储，不分配堆内存；行列信息用于编译期运算检查。
 * Owns std::array storage without heap allocation; shapes checked at compile time.
 * 零维仅用于无控制模型的空类型，不能参与乘法或转置。
 * Zero dimensions represent absent controls; no product or transpose allowed.
 */
template <std::size_t Rows, std::size_t Cols> class matrix_t
{
    static_assert(Rows <= 65535 && Cols <= 65535, "Invalid matrix dimensions");
    static_assert(Rows == 0 || Cols <= std::numeric_limits<std::size_t>::max() /
                                           sizeof(float) /
                                           (Rows == 0 ? 1 : Rows),
                  "Matrix storage size overflow");

  public:
    using storage_t                   = std::array<float, Rows * Cols>;
    static constexpr std::size_t rows = Rows;
    static constexpr std::size_t cols = Cols;

    /// 默认全零 / Zero-initialized by default.
    constexpr matrix_t()              = default;

    /// 行优先列表，未给出的元素为零；过多元素编译失败。
    /// Row-major list, omitted entries zeroed; excess entries fail compilation.
    template <typename... Values,
              std::enable_if_t<(sizeof...(Values) > 0 &&
                                sizeof...(Values) <= Rows * Cols &&
                                (std::is_convertible_v<Values, float> && ...)),
                               int> = 0>
    constexpr matrix_t(Values... values)
        : _values{static_cast<float>(values)...}
    {
    }

    /// 从同容量数组复制；可直接传入原有 std::array 参数。
    /// Copy same-sized array; existing std::array arguments remain accepted.
    constexpr matrix_t(const storage_t &values) : _values(values)
    {
    }

    constexpr float *data()
    {
        return _values.data();
    }
    constexpr const float *data() const
    {
        return _values.data();
    }
    constexpr std::size_t size() const
    {
        return Rows * Cols;
    }
    constexpr bool empty() const
    {
        return size() == 0;
    }
    constexpr auto begin()
    {
        return _values.begin();
    }
    constexpr auto begin() const
    {
        return _values.begin();
    }
    constexpr auto end()
    {
        return _values.end();
    }
    constexpr auto end() const
    {
        return _values.end();
    }
    void fill(float value)
    {
        _values.fill(value);
    }
    /// 一维下标，不检查越界 / Flat index without bounds checks.
    constexpr float &operator[](std::size_t index)
    {
        return _values[index];
    }
    constexpr const float &operator[](std::size_t index) const
    {
        return _values[index];
    }
    /// 二维下标，不检查越界 / Row/column index without bounds checks.
    constexpr float &operator()(std::size_t row, std::size_t col)
    {
        return _values[row * Cols + col];
    }
    constexpr const float &operator()(std::size_t row, std::size_t col) const
    {
        return _values[row * Cols + col];
    }
    /// 借用底层数组，供既有数组接口适配 / Borrow storage for array APIs.
    constexpr storage_t &as_array() &
    {
        return _values;
    }
    constexpr const storage_t &as_array() const &
    {
        return _values;
    }
    // 避免从临时矩阵借用悬空数组 / Prevent borrowing from temporaries.
    storage_t &as_array()             && = delete;
    const storage_t &as_array() const && = delete;

    /// 同形状加减 / Same-shaped sum and difference.
    matrix_t operator+(const matrix_t &B) const;
    matrix_t operator-(const matrix_t &B) const;
    matrix_t operator-() const;
    matrix_t &operator+=(const matrix_t &B);
    matrix_t &operator-=(const matrix_t &B);
    /// 矩阵乘法；B 行数必须等于本矩阵列数 / Shape-checked matrix product.
    template <std::size_t OtherCols>
    matrix_t<Rows, OtherCols>
    operator*(const matrix_t<Cols, OtherCols> &B) const;
    /// 原地右乘方阵，安全支持 A*=A / Right-multiply square B; A*=A is safe.
    matrix_t &operator*=(const matrix_t<Cols, Cols> &B);
    /// 标量乘除；除零填充 NaN / Scalar scaling; division by zero fills NaN.
    matrix_t operator*(float scalar) const;
    matrix_t operator/(float scalar) const;
    matrix_t &operator*=(float scalar);
    matrix_t &operator/=(float scalar);
    /// 返回独立转置结果 / Return an independent transposed matrix.
    matrix_t<Cols, Rows> transpose() const;
    /// 精确逐元素相等，不是近似比较 / Exact elementwise equality.
    bool operator==(const matrix_t &B) const
    {
        return _values == B._values;
    }
    bool operator!=(const matrix_t &B) const
    {
        return !(*this == B);
    }

  private:
    storage_t _values{};
};

/// 支持 scalar*A / Support scalar*A.
template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> operator*(float scalar, const matrix_t<Rows, Cols> &A);

/** @brief 向量存储，Size×1 / Vector storage, Size×1. */
template <std::size_t Size> using vector_t = matrix_t<Size, 1>;

namespace matrix
{

/** @brief 只读紧凑矩阵视图；借用连续 rows*cols 个 float。
 *         Read-only packed view borrowing rows*cols contiguous floats.
 * 不拥有数据；调用方保证容量、生命周期及同步，无跨行填充。
 * Caller ensures capacity, lifetime and synchronization; no row padding.
 */
struct const_view_t
{
    const float *data;
    std::size_t rows;
    std::size_t cols;
};

/** @brief 可写紧凑矩阵视图；容量和生命周期规则同 const_view_t。
 *         Writable packed view with the same borrowing rules.
 */
struct view_t
{
    float *data;
    std::size_t rows;
    std::size_t cols;

    operator const_view_t() const
    {
        return {data, rows, cols};
    }
};

/** @brief 从固定数组创建视图，编译期检查容量；尾部可作为工作区。
 *         Make a view with compile-time capacity checks; unused tail allowed.
 */
template <std::size_t Rows, std::size_t Cols, std::size_t Size>
constexpr view_t view(std::array<float, Size> &values)
{
    static_assert(Rows > 0 && Cols > 0 && Rows <= 65535 && Cols <= 65535,
                  "Invalid matrix dimensions");
    static_assert(Size >= Rows * Cols, "Insufficient matrix storage");
    return {values.data(), Rows, Cols};
}

template <std::size_t Rows, std::size_t Cols, std::size_t Size>
constexpr const_view_t view(const std::array<float, Size> &values)
{
    static_assert(Rows > 0 && Cols > 0 && Rows <= 65535 && Cols <= 65535,
                  "Invalid matrix dimensions");
    static_assert(Size >= Rows * Cols, "Insufficient matrix storage");
    return {values.data(), Rows, Cols};
}

/// 从矩阵推导形状；也可显式指定较小紧凑工作区形状。
/// Deduce matrix shape, or explicitly select a smaller packed workspace shape.
template <std::size_t ViewRows = 0, std::size_t ViewCols = 0, std::size_t Rows,
          std::size_t Cols>
constexpr view_t view(matrix_t<Rows, Cols> &values)
{
    constexpr auto r = ViewRows == 0 ? Rows : ViewRows;
    constexpr auto c = ViewCols == 0 ? Cols : ViewCols;
    static_assert(r > 0 && c > 0 && r <= 65535 && c <= 65535,
                  "Invalid matrix dimensions");
    static_assert(r * c <= Rows * Cols, "Insufficient matrix storage");
    return {values.data(), r, c};
}

template <std::size_t ViewRows = 0, std::size_t ViewCols = 0, std::size_t Rows,
          std::size_t Cols>
constexpr const_view_t view(const matrix_t<Rows, Cols> &values)
{
    constexpr auto r = ViewRows == 0 ? Rows : ViewRows;
    constexpr auto c = ViewCols == 0 ? Cols : ViewCols;
    static_assert(r > 0 && c > 0 && r <= 65535 && c <= 65535,
                  "Invalid matrix dimensions");
    static_assert(r * c <= Rows * Cols, "Insufficient matrix storage");
    return {values.data(), r, c};
}

// 按 float32 位模式识别 NaN/Inf，支持 finite-math 优化。
// Inspect float32 bits so finite-math optimizations cannot bypass guards.
inline bool is_finite(float value)
{
    static_assert(sizeof(float) == sizeof(uint32_t), "Requires float32");
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000U) != 0x7f800000U;
}

template <std::size_t Size>
bool is_finite(const std::array<float, Size> &values)
{
    for (float value : values)
        if (!is_finite(value))
            return false;
    return true;
}

template <std::size_t Rows, std::size_t Cols>
bool is_finite(const matrix_t<Rows, Cols> &values)
{
    return is_finite(values.as_array());
}

/**
 * 运算返回 CMSIS arm_status，以保留底层错误码。
 * Operations return CMSIS arm_status to preserve backend diagnostics.
 * 无效指针/尺寸/重叠返回 ARGUMENT_ERROR，形状不匹配返回 SIZE_MISMATCH。
 * Invalid pointers/dimensions/overlap: ARGUMENT_ERROR; shapes: SIZE_MISMATCH.
 * 不分配内存；检查失败不写输出；数值求解失败时输出可能部分改写。
 * No allocation; validation failures leave output intact; solver failures may
 * partially write output. 数值有限性及业务约束由调用方检查。
 * Callers check numeric finiteness and application-specific constraints.
 */

/// out=A*B；输出不得与输入重叠 / Product; output must not overlap inputs.
arm_status multiply(const_view_t A, const_view_t B, view_t out);
/// out=A^T；输出不得与输入重叠 / Transpose; disjoint output required.
arm_status transpose(const_view_t A, view_t out);
/// out=A+B；允许 out 与任一输入完全相同，不允许部分重叠。
/// Sum; exact in-place output allowed, partial overlap rejected.
arm_status add(const_view_t A, const_view_t B, view_t out);
/// out=A-B；重叠规则同 add / Difference; same overlap rules as add.
arm_status subtract(const_view_t A, const_view_t B, view_t out);
/// out=A^-1；workspace 为同形状可写副本，三个缓冲区互不重叠。
/// Inverse using same-shaped scratch; all three buffers must be disjoint.
/// A 保持不变，workspace 会被改写 / A is preserved; scratch is overwritten.
arm_status inverse(const_view_t A, view_t out, view_t workspace);
/// A=L*L^T；要求 A 对称正定，L 上三角清零，输入输出不重叠。
/// Cholesky for symmetric positive-definite A; zero upper L; disjoint buffers.
arm_status cholesky(const_view_t A, view_t L);
/// L*X=B；L 下三角，B 可有多列；输出不与输入重叠。
/// Solve lower-triangular L*X=B for multiple RHS columns; disjoint output.
arm_status solve_lower(const_view_t L, const_view_t B, view_t X);
/// U*X=B；U 上三角，其他规则同 solve_lower。
/// Solve upper-triangular U*X=B; same rules as solve_lower.
arm_status solve_upper(const_view_t U, const_view_t B, view_t X);

} // namespace matrix
} // namespace pyro

#include "pyro_algo_matrix.tpp"
#endif // PYRO_ALGO_MATRIX_H
