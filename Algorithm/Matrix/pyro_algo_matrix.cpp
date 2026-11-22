/**
 * @file pyro_algo_matrix.cpp
 * @brief CMSIS-DSP 矩阵后端与视图检查 / CMSIS-DSP backend and view checks.
 */
#include "pyro_algo_matrix.h"
#include <algorithm>
#include <limits>

namespace pyro
{
namespace matrix
{
namespace
{
bool valid(const_view_t A)
{
    return A.data && A.rows > 0 && A.cols > 0 && A.rows <= 65535 &&
           A.cols <= 65535 &&
           A.rows <=
               std::numeric_limits<std::size_t>::max() / sizeof(float) / A.cols;
}

bool same_shape(const_view_t A, const_view_t B)
{
    return A.rows == B.rows && A.cols == B.cols;
}

// 地址差避免尾地址加法溢出；只在视图已验证后调用。
// Address differences avoid end-address overflow; validate views first.
bool overlaps(const_view_t A, const_view_t B)
{
    const auto a = reinterpret_cast<std::uintptr_t>(A.data);
    const auto b = reinterpret_cast<std::uintptr_t>(B.data);
    return a <= b ? b - a < A.rows * A.cols * sizeof(float)
                  : a - b < B.rows * B.cols * sizeof(float);
}

arm_matrix_instance_f32 descriptor(const_view_t A)
{
    arm_matrix_instance_f32 result;
    // 只读运算不改写输入；const_cast 仅适配 CMSIS 描述符。
    // Read-only operations preserve inputs; adapt CMSIS's mutable descriptor.
    arm_mat_init_f32(&result, static_cast<uint16_t>(A.rows),
                     static_cast<uint16_t>(A.cols),
                     const_cast<float *>(A.data));
    return result;
}

arm_status elementwise(const_view_t A, const_view_t B, view_t out,
                       bool subtract_values)
{
    if (!valid(A) || !valid(B) || !valid(out))
        return ARM_MATH_ARGUMENT_ERROR;
    if (!same_shape(A, B) || !same_shape(A, out))
        return ARM_MATH_SIZE_MISMATCH;
    if ((overlaps(A, out) && A.data != out.data) ||
        (overlaps(B, out) && B.data != out.data))
        return ARM_MATH_ARGUMENT_ERROR;
    auto a = descriptor(A), b = descriptor(B), result = descriptor(out);
    return subtract_values ? arm_mat_sub_f32(&a, &b, &result)
                           : arm_mat_add_f32(&a, &b, &result);
}

arm_status triangular(const_view_t A, const_view_t B, view_t X, bool lower)
{
    if (!valid(A) || !valid(B) || !valid(X))
        return ARM_MATH_ARGUMENT_ERROR;
    if (A.rows != A.cols || A.cols != B.rows || !same_shape(B, X))
        return ARM_MATH_SIZE_MISMATCH;
    if (overlaps(A, X) || overlaps(B, X))
        return ARM_MATH_ARGUMENT_ERROR;
    auto a = descriptor(A), b = descriptor(B), result = descriptor(X);
    return lower ? arm_mat_solve_lower_triangular_f32(&a, &b, &result)
                 : arm_mat_solve_upper_triangular_f32(&a, &b, &result);
}
} // namespace

arm_status multiply(const_view_t A, const_view_t B, view_t out)
{
    if (!valid(A) || !valid(B) || !valid(out))
        return ARM_MATH_ARGUMENT_ERROR;
    if (A.cols != B.rows || out.rows != A.rows || out.cols != B.cols)
        return ARM_MATH_SIZE_MISMATCH;
    if (overlaps(A, out) || overlaps(B, out))
        return ARM_MATH_ARGUMENT_ERROR;
    auto a = descriptor(A), b = descriptor(B), result = descriptor(out);
    return arm_mat_mult_f32(&a, &b, &result);
}

arm_status transpose(const_view_t A, view_t out)
{
    if (!valid(A) || !valid(out))
        return ARM_MATH_ARGUMENT_ERROR;
    if (out.rows != A.cols || out.cols != A.rows)
        return ARM_MATH_SIZE_MISMATCH;
    if (overlaps(A, out))
        return ARM_MATH_ARGUMENT_ERROR;
    auto a = descriptor(A), result = descriptor(out);
    return arm_mat_trans_f32(&a, &result);
}

arm_status add(const_view_t A, const_view_t B, view_t out)
{
    return elementwise(A, B, out, false);
}

arm_status subtract(const_view_t A, const_view_t B, view_t out)
{
    return elementwise(A, B, out, true);
}

arm_status inverse(const_view_t A, view_t out, view_t workspace)
{
    if (!valid(A) || !valid(out) || !valid(workspace))
        return ARM_MATH_ARGUMENT_ERROR;
    if (A.rows != A.cols || !same_shape(A, out) || !same_shape(A, workspace))
        return ARM_MATH_SIZE_MISMATCH;
    if (overlaps(A, out) || overlaps(A, workspace) || overlaps(out, workspace))
        return ARM_MATH_ARGUMENT_ERROR;
    // CMSIS 求逆修改源矩阵；显式借用工作区保证原始 A 不变。
    // CMSIS inversion mutates its source; caller-owned scratch preserves A.
    std::copy_n(A.data, A.rows * A.cols, workspace.data);
    auto work = descriptor(workspace), result = descriptor(out);
    return arm_mat_inverse_f32(&work, &result);
}

arm_status cholesky(const_view_t A, view_t L)
{
    if (!valid(A) || !valid(L))
        return ARM_MATH_ARGUMENT_ERROR;
    if (A.rows != A.cols || !same_shape(A, L))
        return ARM_MATH_SIZE_MISMATCH;
    if (overlaps(A, L))
        return ARM_MATH_ARGUMENT_ERROR;
    // CMSIS 只写下三角，清空整个结果以支持复用工作区。
    // CMSIS writes only lower entries; clear output for workspace reuse.
    std::fill_n(L.data, L.rows * L.cols, 0.0f);
    auto a = descriptor(A), result = descriptor(L);
    return arm_mat_cholesky_f32(&a, &result);
}

arm_status solve_lower(const_view_t L, const_view_t B, view_t X)
{
    return triangular(L, B, X, true);
}

arm_status solve_upper(const_view_t U, const_view_t B, view_t X)
{
    return triangular(U, B, X, false);
}

} // namespace matrix
} // namespace pyro
