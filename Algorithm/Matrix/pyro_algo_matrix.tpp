/**
 * @file pyro_algo_matrix.tpp
 * @brief 固定尺寸矩阵运算符实现 / Fixed-shape matrix operators.
 */
#ifndef PYRO_ALGO_MATRIX_TPP
#define PYRO_ALGO_MATRIX_TPP

namespace pyro
{

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> matrix_t<Rows, Cols>::operator+(const matrix_t &B) const
{
    matrix_t result = *this;
    result += B;
    return result;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> matrix_t<Rows, Cols>::operator-(const matrix_t &B) const
{
    matrix_t result = *this;
    result -= B;
    return result;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> matrix_t<Rows, Cols>::operator-() const
{
    matrix_t result;
    for (std::size_t i = 0; i < size(); ++i)
        result[i] = -_values[i];
    return result;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> &matrix_t<Rows, Cols>::operator+=(const matrix_t &B)
{
    for (std::size_t i = 0; i < size(); ++i)
        _values[i] += B[i];
    return *this;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> &matrix_t<Rows, Cols>::operator-=(const matrix_t &B)
{
    for (std::size_t i = 0; i < size(); ++i)
        _values[i] -= B[i];
    return *this;
}

template <std::size_t Rows, std::size_t Cols>
template <std::size_t OtherCols>
matrix_t<Rows, OtherCols>
matrix_t<Rows, Cols>::operator*(const matrix_t<Cols, OtherCols> &B) const
{
    static_assert(Rows > 0 && Cols > 0 && OtherCols > 0,
                  "Cannot multiply empty matrices");
    matrix_t<Rows, OtherCols> result;
    // 固定形状与独立结果保证结构有效；保留异常后端的非有限失败结果。
    // Fixed shapes and independent output ensure validity; backend errors yield NaN.
    if (matrix::multiply(matrix::view(*this), matrix::view(B),
                         matrix::view(result)) != ARM_MATH_SUCCESS)
        result.fill(std::numeric_limits<float>::quiet_NaN());
    return result;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> &
matrix_t<Rows, Cols>::operator*=(const matrix_t<Cols, Cols> &B)
{
    // 先完整计算，再赋值；防止原地写入污染后续乘法。
    // Complete the product before assignment to avoid aliasing input reads.
    *this = *this * B;
    return *this;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> matrix_t<Rows, Cols>::operator*(float scalar) const
{
    matrix_t result = *this;
    result *= scalar;
    return result;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> matrix_t<Rows, Cols>::operator/(float scalar) const
{
    matrix_t result = *this;
    result /= scalar;
    return result;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> &matrix_t<Rows, Cols>::operator*=(float scalar)
{
    for (float &value : _values)
        value *= scalar;
    return *this;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> &matrix_t<Rows, Cols>::operator/=(float scalar)
{
    if (scalar == 0.0f)
        fill(std::numeric_limits<float>::quiet_NaN());
    else
        for (float &value : _values)
            value /= scalar;
    return *this;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Cols, Rows> matrix_t<Rows, Cols>::transpose() const
{
    static_assert(Rows > 0 && Cols > 0, "Cannot transpose empty matrices");
    matrix_t<Cols, Rows> result;
    if (matrix::transpose(matrix::view(*this), matrix::view(result)) !=
        ARM_MATH_SUCCESS)
        result.fill(std::numeric_limits<float>::quiet_NaN());
    return result;
}

template <std::size_t Rows, std::size_t Cols>
matrix_t<Rows, Cols> operator*(float scalar, const matrix_t<Rows, Cols> &A)
{
    return A * scalar;
}

} // namespace pyro
#endif // PYRO_ALGO_MATRIX_TPP
