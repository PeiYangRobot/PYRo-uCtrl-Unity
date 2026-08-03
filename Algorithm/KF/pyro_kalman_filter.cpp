#include "pyro_kalman_filter.h"

using namespace pyro;

kf_t::kf_t(uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    _xhatSize = xhatSize;
    _uSize = uSize;
    _zSize = zSize;
    _is_init = 0;
}

kf_t::~kf_t()
{
    if (_A != nullptr)
        delete _A;
    if (_B != nullptr)
        delete _B;
    if (_H != nullptr)
        delete _H;
    if (_Q != nullptr)
        delete _Q;
    if (_R != nullptr)
        delete _R;
    if (_K != nullptr)
        delete _K;
    if (_P_minus != nullptr)
        delete _P_minus;
    if (_P != nullptr)
        delete _P;
    if (_xhat != nullptr)
        delete _xhat;
    if (_xhat_data != nullptr)
        delete[] _xhat_data;
    if (_xhat_minus != nullptr)
        delete _xhat_minus;
    if (_xhat_minus_data != nullptr)
        delete[] _xhat_minus_data;
    if (_z != nullptr)
        delete _z;
    if (_z_data != nullptr)
        delete[] _z_data;
    if (_u != nullptr)
        delete _u;
    if (_u_data != nullptr)
        delete[] _u_data;
}

status_t kf_t::init(float *A_data, float *B_data, float *H_data, float *Q_data, 
                                                                float *R_data)
{
    if (A_data == nullptr || B_data == nullptr || H_data == nullptr 
                            || Q_data == nullptr || R_data == nullptr)
    {
        return PYRO_PARAM_ERROR;
    }
    if(1 == _is_init)
    {
        return PYRO_ALREADY_INIT;
    }

    /*                 Initialize matrices          */
    /* init state transition matrix */
    _A = new mat;
    CHECK_NEW_RET(_A)
    arm_mat_init_f32(_A, _xhatSize, _xhatSize, A_data);

    /* init control input matrix */
    _B = new mat;
    CHECK_NEW_RET(_B)
    arm_mat_init_f32(_B, _xhatSize, _uSize, B_data);

    /* init measurement matrix */
    _H = new mat;
    CHECK_NEW_RET(_H)
    arm_mat_init_f32(_H, _zSize, _xhatSize, H_data);

    /* init process noise covariance matrix */
    _Q = new mat;
    CHECK_NEW_RET(_Q)
    arm_mat_init_f32(_Q, _xhatSize, _xhatSize, Q_data);

    /* init measurement noise covariance matrix */
    _R = new mat;
    CHECK_NEW_RET(_R)
    arm_mat_init_f32(_R, _zSize, _zSize, R_data);

    /* init Kalman gain matrix */
    _K = new mat;
    CHECK_NEW_RET(_K)
    arm_mat_init_f32(_K, _xhatSize, _zSize, nullptr);

    /* init posterior covariance matrix */
    _P = new mat;
    CHECK_NEW_RET(_P)
    arm_mat_init_f32(_P, _xhatSize, _xhatSize, nullptr);

    /* init residual covariance matrices */
    _P_minus = new mat;
    CHECK_NEW_RET(_P_minus)
    arm_mat_init_f32(_P_minus, _xhatSize, _xhatSize, nullptr);

    /* Initialize  vector */

    /* init posterior state estimate vector */
    _xhat_data = new float[_xhatSize];
    CHECK_NEW_RET(_xhat_data)
    memset(_xhat_data, 0, sizeof(float) * _xhatSize);
    _xhat = new mat;
    CHECK_NEW_RET(_xhat)
    arm_mat_init_f32(_xhat, _xhatSize, 1, _xhat_data);

    /* init prior state estimate vector */
    _xhat_minus_data = new float[_xhatSize];
    CHECK_NEW_RET(_xhat_minus_data)
    memset(_xhat_minus_data, 0, sizeof(float) * _xhatSize);
    _xhat_minus = new mat;
    CHECK_NEW_RET(_xhat_minus)
    arm_mat_init_f32(_xhat_minus, _xhatSize, 1, _xhat_minus_data);

    /* init control input vector */
    _u_data = new float[_uSize];
    CHECK_NEW_RET(_u_data)
    memset(_u_data, 0, sizeof(float) * _uSize);
    _u = new mat;
    CHECK_NEW_RET(_u)
    arm_mat_init_f32(_u, _uSize, 1, _u_data);

    /* init measurement vector */
    _z_data = new float[_zSize];
    CHECK_NEW_RET(_z_data)
    memset(_z_data, 0, sizeof(float) * _zSize);
    _z = new mat;
    CHECK_NEW_RET(_z)
    arm_mat_init_f32(_z, _zSize, 1, _z_data);

    /* set init flag, to avoid re-initialization and call function update 
       before initialization */
    _is_init = 1;
    return PYRO_OK;
}

status_t kf_t::update(float *measure_vec, float *control_vec, float *estimated_ret)
{
    arm_status math_ret;
    if(measure_vec == nullptr || control_vec == nullptr || estimated_ret == nullptr)
    {
        return PYRO_PARAM_ERROR;
    }
    if(!_is_init)
    {
        return PYRO_NOT_FOUND;
    }
    /* temp variance to temporary storage result  */
    float *temp_data_x_size_1 = new float[_xhatSize];
    CHECK_NEW_RET(temp_data_x_size_1);
    mat *temp_mat_x_size_1 = new mat;
    CHECK_NEW_RET(temp_mat_x_size_1);
    arm_mat_init_f32(temp_mat_x_size_1, _xhatSize, 1, 
                                                    temp_data_x_size_1);
    float *temp_data_x_size_2 = new float[_xhatSize];
    CHECK_NEW_RET(temp_data_x_size_2);
    mat *temp_mat_x_size_2 = new mat;
    CHECK_NEW_RET(temp_mat_x_size_2);
    arm_mat_init_f32(temp_mat_x_size_2, _xhatSize, 1, 
                                                    temp_data_x_size_2);

    float *temp_data_xx_size_1 = new float[_xhatSize * _xhatSize];
    CHECK_NEW_RET(temp_data_xx_size_1);
    mat *temp_mat_xx_size_1 = new mat;
    CHECK_NEW_RET(temp_mat_xx_size_1);
    arm_mat_init_f32(temp_mat_xx_size_1, _xhatSize, 
                            _xhatSize, temp_data_xx_size_1);

    float *temp_data_xx_size_2 = new float[_xhatSize * _xhatSize];
    CHECK_NEW_RET(temp_data_xx_size_2);
    mat *temp_mat_xx_size_2 = new mat;
    CHECK_NEW_RET(temp_mat_xx_size_2);
    arm_mat_init_f32(temp_mat_xx_size_2, _xhatSize, 
                            _xhatSize, temp_data_xx_size_2);

     /* update measurement vector and control input vector */
    memcpy(_z_data, measure_vec, sizeof(float) * _zSize);
    memcpy(_u_data, control_vec, sizeof(float) * _uSize);

    /* update posterior estimate state: 
      hat(x)_{k}^- = A * hat(x)_{k-1} + B * u_{k-1} */
    math_ret = arm_mat_mult_f32(_A, _xhat, temp_mat_x_size_1);
    CHECK_ARM_MATH_RET(math_ret);
    math_ret = arm_mat_mult_f32(_B, _u, temp_mat_x_size_2);
    CHECK_ARM_MATH_RET(math_ret);
    math_ret = arm_mat_add_f32(temp_mat_x_size_1, temp_mat_x_size_2,
                                                         _xhat_minus);
    CHECK_ARM_MATH_RET(math_ret);
                                                        
    /* update prior estimate error covariance:
        P_{k}^- = A * P_{k-1} * A^T + Q */
    math_ret = arm_mat_mult_f32(_A, _P, 
                                    temp_mat_xx_size_1);
    CHECK_ARM_MATH_RET(math_ret);
    math_ret = arm_mat_mult_f32(temp_mat_xx_size_1, _A, 
                                    temp_mat_xx_size_2);
    CHECK_ARM_MATH_RET(math_ret);
    math_ret = arm_mat_add_f32(temp_mat_xx_size_2, _Q, 
                                    _P_minus);
    CHECK_ARM_MATH_RET(math_ret);

    /* update Kalman gain:
        K_{k} = P_{k}^- * H^T * inv(H * P_{k}^- * H^T + R) */
    math_ret = arm_mat_mult_f32(_H, _P_minus, temp_mat_xx_size_1);
   return PYRO_OK; 
}