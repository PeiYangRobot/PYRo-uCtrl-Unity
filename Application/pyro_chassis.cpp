#include "pyro_core_def.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"
#include "arm_math.h"
#include "pyro_rc_hub.h"

using namespace pyro;

#define R_MOTOR1_OFFSET 2.66f
#define R_MOTOR2_OFFSET 3.979092653f
#define L_MOTOR1_OFFSET 0.0f
#define L_MOTOR2_OFFSET 0.0f

//the cofficients for phi calculation
#define PHI_K0 22506.0f
#define PHI_K1 475976946.0f
#define PHI_K2 296955904.0f
#define PHI_K3 179021042.0f
#define PHI_K4 18922.0f

//the cofficients for position in polar coordinates
#define POLAR_K0 236976927.0f
#define POLAR_K1 21059.0f
#define POLAR_K2 946100000.0f
#define POLAR_K3 100000.0f

//the cofficients for transformation matrix
#define TRANS_K0 21059.0f
#define TRANS_K1 100000.0f
//the cofficients of lqr gain
//kp_11
#define LQR_KP_11_K0 0.0f
#define LQR_KP_11_K1 0.0f
#define LQR_KP_11_K2 0.0f
//kp_12
#define LQR_KP_12_K0 0.0f
#define LQR_KP_12_K1 0.0f
#define LQR_KP_12_K2 0.0f
//kp_13
#define LQR_KP_13_K0 0.0f
#define LQR_KP_13_K1 0.0f
#define LQR_KP_13_K2 0.0f
//kp_14
#define LQR_KP_14_K0 0.0f
#define LQR_KP_14_K1 0.0f
#define LQR_KP_14_K2 0.0f
//kp_15
#define LQR_KP_15_K0 0.0f
#define LQR_KP_15_K1 0.0f
#define LQR_KP_15_K2 0.0f
//kp_16
#define LQR_KP_16_K0 0.0f
#define LQR_KP_16_K1 0.0f
#define LQR_KP_16_K2 0.0f
//kp_21
#define LQR_KP_21_K0 0.0f
#define LQR_KP_21_K1 0.0f
#define LQR_KP_21_K2 0.0f
//kp_22
#define LQR_KP_22_K0 0.0f
#define LQR_KP_22_K1 0.0f
#define LQR_KP_22_K2 0.0f
//kp_23
#define LQR_KP_23_K0 0.0f
#define LQR_KP_23_K1 0.0f
#define LQR_KP_23_K2 0.0f
//kp_24
#define LQR_KP_24_K0 0.0f
#define LQR_KP_24_K1 0.0f
#define LQR_KP_24_K2 0.0f
//kp_25
#define LQR_KP_25_K0 0.0f
#define LQR_KP_25_K1 0.0f
#define LQR_KP_25_K2 0.0f
//kp_26
#define LQR_KP_26_K0 0.0f
#define LQR_KP_26_K1 0.0f
#define LQR_KP_26_K2 0.0f
#define BARYCENTER_K0 0.0332911f
#define BARYCENTER_K2 0.2033493f
#define BARYCENTER_K1 -0.5908261f
const pyro::dr16_drv_t::dr16_ctrl_t *rc_data;
ins_drv_t* ins;

dm_motor_drv_t* r_motor1;
dm_motor_drv_t* r_motor2;
dm_motor_drv_t* l_motor1;
dm_motor_drv_t* l_motor2;
dji_m3508_motor_drv_t* r_wheel;
dji_m3508_motor_drv_t* l_wheel;

float yaw, pitch, roll;
float g_yaw, g_pitch, g_roll;
float acc_x, acc_y, acc_z;

float r_theta1, r_theta2, l_theta1, l_theta2;
float r_phi1, r_phi2, l_phi1, l_phi2;
float r_alpha, l_alpha;
float r_l, l_l;

//state variables
float x, d_x;
float beta, d_beta;
float gamma, d_gamma;
//motor output torque
float r_T[2],r_F[2];
//vmc output force and torque
float l_T[2],l_F[2];
//vlaue of the matrix which transfoms vmc output to motor output
float r_T_val[4], l_T_val[4];
//state variables vectors
float r_X[6], l_X[6];
//value of lqr gain matrix
float r_K_val[12], l_K_val[12];
//transformation matrix instance
arm_matrix_instance_f32 r_T_mat, l_T_mat;
//lqr gain matrix instance
arm_matrix_instance_f32 r_K_mat, l_K_mat;

rc_drv_t *dr16_drv;

float calc_barycenter(float leg_length);

status_t enable(void);
status_t disable(void);
status_t update_feedback(void);
status_t kinomatic_solve(float theta1, float theta2,
                         float* phi1, float* phi2,
                         float* alpha, float* l);
                         
void update_transform_matrix(float phi1, float phi2,
                             float theta1, float theta2,
                             float alpha, float l, float* T);
status_t update_lqr_gain_matrix(float l, float* K_val);

extern "C" void pyro_chassis(void* argument)
{
    //Init

    //Init RC
    dr16_drv = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16);

    //Init INS
    ins = ins_drv_t::get_instance();

    //Init matrix
    arm_mat_init_f32(&r_T_mat, 2, 2, r_T_val);
    arm_mat_init_f32(&l_T_mat, 2, 2, l_T_val);
    arm_mat_init_f32(&r_K_mat, 2, 6, r_K_val);
    arm_mat_init_f32(&l_K_mat, 2, 6, l_K_val);

    //Init motors
    r_motor1 = new dm_motor_drv_t(0x01, 0x11,
                                                     can_hub_t::can1);
    r_motor1->set_rotate_range(-54.0f, 54.0f);
    r_motor1->set_position_range(-12.5f, 12.5f);
    r_motor1->set_rotate_range(-45.0f, 45.0f);
    r_motor2 = new dm_motor_drv_t(0x02, 0x12,
                                                     can_hub_t::can1);
    r_motor2->set_rotate_range(-54.0f, 54.0f);
    r_motor2->set_position_range(-12.5f, 12.5f);
    r_motor2->set_rotate_range(-45.0f, 45.0f);
    l_motor1 = new dm_motor_drv_t(0x03, 0x13,
                                                     can_hub_t::can2);
    l_motor1->set_rotate_range(-54.0f, 54.0f);
    l_motor1->set_position_range(-12.5f, 12.5f);
    l_motor1->set_rotate_range(-45.0f, 45.0f);
    l_motor2 = new dm_motor_drv_t(0x04, 0x14,
                                                     can_hub_t::can2);
    l_motor2->set_rotate_range(-54.0f, 54.0f);
    l_motor2->set_position_range(-12.5f, 12.5f);
    l_motor2->set_rotate_range(-45.0f, 45.0f);

    r_wheel = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, 
                                                        can_hub_t::can1);
    l_wheel = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, 
                                                        can_hub_t::can2);
    while(1)
    {   
        //update date of motor and rc
        update_feedback();
        //kinomatic solve
        kinomatic_solve(r_theta1, r_theta2, 
                        &r_phi1, &r_phi2, 
                        &r_alpha, &r_l);
        kinomatic_solve(l_theta1, l_theta2, 
                        &l_phi1, &l_phi2,
                        &l_alpha, &l_l);
        //update lqr gain matrix
        update_lqr_gain_matrix(r_l, r_K_val);
        update_lqr_gain_matrix(l_l, l_K_val);

        //calculate the vmc target force and torque



        
        // VMC transform
        r_F[0] = 1.0f;
        r_F[1] = 1.0f;
        
        update_transform_matrix(r_phi1, r_phi2, 
                              r_theta1, r_theta2,
                               r_alpha, r_l, r_T_val);
        arm_mat_vec_mult_f32(&r_T_mat, r_F, r_T);
            
        vTaskDelay(1);

    }
}

status_t enable(void)
{
    status_t ret;

    ret = r_wheel->enable();
    CHECK_PYRO_RET(ret);
    ret = l_wheel->enable();
    CHECK_PYRO_RET(ret);
    
    if(dm_motor_drv_t::ok != r_motor1->get_error_code())
    {
        ret = r_motor1->clear_error();
        CHECK_PYRO_RET(ret);
    }
    ret = r_motor1->enable();
    CHECK_PYRO_RET(ret);
    if(dm_motor_drv_t::ok != r_motor2->get_error_code())
    {
        ret = r_motor2->clear_error();
        CHECK_PYRO_RET(ret);
    }
    if(dm_motor_drv_t::ok != l_motor1->get_error_code())
    {
        ret = l_motor1->clear_error();
        CHECK_PYRO_RET(ret);
    }
    ret = l_motor1->enable();
    CHECK_PYRO_RET(ret);
    if(dm_motor_drv_t::ok != l_motor2->get_error_code())
    {
        ret = l_motor2->clear_error();
        CHECK_PYRO_RET(ret);
    }
    ret = l_motor2->enable();
    CHECK_PYRO_RET(ret);

    return PYRO_OK;
}

status_t disable(void)
{
    status_t ret;

    ret = r_wheel->disable();
    CHECK_PYRO_RET(ret)
    ret = l_wheel->disable();
    CHECK_PYRO_RET(ret);
    ret = r_motor1->disable();
    CHECK_PYRO_RET(ret);
    ret = r_motor2->disable();
    CHECK_PYRO_RET(ret);
    ret = l_motor1->disable();
    CHECK_PYRO_RET(ret);
    ret = l_motor2->disable();
    CHECK_PYRO_RET(ret);
    
    return PYRO_OK;
}


status_t update_feedback(void)
{
    status_t ret;
    //update RC data
    read_scope_lock rc_read_lock(dr16_drv->get_lock());
    rc_data = static_cast<const pyro::dr16_drv_t::dr16_ctrl_t *>(
                                                            dr16_drv->read());
    //update INS data
    ins->get_angles_n(&yaw, &pitch, &roll);
    ins->get_gyro_n(&g_yaw, &g_pitch, &g_roll);

    //update motor feedback
    ret = r_motor1->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = r_motor2->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = l_motor1->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = l_motor2->update_feedback();
    CHECK_PYRO_RET(ret);

    ret = r_wheel->update_feedback();
    CHECK_PYRO_RET(ret);
    ret = l_wheel->update_feedback();
    CHECK_PYRO_RET(ret);

    //update theta of both legs, due to the motor installation direction,
    //right leg theta is opposite to motot feedback.
    r_theta1 = -(r_motor1->get_current_position()) + R_MOTOR1_OFFSET;
    r_theta2 = -(r_motor2->get_current_position()) + R_MOTOR2_OFFSET;
    l_theta1 =  (l_motor1->get_current_position()) + L_MOTOR1_OFFSET;
    l_theta2 =  (l_motor2->get_current_position()) + L_MOTOR2_OFFSET;

    return PYRO_OK;
}

status_t kinomatic_solve(float theta1, float theta2,
                         float* phi1, float* phi2,
                         float* alpha, float* l)
{
    arm_status ret;
    //1.Calculate phi1, phi2
    float temp1, temp2;  //two temporary variables to store claculation results
    //1.1 calculate the molecule of phi1 and phi2
    ret = arm_sqrt_f32(  (PHI_K1 - PHI_K2*arm_cos_f32(theta1 - theta2) - 
                    PHI_K3*arm_cos_f32(2*theta1 - 2*theta2)), &temp1);
    CHECK_ARM_MATH_RET(ret);
    temp1 = PHI_K0*arm_sin_f32(theta1) - PHI_K0*arm_sin_f32(theta2)+temp1;
    //1.2 calculate phi1
    ret = arm_atan2_f32(temp1, (PHI_K0*arm_cos_f32(theta1)
         - PHI_K0*arm_cos_f32(theta2) + PHI_K4*arm_cos_f32(theta1 - theta2)
         - PHI_K4), &temp2);
    CHECK_ARM_MATH_RET(ret);
    *phi1 = (2 * temp2);
    //1.3 calculate phi2, the diffrenece with phi1 is the sign of K4 cos theta1 
    // and constant K4
    ret = arm_atan2_f32(temp1, (PHI_K0*arm_cos_f32(theta1)
         - PHI_K0*arm_cos_f32(theta2) - PHI_K4*arm_cos_f32(theta1 - theta2)
         + PHI_K4), &temp2);
    CHECK_ARM_MATH_RET(ret);
    *phi2 = (2 * temp2);  

    //2. Calculate the position in polar coordinates
    //2.1 calculate alpha
    ret = arm_atan2_f32((POLAR_K0*arm_sin_f32(*phi1))/POLAR_K2 
        + (POLAR_K1*arm_sin_f32(theta1))/POLAR_K3,
    (POLAR_K0*arm_cos_f32(*phi1))/POLAR_K2 
        + (POLAR_K1*arm_cos_f32(theta1))/POLAR_K3, &temp2);
    CHECK_ARM_MATH_RET(ret);
    *alpha = temp2;
    //2.2 calculate l 
    temp1 = (POLAR_K0*arm_cos_f32(*phi1))/POLAR_K2 + 
                            (POLAR_K1*arm_cos_f32(theta1))/POLAR_K3;
    temp1 = temp1 * temp1;
    temp2 = (POLAR_K0*arm_sin_f32(*phi1))/POLAR_K2 +
                            (POLAR_K1*arm_sin_f32(theta1))/POLAR_K3;
    temp2 = temp2 * temp2;
    ret = arm_sqrt_f32(temp1 + temp2, l);
    CHECK_ARM_MATH_RET(ret);
    return PYRO_OK;

}
 
void update_transform_matrix(float phi1, float phi2,
                             float theta1, float theta2,
                             float alpha, float l, float* T)
{
    T[0] = (TRANS_K0*arm_cos_f32(phi2)*arm_sin_f32(alpha)
        *arm_sin_f32(phi1 - theta1))/(TRANS_K1*arm_sin_f32(phi1 - phi2)) 
        - (TRANS_K0*arm_cos_f32(alpha)*arm_sin_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(TRANS_K1*arm_sin_f32(phi1 - phi2));
    T[1] = -((TRANS_K0*arm_cos_f32(alpha)*arm_cos_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(TRANS_K1*arm_sin_f32(phi1 - phi2)) 
        - (TRANS_K0*arm_sin_f32(alpha)*arm_sin_f32(phi2)
        *arm_sin_f32(phi1 - theta1))/(TRANS_K1*arm_sin_f32(phi1 - phi2)))/l;
    T[2] = (TRANS_K0*arm_cos_f32(alpha)*arm_sin_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(TRANS_K1*arm_sin_f32(phi1 - phi2)) 
        - (TRANS_K0*arm_cos_f32(phi1)*arm_sin_f32(alpha)
        *arm_sin_f32(phi2 - theta2))/(TRANS_K1*arm_sin_f32(phi1 - phi2));
    T[3] = ((TRANS_K0*arm_cos_f32(alpha)*arm_cos_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(TRANS_K1*arm_sin_f32(phi1 - phi2)) 
        - (TRANS_K0*arm_sin_f32(alpha)*arm_sin_f32(phi1)
        *arm_sin_f32(phi2 - theta2))/(TRANS_K1*arm_sin_f32(phi1 - phi2)))/l; 

}

status_t update_lqr_gain_matrix(float l, float* K_val)
{
    if(NULL == K_val)
    {
        return PYRO_PARAM_ERROR;
    }
    K_val[0]  = LQR_KP_11_K0 + LQR_KP_11_K1 * l + LQR_KP_11_K2 * l * l;
    K_val[1]  = LQR_KP_12_K0 + LQR_KP_12_K1 * l + LQR_KP_12_K2 * l * l;
    K_val[2]  = LQR_KP_13_K0 + LQR_KP_13_K1 * l + LQR_KP_13_K2 * l * l;
    K_val[3]  = LQR_KP_14_K0 + LQR_KP_14_K1 * l + LQR_KP_14_K2 * l * l;
    K_val[4]  = LQR_KP_15_K0 + LQR_KP_15_K1 * l + LQR_KP_15_K2 * l * l;
    K_val[5]  = LQR_KP_16_K0 + LQR_KP_16_K1 * l + LQR_KP_16_K2 * l * l;
    K_val[6]  = LQR_KP_21_K0 + LQR_KP_21_K1 * l + LQR_KP_21_K2 * l * l;
    K_val[7]  = LQR_KP_22_K0 + LQR_KP_22_K1 * l + LQR_KP_22_K2 * l * l;
    K_val[8]  = LQR_KP_23_K0 + LQR_KP_23_K1 * l + LQR_KP_23_K2 * l * l;
    K_val[9]  = LQR_KP_24_K0 + LQR_KP_24_K1 * l + LQR_KP_24_K2 * l * l;
    K_val[10] = LQR_KP_25_K0 + LQR_KP_25_K1 * l + LQR_KP_25_K2 * l * l;
    K_val[11] = LQR_KP_26_K0 + LQR_KP_26_K1 * l + LQR_KP_26_K2 * l * l;
    return PYRO_OK;
}



float calc_barycenter(float leg_length)
{
    return BARYCENTER_K0 + BARYCENTER_K1 * leg_length + BARYCENTER_K2 * leg_length * leg_length;
}