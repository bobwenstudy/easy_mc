#include <stdlib.h>
#include <math.h>

#include "easy_mc.h"

#define UBUS           1.0f
#define UBUS_SVPWM_MAX (UBUS / EASY_MC_MATH_SQRT3)
// #define UBUS_SVPWM_MAX (UBUS)

static void SVPWM_Process(float u_alpha, float u_beta, float v_dc, float t_s, float time_array[3])
{
    float X, Y, Z;
    float Tfirst, Tsecond;
    float Vref1, Vref2, Vref3;
    float Ta, Tb, Tc;
    float Tcm1, Tcm2, Tcm3;
    float temp0, temp1, temp2, temp3;
    int N;

    easy_mc_hw_debug_io_control(0, 1);

    // FOC_TEST_PRINTF("u_alpha = %f, u_beta = %f\n"
    //     , u_alpha, u_beta);

    // sqrt(3)/2*u_alpha
    temp0 = EASY_MC_MATH_SQRT3_2 * u_alpha;
    temp1 = u_beta / 2;

    // FOC_TEST_PRINTF("temp0 = %f, temp1 = %f\n"
    //     , temp0, temp1);

    Vref1 = u_beta;
    Vref2 = temp0 - temp1;
    Vref3 = -temp0 - temp1;

    // FOC_TEST_PRINTF("Vref1 = %f, Vref2 = %f, Vref3 = %f\n", Vref1, Vref2, Vref3);

    // N = 4C + 2B + A
    N = ((Vref3 > 0) << 2) + ((Vref2 > 0) << 1) + ((Vref1 > 0));

    // FOC_TEST_PRINTF("N = %d\n", N);

    temp2 = EASY_MC_MATH_SQRT3 * t_s / v_dc;
    // FOC_TEST_PRINTF("temp2 = %f\n"
    //     , temp2);

    X = temp2 * u_beta;
    Y = temp2 * (temp0 + temp1);
    Z = temp2 * (-temp0 + temp1);

    // FOC_TEST_PRINTF("X = %f, Y = %f, Z = %f\n", X, Y, Z);

    switch (N)
    {
    case 1:
        Tfirst = Z;
        Tsecond = Y;
        break;
    case 2:
        Tfirst = Y;
        Tsecond = -X;
        break;
    case 3:
        Tfirst = -Z;
        Tsecond = X;
        break;
    case 4:
        Tfirst = -X;
        Tsecond = Z;
        break;
    case 5:
        Tfirst = X;
        Tsecond = -Y;
        break;
    case 6:
        Tfirst = -Y;
        Tsecond = -Z;
        break;
    default:
        // N = 0, 7
        Tfirst = 0;
        Tsecond = 0;
        break;
    }

    temp3 = (Tfirst + Tsecond);
    if (temp3 > t_s)
    {
        // FOC_TEST_PRINTF("error, Tfirst = %f, Tsecond = %f, temp3 = %f\n"
        //     , Tfirst, Tsecond, temp3);
        Tfirst = t_s * Tfirst / temp3;
        Tsecond = t_s * Tsecond / temp3;
    }

    Ta = (t_s - Tfirst - Tsecond) / 4;
    Tb = Ta + Tfirst / 2;
    Tc = Tb + Tsecond / 2;

    // FOC_TEST_PRINTF("Tfirst = %f, Tsecond = %f, Ta = %f, Tb = %f, Tc = %f\n"
    //     , Tfirst, Tsecond, Ta, Tb, Tc);

    switch (N)
    {
    case 1:
        Tcm1 = Tb;
        Tcm2 = Ta;
        Tcm3 = Tc;
        break;
    case 2:
        Tcm1 = Ta;
        Tcm2 = Tc;
        Tcm3 = Tb;
        break;
    case 3:
        Tcm1 = Ta;
        Tcm2 = Tb;
        Tcm3 = Tc;
        break;
    case 4:
        Tcm1 = Tc;
        Tcm2 = Tb;
        Tcm3 = Ta;
        break;
    case 5:
        Tcm1 = Tc;
        Tcm2 = Ta;
        Tcm3 = Tb;
        break;
    case 6:
        Tcm1 = Tb;
        Tcm2 = Tc;
        Tcm3 = Ta;
        break;
    default:
        // N = 0, 7
        Tcm1 = 0;
        Tcm2 = 0;
        Tcm3 = 0;
        break;
    }

    time_array[0] = Tcm1;
    time_array[1] = Tcm2;
    time_array[2] = Tcm3;

    easy_mc_hw_debug_io_control(0, 0);
}

void foc_svpwm_test(void)
{
    float u_d = 0.0f;
    float u_q = UBUS_SVPWM_MAX;
    float v_dc = UBUS;
    float t_s = 1.0f;

    float time_array[3] = {0.0f, 0.0f, 0.0f};

    float angle_el = 0.0f;

    float u_alpha;
    float u_beta;

    while (angle_el < 2 * EASY_MC_MATH_PI)
    {
        // rev-park converter, https://zhuanlan.zhihu.com/p/12003413490
        u_alpha = u_d * EASY_MC_MATH_COS(angle_el) - u_q * EASY_MC_MATH_SIN(angle_el);
        u_beta = u_d * EASY_MC_MATH_SIN(angle_el) + u_q * EASY_MC_MATH_COS(angle_el);

        SVPWM_Process(u_alpha, u_beta, v_dc, t_s, time_array);

        EASY_MC_LOG_INF("angle_el = %f, u_alpha = %f, u_beta = %f, v_dc = %f, t_s = %f, Tcm1: %f, Tcm2: %f, Tcm3: %f\n", angle_el, u_alpha, u_beta, v_dc, t_s,
                        time_array[0], time_array[1], time_array[2]);

        angle_el += 0.1f;
    }
}

struct foc_test_vofa_data
{
    float u_d;
    float u_q;
    float angle_el;
    float u_alpha;
    float u_beta;

    float t_cm1;
    float t_cm2;
    float t_cm3;

    unsigned char tail[4];
};

void foc_svpwm_wave_test(void)
{
    float u_d = 0.0f;
    float u_q = UBUS_SVPWM_MAX;
    float v_dc = UBUS;
    float t_s = 1.0f;

    float time_array[3] = {0.0f, 0.0f, 0.0f};

    float angle_el = 0.0f;

    float u_alpha;
    float u_beta;

    struct foc_test_vofa_data vofa_data = {.tail = {0x00, 0x00, 0x80, 0x7f}};

    while (1)
    {
        float sin_angle_el;
        float cos_angle_el;
        easy_mc_hw_debug_io_control(1, 1);
        angle_el += 0.001f;

        if (angle_el > 2 * EASY_MC_MATH_PI)
        {
            angle_el = 0.0f;
        }

        // rev-park converter, https://zhuanlan.zhihu.com/p/12003413490
        sin_angle_el = EASY_MC_MATH_SIN(angle_el);
        cos_angle_el = EASY_MC_MATH_COS(angle_el);

        u_alpha = u_d * cos_angle_el - u_q * sin_angle_el;
        u_beta = u_d * sin_angle_el + u_q * cos_angle_el;

        SVPWM_Process(u_alpha, u_beta, v_dc, t_s, time_array);

        easy_mc_hw_debug_io_control(1, 0);

        // Set data to vofa_data
        vofa_data.u_d = u_d;
        vofa_data.u_q = u_q;
        vofa_data.angle_el = angle_el;
        vofa_data.u_alpha = u_alpha;
        vofa_data.u_beta = u_beta;
        vofa_data.t_cm1 = time_array[0];
        vofa_data.t_cm2 = time_array[1];
        vofa_data.t_cm3 = time_array[2];

        easy_mc_hw_vofa_debug_out((uint8_t *)&vofa_data, sizeof(vofa_data));

        // Avoid uart send too fast
        HAL_Delay(1);
    }
}

void foc_test(void)
{
    foc_svpwm_test();

    foc_svpwm_wave_test();
}

void app_main(void)
{
    EASY_MC_LOG_INF("svpwm_test\r\n");
    foc_test();
}

void app_main_loop(void)
{
}
