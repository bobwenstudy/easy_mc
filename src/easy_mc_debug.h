
#ifndef _EASY_MC_DEBUG_H_
#define _EASY_MC_DEBUG_H_

/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

struct easy_mc_debug_vofa_data
{
    float ia;
    float ib;
    float ic;

    float angle;

    float v_bus;
    float i_bus;

    float u_alpha;
    float u_beta;

    float uq;
    float ud;

    float iq_ref;
    float iq;
    float id_ref;
    float id;
    float position;
    float position_ref;
    float speed;
    float speed_ref;

    float tcm1;
    float tcm2;
    float tcm3;

    float debug_0;
    float debug_1;
    float debug_2;
    float debug_3;

    unsigned char tail[4];
};

extern struct easy_mc_debug_vofa_data g_easy_mc_debug_vofa_data;

void easy_mc_vofa_polling_send_data(void);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif

#endif /* _EASY_MC_DEBUG_H_ */