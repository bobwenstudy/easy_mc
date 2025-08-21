#include <math.h>

#include "easy_mc.h"
#include "easy_mc_debug.h"

struct easy_mc_debug_vofa_data g_easy_mc_debug_vofa_data = {.tail = {0x00, 0x00, 0x80, 0x7f}};

void easy_mc_vofa_polling_send_data(void)
{
    easy_mc_hw_vofa_debug_out((uint8_t *)&g_easy_mc_debug_vofa_data, sizeof(g_easy_mc_debug_vofa_data));
}
