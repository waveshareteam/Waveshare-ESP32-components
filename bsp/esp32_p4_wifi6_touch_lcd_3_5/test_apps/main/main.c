// SPDX-License-Identifier: Apache-2.0

#include "bsp/esp32_p4_wifi6_touch_lcd_3_5.h"

#if LVGL_VERSION_MAJOR != 9
#error "The LVGL 9 compatibility test must resolve LVGL 9"
#endif

void app_main(void)
{
    void (*rotate)(lv_display_t *, lv_disp_rotation_t) = bsp_display_rotate;
    (void)rotate;
}
