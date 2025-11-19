#ifndef LR11XX_VER_TEMP_H
#define LR11XX_VER_TEMP_H
#include "lr11xx_driver/lr11xx_system.h"
#ifdef __cplusplus
extern "C" {
#endif

void lora_print_version( const void* context );
void lora_print_temp( const void* context );
void lora_print_vbat( const void* context );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_VER_TEMP_H