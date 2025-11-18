#include "lr11xx_printf_info.h"
#include "lr1121_common/lr1121_common.h"
#include "stdio.h"

// Print the version information of LR1121
void lora_print_version( const void* context )
{
  lr11xx_system_version_t ver; // Variable to store version information
  if(lr11xx_system_get_version(context, &ver) == 0) // Check if version information is successfully retrieved
  {
    if(ver.type == LR11XX_SYSTEM_VERSION_TYPE_LR1121) // Check if the device is LR1121
    {
      printf( "LR1121 information:\n" ); // Print header for LR1121 information
      printf( "  - Firmware = 0x%04X\n",ver.fw ); // Print firmware version in hexadecimal format
      printf( "  - Hardware = 0x%02X\n", ver.hw ); // Print hardware version in hexadecimal format
    }
    else
      printf("NO LR1121\r\n"); // Print message if the device is not LR1121
  }
}

// Print the temperature information of LR1121
void lora__print_temp( const void* context )
{
  uint16_t temp; // Variable to store raw temperature value
  float t; // Variable to store calculated temperature in Celsius
  lr11xx_system_get_temp(context,&temp); // Retrieve raw temperature value from the device
  t = ((((float)(temp&0x07ff)/2047) * 1.35)-0.7295)*(1000/(-1.7)) + 25; // Calculate temperature in Celsius based on the raw value
  printf( "  LR1121 Temp = %0.2f ℃\n", t); // Print temperature with two decimal places
}

// Print the battery voltage information of LR1121
void lora_print_vbat( const void* context )
{
  uint8_t vbat; // Variable to store raw battery voltage value
  float v; // Variable to store calculated battery voltage in volts
  lr11xx_system_get_vbat(context,&vbat); // Retrieve raw battery voltage value from the device
  v = (((float)(5*vbat)/255)-1)*1.35; // Calculate battery voltage in volts based on the raw value
  printf( "  LR1121 Vbat = %0.2fV\n", v); // Print battery voltage with two decimal places
}
