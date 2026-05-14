#pragma once
#include <stdint.h>
#include <stdbool.h>

void Motion_control_init();
void Motion_control_set_PWM(uint8_t CHx, int PWM);
void Motion_control_run(int error);
bool Motion_control_save_dm_key_none_thresholds(void);
bool Motion_control_uart_input(uint8_t channel_id);
bool Motion_control_uart_output(uint8_t channel_id);
bool Motion_control_uart_take_done(void);
const char *Motion_control_buffer_mode_name(uint8_t channel_id);
uint8_t Motion_control_key_state(uint8_t channel_id);

void MC_PULL_detect_channels_inserted();

// Externy
extern float   MC_PULL_V_OFFSET[4];
extern float   MC_PULL_V_MIN[4];
extern float   MC_PULL_V_MAX[4];
extern uint8_t MC_PULL_pct[4];
extern int8_t  MC_PULL_POLARITY[4];
extern float   MC_DM_KEY_NONE_THRESH[4];
extern bool    filament_channel_inserted[4];

// platformio.ini: -DBAMBU_BUS_AMS_NUM
#ifndef BAMBU_BUS_AMS_NUM
#define BAMBU_BUS_AMS_NUM 0
#endif

// platformio.ini: -DAMS_RETRACT_LEN
#ifndef AMS_RETRACT_LEN
#define AMS_RETRACT_LEN 0.2f
#endif

// platformio.ini: -DBMCU_DM_TWO_MICROSWITCH=1 (DM dual microswitch + autoload assist)
#ifndef BMCU_DM_TWO_MICROSWITCH
#define BMCU_DM_TWO_MICROSWITCH 0
#endif

// platformio.ini: -DBMCU_ONLINE_LED_FILAMENT_RGB=1 (show filament RGB on ONLINE LED when loaded)
#ifndef BMCU_ONLINE_LED_FILAMENT_RGB
#define BMCU_ONLINE_LED_FILAMENT_RGB 0
#endif

// platformio.ini: -DFILAMENT_BUFFER_BMCU_HIGH_FORCE=1
#ifndef FILAMENT_BUFFER_BMCU_HIGH_FORCE
#define FILAMENT_BUFFER_BMCU_HIGH_FORCE 0
#endif

#define UFB_LED_RGB_OFF     0x00u, 0x00u, 0x00u
#define UFB_LED_RGB_WHITE   0x60u, 0x60u, 0x60u
#define UFB_LED_RGB_BLUE    0x00u, 0x00u, 0xFFu
#define UFB_LED_RGB_RED     0xFFu, 0x00u, 0x00u
#define UFB_LED_RGB_GREEN   0x00u, 0xD5u, 0x2Au
#define UFB_LED_RGB_CYAN    0x00u, 0xB0u, 0xFFu
#define UFB_LED_RGB_PURPLE  0xA0u, 0x2Du, 0xFFu

#ifndef motion_control_ams_num
#define motion_control_ams_num BAMBU_BUS_AMS_NUM
#endif

#ifndef motion_control_pull_back_distance
#define motion_control_pull_back_distance AMS_RETRACT_LEN
#endif

#if (BAMBU_BUS_AMS_NUM < 0) || (BAMBU_BUS_AMS_NUM > 3)
#error "BAMBU_BUS_AMS_NUM must be in range 0..3"
#endif
