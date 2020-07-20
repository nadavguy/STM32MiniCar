#ifndef __INCLUDES_H__
#define __INCLUDES_H__

//-----------------------------------------------------------------------------
//#define __cplusplus
#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

#define ARM_MATH_CM7
#define __FPU_PRESENT 1


// General include
//#include "stm32f7xx_ll_rcc.h"
#include <string.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
//#include "typedefs.h"

// StdPeriph, HW, RTOS and Modules
//#include "__modules_common.h"

// File_System , SDIO
//#include "../__FAT/high_level_sd.h"
//#include "../__FAT/diskio.h"
//#include "../__FAT/ff.h"
//#include "../__FAT/fs_display.h"

#include "diskio.h"
#include "ff.h"
#include "fs_display.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

// IMU
//#include "an_packet_protocol.h"
//#include "orientus_packets.h"
//#include "imu.h"

// Apllication
//#include "watchdog_server.h"
//#include "rc.h"
//
//#include "application.h"
//#include "usart.h"
//#include "timer.h"
//#include "adc_dma.h"
//#include "eeprom_app.h"
//#include "emergency_check.h"
//#include "terminal.h"
//#include "telemetry.h"
//#include "imu.h"
//#include "watchdog_server.h"
//#include "rc.h"
//#include "led_WS2811.h"
//#include "log.h"
//#include "pyro.h"
//#include "buzzer.h"
//#include "discharge.h"
//
//#include "ring_buffer.h"
//#include "str_util.h"
//#include "__modules_common.h"

#include "arm_math.h"

//-----------------------------------------------------------------------------
//extern CRC_HandleTypeDef hcrc;
//extern ADC_HandleTypeDef hadc1;
//extern DMA_HandleTypeDef hdma_adc1;
//extern DMA_HandleTypeDef hdma_tim4_ch3;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim12;
//extern TIM_HandleTypeDef htim14;
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart6;
//extern U32 counter_1sec;
//
//#define M_PI   3.14159265358979323846264338327950288
//
//// For startup check
//// not used
//
//#define PC_DIS        (1) // discharge pin signal
//#define PC_ATT        (2) //Move system to an orientation different than the installation orientation	Message should be <0,0,2,x>
//#define PC_SD_ERR      (4) //Remove SD card	Message should be <0,0,4,x>
////#define PC_SENSOR     (8)
//#define PC_NO_GPS       (8) //Disconnect GPS antenna	Message should be <0,0,8,x>
//#define PC_PYRO       (16) //Remove Pyro dummy	Message should be <0,0,16,x>
//#define PC_WDG        (32) //Disconnect WD	Message should be <0,0,32,x>
//#define PC_IMU        (64) //Disconnect SenseAir/Orientus	Message should be <0,0,64,x>
//#define PC_MAN        (128) //Change triggering mode to manual	Message should be <1,0,128,x>
////Not in use	Message should be <0,0,128,x>
////#define PC_12V_!        (128)
////#define PC_5V        (256)
//#define PC_LINK_ERR       (256) //Remove Sim card/ disconnect GSM antenna (Datalink error)	Message should be <0,0,256,x>
//#define PC_EMRGENCY   (512) //Disconnect backup battery 	Message should be <0,0,512,x>
//#define PC_MAIN_BAT   (1024) //Disconnect main battery (power up from backup battery)	Message should be <0,0,1024,x>
////#define PC_CAPACITOR  (2048)
//#define PC_RC_PYRO    (4096) //Disconnect Trigger RC channel	Message should be <0,0,4096,x>
//#define PC_RC_ARM     (8192) //Disconnect Arm RC channel	Message should be <0,0,8192,x>
//#define PC_FIRE        (16384) //Activate Trigger RC channel	Message should be <0,0,16384,x>
//#define PC_ARM       (32768) //Activate Arm RC channel 	Message should be <0,0,32768,x>
//#define PC_FW       (65536) //Create a firmware version mismatch	Message should be <0,0,65536,x>



// IMU configuration
// orientus = 11 = (8+2+1)
// AenseAir = 19 = (16+2+1)  1+2+4+16+32=55
//#define ATT_CHK            (1)
//#define FALL_DETECTION_CHK (2)
//#define ATT_RATE_CHK       (4)
//#define IMU_ORIENTUS       (8)
//#define IMU_SENSEAIR       (16)
//#define HEIGHT_CHK         (32)
//
////RC configuration
//#define RC_TRIG         (1)
//#define RC_ARM_DISARM   (2)
//#define RC_TRIG_OVERRIDE (4)

#define TELEM_SEND_CNT 200
#define UID1 (*(__I uint32_t *) 0x1FF0F420)
#define UID2 (*(__I uint32_t *) 0x1FF0F424)
#define UID3 (*(__I uint32_t *) 0x1FF0F428)
#define UID_MAC 0x1FF0F420

//-----------------------------------------------------------------------------


/* Definition for QSPI clock resources */
//#define QSPI_CLK_ENABLE()          __HAL_RCC_QSPI_CLK_ENABLE()
//#define QSPI_CLK_DISABLE()         __HAL_RCC_QSPI_CLK_DISABLE()
//
//#define QSPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
//#define QSPI_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
//#define QSPI_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
//#define QSPI_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
//#define QSPI_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
//#define QSPI_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
//
//#define QSPI_FORCE_RESET()         __HAL_RCC_QSPI_FORCE_RESET()
//#define QSPI_RELEASE_RESET()       __HAL_RCC_QSPI_RELEASE_RESET()
//
///* Definition for QSPI Pins */
//#define QSPI_CS_PIN                GPIO_PIN_6
//#define QSPI_CS_PIN_AF             GPIO_AF10_QUADSPI
//#define QSPI_CS_GPIO_PORT          GPIOB
//
//#define QSPI_CLK_PIN               GPIO_PIN_2
//#define QSPI_CLK_PIN_AF            GPIO_AF9_QUADSPI
//#define QSPI_CLK_GPIO_PORT         GPIOB
//
//#define QSPI_D0_PIN                GPIO_PIN_9
//#define QSPI_D0_PIN_AF             GPIO_AF9_QUADSPI
//#define QSPI_D0_GPIO_PORT          GPIOC
//
//#define QSPI_D1_PIN                GPIO_PIN_10
//#define QSPI_D1_PIN_AF             GPIO_AF9_QUADSPI
//#define QSPI_D1_GPIO_PORT          GPIOC
//
//#define QSPI_D2_PIN                GPIO_PIN_2
//#define QSPI_D2_PIN_AF             GPIO_AF9_QUADSPI
//#define QSPI_D2_GPIO_PORT          GPIOE
//
//#define QSPI_D3_PIN                GPIO_PIN_13
//#define QSPI_D3_PIN_AF             GPIO_AF9_QUADSPI
//#define QSPI_D3_GPIO_PORT          GPIOD
//
///* MX25L51245G Macronix memory */
//
////#define QSPI_FLASH_SIZE                  0x4000000 /* 512 MBits => 64MBytes */
////#define QSPI_SECTOR_SIZE                 0x10000   /* 1024 sectors of 64KBytes */
////#define QSPI_SUBSECTOR_SIZE              0x1000    /* 16384 subsectors of 4kBytes */
//#define QSPI_PAGE_SIZE                   0x100     /* 262144 pages of 256 bytes */
//
//#define QSPI_DUMMY_CYCLES_READ_QUAD      3
//#define QSPI_DUMMY_CYCLES_READ           8
//#define QSPI_DUMMY_CYCLES_READ_QUAD_IO   10
//#define QSPI_DUMMY_CYCLES_READ_DTR       6
//#define QSPI_DUMMY_CYCLES_READ_QUAD_DTR  8
//
//#define QSPI_BULK_ERASE_MAX_TIME         600000
//#define QSPI_SECTOR_ERASE_MAX_TIME       2000
//#define QSPI_SUBSECTOR_ERASE_MAX_TIME    800
//
///**
//  * @brief  QSPI Commands
//  */
///* Reset Operations */
//#define RESET_ENABLE_CMD                     0x66
//#define RESET_MEMORY_CMD                     0x99
//
///* Identification Operations */
//#define READ_ID_CMD                          0x9F
//#define MULTIPLE_IO_READ_ID_CMD              0xAF
//#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A
//
///* Read Operations */
//#define READ_CMD                             0x03
//#define READ_4_BYTE_ADDR_CMD                 0x13
//
//#define FAST_READ_CMD                        0x0B
//#define FAST_READ_DTR_CMD                    0x0D
//#define FAST_READ_4_BYTE_ADDR_CMD            0x0C
//
//#define DUAL_OUT_FAST_READ_CMD               0x3B
//#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x3C
//
//#define DUAL_INOUT_FAST_READ_CMD             0xBB
//#define DUAL_INOUT_FAST_READ_DTR_CMD         0xBD
//#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xBC
//
//#define QUAD_OUT_FAST_READ_CMD               0x6B
//#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x6C
//
//#define QUAD_INOUT_FAST_READ_CMD             0xEB
//#define QUAD_INOUT_FAST_READ_DTR_CMD         0xED
//#define QPI_READ_4_BYTE_ADDR_CMD             0xEC
//
///* Write Operations */
//#define WRITE_ENABLE_CMD                     0x06
//#define WRITE_DISABLE_CMD                    0x04
//
///* Register Operations */
//#define READ_STATUS_REG_CMD                  0x05
//#define READ_CFG_REG_CMD                     0x15
//#define WRITE_STATUS_CFG_REG_CMD             0x01
//
//#define READ_LOCK_REG_CMD                    0x2D
//#define WRITE_LOCK_REG_CMD                   0x2C
//
//#define READ_EXT_ADDR_REG_CMD                0xC8
//#define WRITE_EXT_ADDR_REG_CMD               0xC5
//
///* Program Operations */
//#define PAGE_PROG_CMD                        0x02
//#define QPI_PAGE_PROG_4_BYTE_ADDR_CMD        0x12
//
//#define QUAD_IN_FAST_PROG_CMD                0x38
//#define EXT_QUAD_IN_FAST_PROG_CMD            0x38
//#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD    0x3E
//
///* Erase Operations */
//#define SUBSECTOR_ERASE_CMD                  0x20
//#define SUBSECTOR_ERASE_4_BYTE_ADDR_CMD      0x21
//
//#define SECTOR_ERASE_CMD                     0xD8
//#define SECTOR_ERASE_4_BYTE_ADDR_CMD         0xDC
//
//#define BULK_ERASE_CMD                       0xC7
//
//#define BULK_ERASE_CMD 0xC7
//
//#define PROG_ERASE_RESUME_CMD 0x30
//#define PROG_ERASE_SUSPEND_CMD 0xB0
//
///* 4-byte Address Mode Operations */
//#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
//#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9
//
///* Quad Operations */
//#define ENTER_QUAD_CMD                       0x35
//#define EXIT_QUAD_CMD                        0xF5

/**
  * @brief  QSPI Registers
  */
/* Status Register */
//#define QSPI_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
//#define QSPI_SR_WREN                     ((uint8_t)0x02)    /*!< Write enable latch */
//#define QSPI_SR_BLOCKPR                  ((uint8_t)0x5C)    /*!< Block protected against program and erase operations */
//#define QSPI_SR_PRBOTTOM                 ((uint8_t)0x20)    /*!< Protected memory area defined by BLOCKPR starts from top or bottom */
//#define QSPI_SR_QUADEN                   ((uint8_t)0x40)    /*!< Quad IO mode enabled if =1 */
//#define QSPI_SR_SRWREN                   ((uint8_t)0x80)    /*!< Status register write enable/disable */
//
///* Configuration Register */
//#define QSPI_CR_ODS                      ((uint8_t)0x07)    /*!< Output driver strength */
//#define QSPI_CR_ODS_30                   ((uint8_t)0x07)    /*!< Output driver strength 30 ohms (default)*/
//#define QSPI_CR_ODS_15                   ((uint8_t)0x06)    /*!< Output driver strength 15 ohms */
//#define QSPI_CR_ODS_20                   ((uint8_t)0x05)    /*!< Output driver strength 20 ohms */
//#define QSPI_CR_ODS_45                   ((uint8_t)0x03)    /*!< Output driver strength 45 ohms */
//#define QSPI_CR_ODS_60                   ((uint8_t)0x02)    /*!< Output driver strength 60 ohms */
//#define QSPI_CR_ODS_90                   ((uint8_t)0x01)    /*!< Output driver strength 90 ohms */
//#define QSPI_CR_TB                       ((uint8_t)0x08)    /*!< Top/Bottom bit used to configure the block protect area */
//#define QSPI_CR_PBE                      ((uint8_t)0x10)    /*!< Preamble Bit Enable */
//#define QSPI_CR_4BYTE                    ((uint8_t)0x20)    /*!< 3-bytes or 4-bytes addressing */
//#define QSPI_CR_NB_DUMMY                 ((uint8_t)0xC0)    /*!< Number of dummy clock cycles */
//
//#define QSPI_MANUFACTURER_ID               ((uint8_t)0xC2)
//#define QSPI_DEVICE_ID_MEM_TYPE            ((uint8_t)0x20)
//#define QSPI_DEVICE_ID_MEM_CAPACITY        ((uint8_t)0x1A)
//#define QSPI_UNIQUE_ID_DATA_LENGTH         ((uint8_t)0x10)  /*JCC: not checked */
//
//void _Error_Handler(char *file, int line);
//-----------------------------------------------------------------------------

//void GyroX_aver(float new_val);
//void GyroY_aver(float new_val);
//void GyroZ_aver(float new_val);
//void Height_Diff_Aver(float new_val);

//extern cUSART<TERMINAL_RX_BUFFER_SIZE, TERMINAL_TX_BUFFER_SIZE> g_term;
//extern cUSART<TELEMETRY_RX_BUFFER_SIZE, TELEMETRY_TX_BUFFER_SIZE> g_telem;
//extern cUSART<IMU_RX_BUFFER_SIZE, IMU_TX_BUFFER_SIZE> g_imu;
//extern int need_to_send_cfg;
//extern bool main_start;
//extern uint16_t Color_Status;
//extern float current_GyroX_average,current_GyroY_average,current_GyroZ_average, HDC_current;
//extern float height_diff;
//extern float Start_Bar_Altitude;
//extern float Current_Vibrations;
//extern U16   Move_UpDown;  // 0 - no, 1 Up, 2 - Down
//extern bool  OnTheGround;
//extern float ec_5_supply_iir;
//extern float ec_12_supply_iir;
//extern float ec_main_supply_iir;
//extern float ec_cap_iir;
//extern float ec_emr_bat_iir;
//extern int Local_Delta_Time;

//extern bool Got_new_data_from_Sense;
//extern U16 RC_FIRE_signal_FAIL;
//extern U16 RC_ARM_signal_FAIL;
//extern U16 RC_ARM_pw;
//extern U16 RC_FIRE_pw;
//extern U16 RC_ARM_1_cnt;
//extern U16 RC_FIRE_1_cnt;
//extern U16 Delay_05sec;
//extern U16 Delay_2sec;
//extern U16 Delay_3sec;
//extern U16 Delay_4sec;

//extern U16 RC_tgl_mode_cnt;
//extern bool RC_tgl_mode_command;
//extern bool RC_gear_command;
//extern bool RC_ARM_command;
//extern bool RC_Battery_command;
//extern bool RC_FIRE_command;
//extern bool RC_OFF_command;
//extern bool RC_FIRE_1;
//extern bool RC_ARM_1;
//extern bool RC_FIRE_command_Executed;
//
//extern int Total;
//extern int Pulse_1_width;
//extern int Pulse_2_width;

//extern U16 RC_ARM_ON;
//extern U16 RC_FIRE_ON;
//extern U16 RC_ARM_ON_MAX;
//extern U16 RC_FIRE_ON_MAX;
//extern U16 PW_ARM_cnt;
//extern uint16_t Buzzer_mode;
//extern 	float free_kb, total_kb;
//extern FATFS SDFatFS;
//extern char start_buffer[1000];
//extern int msec100,msec20;
//extern  U32 systick_count_up;
//extern  U32 systick_log_counter;
//extern  U32 systick_counter;
//extern  U32 Previous_ee2_Saved;
//extern bool calibration_mode;
//extern bool NoVibrations;
//extern U32 NoVibrations_time;
//extern U16 Moving_downward;
//extern U16 Moving_upward;
//extern float altitude_out_prev;
//extern float start_height;
//extern int prev_vibr_sec;
//extern uint16_t init_done;
//extern float Pitch_src, Roll_src;
//extern uint8_t GPS_DL_IMU_unavailable;
//extern bool fall_detected;
//extern U32 fall_detection_duration;
//extern bool Show_Debug;
//extern uint16_t prev_state;
//extern int prev_SentCnt;
//extern int prev_GPS_sec;
//extern bool ec_wait_first_entry;
//extern U16 emergency_fire_state;
//extern U08 blink_state;
//extern bool auto_armed_or_disarmed;
//extern float Smart_VERSION_ID;
//extern float Sense_VERSION_ID;
//extern float Modem_VERSION_ID;
//extern int SentCnt_Start_Num;
//extern float Roll_Initialization_margin, Pitch_Initialization_margin;
//extern float Pyro_resistance_mesured;
//extern U16 Flash_Drive_Status;
//extern int GSM_connection;
//extern pU32 ee_ram_base1;
//extern pU32 ee_ram_ptr1;
//extern pU32 ee_flash_base1;
//extern pU32 ee_flash_ptr1;
//extern U32 ee_size1;
//extern U32 ee_count1;
//extern U32 PyroOnTime;
//extern float Non_Filtered_altitude_out;

//extern pU32 ee_ram_base2;
//extern pU32 ee_ram_ptr2;
//extern pU32 ee_flash_base2;
//extern pU32 ee_flash_ptr2;
//extern U32 ee_size2;
//extern U32 ee_count2;
//
//extern U32 Flight_count;
//extern U32 prev_log_tick;

//extern U16 GlobalLSI;
//extern U08 MinimumLSI;
//extern U32 TelemetryBuadRate;
//extern U32 systickLastSentTelemetry;
//extern bool print_error_enabled;
//extern bool NewMS56DataAvailable;
//extern bool LogWritingInProcess;
//extern U08 GlobalBSI;
//extern float temperature_out;
//extern float Raw_altitude_out, Raw_air_pressure_out;

//extern U32 LastGyroXYMeassurementSystick;
//extern U32 LastGyroZMeassurementSystick;
//extern U32 LastAccMeassurementSystick;
//extern U32 LastBaroMeassurementSystick;
//extern U32 LastPressureMeassurementSystick;
//extern U32 LastPitchRollMeassurementSystick;
//extern U32 LastYawMeassurementSystick;
//extern U32 LastBatteryMeassurementSystick;
//extern U32 LastFirePulseMeassurementSystick;
//extern U32 LastTemperatureMeassurementSystick;
//extern U32 LastZVelocityMeassurementSystick;
//extern U32 LastLogLineWrittenSystick;

//extern bool Calib;
//extern int CalibCount;
//extern float CalibSum;
//extern float Pitch_arr_Calib[10] ;
//extern float Roll_arr_Calib[10];
//extern float Pitch_STD_Calib;
//extern float Roll_STD_Calib;
//extern float PitchCalibAverage;
//extern float RollCalibAverage;
//
//extern float InitPressure;
//extern float InitAltitude;
//extern float MinimumABSAccInSlidingWindow;
//extern float MaxDurationOfMinimumABSAccInSlidingWindow;
//extern U08 VelocityThresholdCounter;
//extern int LogMultiplier;
//extern bool SessionUnlocked;


//eCI_RESULT ci_display_directory(void);
//void Send_Cfg_to_SenseAir();
//char* CT();
//void PRINT_Start_TRG_Mode();
//void PRINT_Start_ARM_Mode();
//void ee_print_parameters(bool print_to_scr);
//void systick_init();
//FRESULT log_init(bool append);
//void TGL_Start_Mode();
//void Open_CFG_file(char* file_name);
//void Set_blink_mode(U16 bm);
//U08 Get_blink_mode(void);
////eCI_RESULT ci_ee_save(void);
//bool ec_rc_calibration(void);
//bool ec_init_att(void);
//void Buzzer_Play(void);
//void Buzzer_Play2(void);
//void Buzzer_Set_Mode(uint16_t bm);
//void print_state(bool print_to_scr);
//void tel_term_check(void);
//bool wdg_fail(void);
//void log_file_rename(void);
//void ec_supply_print(bool print_to_scr);
//void ec_first_init_adc_value(void);
////eCI_RESULT ci_ee_erase(void); // Erase EEPROM parameters
//void SD_Fail(int N);
//void Write_Header(void);
//bool Allow_Arming(void);
//void telem_update_fire(U08 _state, U08 _arm , U16 _cond);
//void telem_update_version(U08 _state, U08 _arm , U32 _cond);
//extern int SD_res;
//extern U08 current_led_mode;
//extern DWORD fre_clust, fre_sect, tot_sect;
//extern FATFS *nfs;
//extern char aRxBufferCh5;
//extern UART_HandleTypeDef huart5;
//extern QSPI_HandleTypeDef hqspi;
//extern uint8_t eefire_servo_pwm, eemotor_servo_pwm;
//uint8_t QSPI_Init(void);
//uint8_t QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
//uint8_t QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
//uint8_t QSPI_Erase_Sector4K(uint32_t SectorAddress);
//uint8_t QSPI_WriteEnable(void);
//uint8_t QSPI_AutoPollingMemReady  (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
//uint8_t QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
//uint8_t QSPI_Read_Status_registers(QSPI_HandleTypeDef *hqspi,uint16_t* R1,uint16_t* R2,uint16_t* R3);
//uint8_t QSPI_Reset_Status_registers(QSPI_HandleTypeDef *hqspi,uint16_t* R1,uint16_t* R2,uint16_t* R3);
//uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength);
//bool ec_battery(void);
//void Go_Standby(bool quiet);
//void Show_battery(void);
//void Height_Diff_Reset();
//void Update_Dynamic_Array(float new_val, float32_t *pSrc, int Array_Length);                                                                          //Added By Nadav G. 2019-08-11
//float Delta_Height_In_Time_Interval(float32_t *DeltaHeightArray, float32_t *DeltaHeightTimeArray, int *DeltaHeightArrayIndex, int *DeltaTimeAverage); //Added By Nadav G. 2019-08-14
//void Value_In_Time_Interval(float32_t *DeltaHeightArray, float32_t *DeltaHeightTimeArray, int *DeltaHeightArrayIndex, int *DeltaTimeAverage);
//void ArrayAverage(float *ReturnedAverage, float32_t *pSrc, int Array_Length);
void WriteLogLine();
//float floatArrayMinValue(float32_t *pSrc, int Array_Length);
//void MinAccelerationDurationForFreefall(float32_t *pSrc, float32_t *timeSrc, int Array_Length, float32_t *Duration, float32_t *MinAccelration);

//-----------------------------------------------------------------------------

void error(void);

#endif
