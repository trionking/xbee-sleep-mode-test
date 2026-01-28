/*
 * user_def.c
 *
 *  Created on: Jan 27, 2026
 *      Author: trion
 */


#include "main.h"
#include "user_def.h"
#include "user_com.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;

// System clock configuration (defined in main.c)
extern void SystemClock_Config(void);

#ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
 HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1);
 return ch;
}

int _write(int file, char *data, int len)
{
    int bytes_written;

    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }

    for (bytes_written = 0; bytes_written < len; bytes_written++)
    {
        __io_putchar(*data);
        data++;
    }

    return bytes_written;
}

// ============================================================================
// Buzzer Control
// ============================================================================
float buzzer_vol_rate[6] = {0, 0.05, 0.07, 0.1, 0.2, 0.5};
uint32_t bz_ply_time = 0;
uint8_t f_bz_stat = 0;
volatile struct BZ_SEQ_ST bz_play_seq_st = {0};

uint16_t bz_seq_data[][2] = {
    {BZ_TYPE1, 20}, {BZ_TYPE2, 20}, {BZ_TYPE3, 20}, {0, 0},                      // Melody1 (BZ_SEQ_USER0)
    {BZ_TYPE3, 20}, {BZ_TYPE2, 20}, {BZ_TYPE1, 20}, {0, 0},                      // Melody2 (BZ_SEQ_USER1)
    {BZ_TYPE3, 5},  {0, 0},                                                      // super short beep (BZ_SEQ_SSHRT)
    {BZ_TYPE1, 20}, {0, 0},                                                      // short beep (BZ_SEQ_SHRT)
    {BZ_TYPE1, 100}, {0, 0},                                                     // long beep (BZ_SEQ_LONG)
    {BZ_TYPE1, 10}, {1, 10}, {BZ_TYPE1, 10}, {0, 0},                             // 2xbeep (BZ_SEQ_2xBEEP)
    {BZ_TYPE1, 10}, {1, 10}, {BZ_TYPE1, 10}, {1, 10}, {BZ_TYPE1, 10}, {0, 0},    // 3xbeep (BZ_SEQ_3xBEEP)
    {BZ_TYPE5, 1},  {0, 0},                                                      // encoder cnt (BZ_SEQ_ENC)
};

void bz_seq_play(uint16_t (*tm_bz_dat)[2])
{
    uint8_t tm_len;

    bz_play_seq_st.bz_dat = tm_bz_dat;
    for (int i = 0; i < 10; i++)
    {
        if (bz_play_seq_st.bz_dat[i][0] == 0)
        {
            tm_len = i;
            break;
        }
    }

    bz_play_seq_st.bz_len = tm_len;
    bz_play_seq_st.f_bz_seq_play = 1;
    bz_play_seq_st.bz_cnt = 0;
}

void bz_play(uint16_t frq_hz, uint32_t bz_p_time)
{
#ifdef BUZ_ENABLE
    if (f_bz_stat == 0)
    {
        bz_ply_time = bz_p_time;
        __HAL_TIM_SET_AUTORELOAD(&htim16, frq_hz);
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,
            ((uint16_t)((float)frq_hz * buzzer_vol_rate[bz_play_seq_st.bz_vol_sel_cnt])));
        HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
        f_bz_stat = 1;
    }
#endif
}

uint8_t bz_is_run(void)
{
    return f_bz_stat;
}

void bz_stop(void)
{
    f_bz_stat = 0;
    bz_ply_time = 0;
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
}

void bz_ctrl_init(void)
{
    bz_play_seq_st.bz_vol_sel_cnt = 5;  // volume (0,[1],2,3,4,5)
}

// Timer callback (called every 1ms by TIM7)
static uint8_t cntT_10ms = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        cntT_10ms++;
        if (cntT_10ms >= 10)
        {
            cntT_10ms = 0;
            bz_proc();
        }
    }
}

// Buzzer process - called every 10ms
void bz_proc(void)
{
    // buzzer control process
    if (f_bz_stat)
    {
        if (bz_ply_time > 0)
        {
            bz_ply_time--;
            if (!bz_ply_time)
            {
                bz_stop();
            }
        }
    }

    if (bz_play_seq_st.f_bz_seq_play)
    {
        if (!f_bz_stat)
        {
            if (bz_play_seq_st.bz_cnt < bz_play_seq_st.bz_len)
            {
                bz_play(bz_play_seq_st.bz_dat[bz_play_seq_st.bz_cnt][0],
                        bz_play_seq_st.bz_dat[bz_play_seq_st.bz_cnt][1]);
                bz_play_seq_st.bz_cnt++;
            }
            else
            {
                bz_play_seq_st.f_bz_seq_play = 0;
            }
        }
    }
}

uint8_t read_switch(void)
{
    uint8_t rtn_val;
    rtn_val = HAL_GPIO_ReadPin(EXTI_SW1_GPIO_Port, EXTI_SW1_Pin);
    rtn_val <<= 1;
    rtn_val |= HAL_GPIO_ReadPin(EXTI_SW2_GPIO_Port, EXTI_SW2_Pin);
    rtn_val <<= 1;
    rtn_val |= HAL_GPIO_ReadPin(EXTI_SW3_GPIO_Port, EXTI_SW3_Pin);

    rtn_val ^= 0xFF;
    rtn_val &= 0x07;

    return rtn_val;
}

// ============================================================================
// XBee (Zigbee) Communication
// ============================================================================
static uint8_t device_state = DEV_STATE_UNKNOWN;  // Remote device state
static uint8_t xbee_rx_buf[UART2_RX_BUF_SIZE];
static volatile uint8_t f_wakeup = 0;

void xbee_init(void)
{
    xbee_sleep_ctl(1);  // Wake up XBee module
    device_state = DEV_STATE_UNKNOWN;
}

void xbee_sleep_ctl(uint8_t ctrl)
{
    if (ctrl)
    {
        // Wake up
        HAL_GPIO_WritePin(OT_ZIG_SLEEP_GPIO_Port, OT_ZIG_SLEEP_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Sleep
        HAL_GPIO_WritePin(OT_ZIG_SLEEP_GPIO_Port, OT_ZIG_SLEEP_Pin, GPIO_PIN_SET);
    }
}

void xbee_tx(uint8_t *tx_dat, uint16_t size)
{
    UART2_SendData(tx_dat, size);
    // Process TX queue
    while (Len_queue(&tx_UART2_queue) > 0)
    {
        UART2_TX_proc();
        HAL_Delay(1);
    }
    HAL_Delay(10);  // Wait for DMA TX complete
}

void xbee_send_cmd(const char *cmd)
{
    printf_UART2("%s\r\n", cmd);
    // Process TX queue
    while (Len_queue(&tx_UART2_queue) > 0)
    {
        UART2_TX_proc();
        HAL_Delay(1);
    }
    HAL_Delay(10);  // Wait for DMA TX complete
    printf("[XBee TX] %s\r\n", cmd);
}

void xbee_send_on(void)
{
    xbee_send_cmd("on");
}

void xbee_send_off(void)
{
    xbee_send_cmd("off");
}

void xbee_send_open(void)
{
    xbee_send_cmd("open");
}

void xbee_send_close(void)
{
    xbee_send_cmd("close");
}

void xbee_send_status(void)
{
    xbee_send_cmd("status");
}

// Request status and wait for response
// Returns: DEV_STATE_ON, DEV_STATE_OFF, or DEV_STATE_UNKNOWN
uint8_t xbee_request_status(void)
{
    uint32_t timeout_tick;
    uint16_t rx_len = 0;
    uint8_t rx_byte;
    HAL_StatusTypeDef status;

    // Clear buffer
    memset(xbee_rx_buf, 0, sizeof(xbee_rx_buf));

    // Send status request
    xbee_send_status();

    // Wait for response using polling (timeout 500ms)
    timeout_tick = HAL_GetTick();
    while ((HAL_GetTick() - timeout_tick) < 500)
    {
        status = HAL_UART_Receive(&huart2, &rx_byte, 1, 10);
        if (status == HAL_OK)
        {
            printf("[RX] 0x%02X '%c'\r\n", rx_byte, rx_byte);  // Debug each byte
            if (rx_len < UART2_RX_BUF_SIZE - 1)
            {
                xbee_rx_buf[rx_len++] = rx_byte;

                // Check for end of line
                if (rx_byte == '\n' || rx_byte == '\r')
                {
                    xbee_rx_buf[rx_len] = '\0';
                    break;
                }
            }
        }
    }

    // Parse response
    if (rx_len > 0)
    {
        printf("[XBee RX] %s\r\n", xbee_rx_buf);

        if (strstr((char *)xbee_rx_buf, "on") != NULL)
        {
            device_state = DEV_STATE_ON;
            return DEV_STATE_ON;
        }
        else if (strstr((char *)xbee_rx_buf, "off") != NULL)
        {
            device_state = DEV_STATE_OFF;
            return DEV_STATE_OFF;
        }
    }

    printf("[XBee] No response or unknown\r\n");
    return DEV_STATE_UNKNOWN;
}

// ============================================================================
// Sleep Mode Control
// ============================================================================

// Configure GPIO for wake-up (EXTI interrupt)
void setup_wakeup_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure switch pins as EXTI (falling edge)
    GPIO_InitStruct.Pin = EXTI_SW1_Pin | EXTI_SW2_Pin | EXTI_SW3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Enable EXTI interrupts
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Restore GPIO to normal input mode
void restore_gpio_normal(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Disable EXTI interrupt
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    // Restore to normal input
    GPIO_InitStruct.Pin = EXTI_SW1_Pin | EXTI_SW2_Pin | EXTI_SW3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// GPIO EXTI callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == EXTI_SW1_Pin || GPIO_Pin == EXTI_SW2_Pin || GPIO_Pin == EXTI_SW3_Pin)
    {
        f_wakeup = 1;
    }
}

// Enter low power STOP mode
// Returns: switch value that caused wake-up (0 if none)
uint8_t enter_sleep_mode(void)
{
    uint8_t wake_sw_val = 0;

    printf("Entering STOP Mode...\r\n");
    HAL_Delay(100);  // Wait for UART to finish

    // Put XBee to sleep
    xbee_sleep_ctl(0);

    // Stop peripherals
    HAL_TIM_Base_Stop_IT(&htim7);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);  // Stop buzzer
    HAL_SuspendTick();

    // Setup wake-up GPIO (EXTI)
    setup_wakeup_gpio();
    f_wakeup = 0;

    // Disable unused GPIO clocks and set pins to analog to reduce power
    // (Optional: can be more aggressive here)

    // Enter STOP mode (Low Power Regulator, WFI)
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    // --- Wake up here ---
    // Note: After STOP mode, system clock is HSI, need to reconfigure

    // Restore system clock
    SystemClock_Config();

    // Resume tick
    HAL_ResumeTick();

    // Restore GPIO
    restore_gpio_normal();

    // Read switch value immediately while button is still held
    wake_sw_val = read_switch();

    // Restart timer
    HAL_TIM_Base_Start_IT(&htim7);

    // Wake up XBee
    xbee_sleep_ctl(1);
    HAL_Delay(500);  // Wait for XBee to wake up and sync with Coordinator

    printf("Woke up from STOP! SW=%d\r\n", wake_sw_val);

    return wake_sw_val;
}

// ============================================================================
// Switch Processing with Sleep
// ============================================================================
void process_switch_action(uint8_t sw_val)
{
    static uint8_t toggle_state = 0;  // 0=OFF, 1=ON

    bz_seq_play(&bz_seq_data[BZ_SEQ_SSHRT]);

    switch (sw_val)
    {
    case SW_ON_OFF:  // Toggle on/off
        // Toggle without status request
        if (toggle_state == 0)
        {
            xbee_send_on();
            bz_seq_play(&bz_seq_data[BZ_SEQ_USER1]);  // Melody2
            toggle_state = 1;
        }
        else
        {
            xbee_send_off();
            bz_seq_play(&bz_seq_data[BZ_SEQ_USER0]);  // Melody1
            toggle_state = 0;
        }
        break;

    case SW_OPEN:  // Open command
        xbee_send_open();
        break;

    case SW_CLOSE:  // Close command
        xbee_send_close();
        break;

    case SW_STATUS:  // Status request (SW2 + SW3) - disabled for now
#if 0
        {
            uint8_t current_state = xbee_request_status();
            if (current_state == DEV_STATE_ON)
            {
                bz_seq_play(&bz_seq_data[BZ_SEQ_2xBEEP]);  // 2 beeps = ON
            }
            else if (current_state == DEV_STATE_OFF)
            {
                bz_seq_play(&bz_seq_data[BZ_SEQ_SHRT]);   // 1 beep = OFF
            }
            else
            {
                bz_seq_play(&bz_seq_data[BZ_SEQ_3xBEEP]); // 3 beeps = Unknown
            }
        }
#endif
        break;

    default:
        break;
    }

    // Wait for buzzer to finish
    HAL_Delay(500);
    while (bz_is_run())
    {
        HAL_Delay(10);
    }
}

// Wait for switch release
void wait_switch_release(void)
{
    while (read_switch() != 0)
    {
        HAL_Delay(10);
    }
    HAL_Delay(50);  // Debounce
}

// ============================================================================
// Main Process
// ============================================================================
void init_proc(void)
{
    HAL_TIM_Base_Start_IT(&htim7);  // Start TIM7 interrupt (1ms)
    init_UART_COM();                // Initialize UART2 DMA communication
    bz_ctrl_init();
    xbee_init();
    bz_seq_play(&bz_seq_data[BZ_SEQ_SSHRT]);  // startup beep
    printf("System Initialized\r\n");
}

void run_proc(void)
{
    uint8_t sw_val;

    init_proc();

#if 1  // Sleep mode enabled (0=Polling for test, 1=Sleep)
    // Main loop: Sleep -> Wake -> Process -> Sleep
    while (1)
    {
        // Enter sleep and wait for button press
        sw_val = enter_sleep_mode();

        // Process the wake-up button action immediately
        if (sw_val != 0)
        {
            process_switch_action(sw_val);
            wait_switch_release();
        }
    }
#else
    // Polling mode for testing (no sleep)
    uint8_t sw_old = 0;
    uint16_t sw_cnt = 0;
    while (1)
    {
        sw_val = read_switch();

        // Debounce
        if (sw_val != 0 && sw_val == sw_old)
        {
            sw_cnt++;
            if (sw_cnt == 10)  // 100ms debounce
            {
                process_switch_action(sw_val);
                wait_switch_release();
                sw_cnt = 0;
            }
        }
        else
        {
            sw_cnt = 0;
        }
        sw_old = sw_val;

        HAL_Delay(10);
    }
#endif
}
