/*
 * user_def.h
 *
 *  Created on: Jan 27, 2026
 *      Author: trion
 */

#ifndef INC_USER_DEF_H_
#define INC_USER_DEF_H_

#include "main.h"

// Buzzer Enable
#define BUZ_ENABLE

// buzzer frequency (10Mhz base)
#define BZ_TYPE1    		3700		// 10Mhz/3700 = 2.7khz
#define BZ_TYPE2    		5200		// 10Mhz/5200 = 1.9khz
#define BZ_TYPE3    		7000		// 10Mhz/7000 = 1.4khz
#define BZ_TYPE4    		15000		// 10Mhz/15000 = 666 hz
#define BZ_TYPE5    		35000		// 10Mhz/35000 = 285 hz

// buzzer sequence index
#define BZ_SEQ_USER0		0
#define BZ_SEQ_USER1		4
#define BZ_SEQ_SSHRT		8
#define BZ_SEQ_SHRT			10
#define BZ_SEQ_LONG			12
#define BZ_SEQ_2xBEEP		14
#define BZ_SEQ_3xBEEP		18
#define BZ_SEQ_ENC			24

struct BZ_SEQ_ST {
	uint8_t bz_cnt;
	uint16_t (*bz_dat)[2];
	uint8_t bz_len;
	uint8_t f_bz_seq_play;
	uint32_t bz_tick;
	float bz_vol_rate;
	uint8_t bz_vol_sel_cnt;
};

// Buzzer functions
void bz_seq_play(uint16_t (*tm_bz_dat)[2]);
void bz_play(uint16_t frq_hz, uint32_t bz_p_time);
uint8_t bz_is_run(void);
void bz_stop(void);
void bz_ctrl_init(void);
void bz_proc(void);

uint8_t read_switch(void);

// XBee (Zigbee) communication
#define SW_ON_OFF   4   // SW1
#define SW_STATUS   3   // SW2 + SW3
#define SW_OPEN     2   // SW2
#define SW_CLOSE    1   // SW3

// Device state
#define DEV_STATE_UNKNOWN   0
#define DEV_STATE_OFF       1
#define DEV_STATE_ON        2

// UART RX buffer size
#define UART2_RX_BUF_SIZE   64

void xbee_init(void);
void xbee_sleep_ctl(uint8_t ctrl);
void xbee_tx(uint8_t *tx_dat, uint16_t size);
void xbee_send_cmd(const char *cmd);
void xbee_send_on(void);
void xbee_send_off(void);
void xbee_send_open(void);
void xbee_send_close(void);
void xbee_send_status(void);
uint8_t xbee_request_status(void);
void xbee_rx_process(void);
void process_switch(void);

// Sleep mode
uint8_t enter_sleep_mode(void);  // Returns switch value that caused wake-up
void setup_wakeup_gpio(void);

void init_proc(void);
void run_proc(void);

#endif /* INC_USER_DEF_H_ */
