/*
 * lcd.h
 *
 *  Created on: Jun 17, 2024
 *      Author: krsin
 */

#ifndef LCD_H
#define LCD_H

#include "stm32f4xx_hal.h"

void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_put_cur(int row, int col);

#endif
