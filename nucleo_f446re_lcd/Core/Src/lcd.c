/*
 * lcd.c
 *
 *  Created on: Jun 17, 2024
 *      Author: krsin
 */


#include "lcd.h"

#define LCD_RS_PIN 	GPIO_PIN_4
#define LCD_RS_PORT	GPIOA
#define LCD_E_PIN	GPIO_PIN_6
#define LCD_E_PORT	GPIOA
#define LCD_D4_PIN	GPIO_PIN_3
#define LCD_D4_PORT GPIOB
#define LCD_D5_PIN	GPIO_PIN_4
#define LCD_D5_PORT GPIOB
#define LCD_D6_PIN	GPIO_PIN_5
#define LCD_D6_PORT GPIOB
#define LCD_D7_PIN	GPIO_PIN_6
#define LCD_D7_PORT GPIOB




void lcd_send_cmd(char cmd)
{
    // Send higher nibble
//	char upperBits;
//	upperBits = ((cmd >> 4) & 0x0F);
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, ((cmd) & 0x10));
	HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, ((cmd) & 0x20));
	HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, ((cmd) & 0x40));
	HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, ((cmd) & 0x80));
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);


    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, ((cmd) & 0x01));
    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, ((cmd) & 0x02));
    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, ((cmd) & 0x04));
    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, ((cmd) & 0x08));
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);


}

void lcd_send_data(char data)
{

//	char upperBits;
//	upperBits = ((data >> 4) & 0x0F);
    // Send higher nibble
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, ((data) & 0x10));
	HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, ((data) & 0x20));
	HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, ((data) & 0x40));
	HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, ((data) & 0x80));
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);


    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, ((data) & 0x01));
    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, ((data) & 0x02));
    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, ((data) & 0x04));
    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, ((data) & 0x08));
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
}

void lcd_init(void)
{
	HAL_Delay(50);
    lcd_send_cmd(0x02); // Initialize LCD in 4-bit mode
    HAL_Delay(5); // Wait for more than 4.1 ms

    lcd_send_cmd(0x28); // Function set: 2 lines, 5x7 matrix
    HAL_Delay(5); // Wait for more than 4.1 ms

    lcd_send_cmd(0x0C); // Display control: Display on, cursor off, no blink
    HAL_Delay(5); // Wait for more than 4.1 ms

    lcd_send_cmd(0x06); // Entry mode set: Increment cursor
    HAL_Delay(5); // Wait for more than 4.1 ms

    lcd_send_cmd(0x01); // Clear display
    HAL_Delay(2); // Wait for more than 1.64 ms
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);  // Clear display
    HAL_Delay(2);
}

void lcd_put_cur(int row, int col)
{
    switch (row) {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd(col);
}






