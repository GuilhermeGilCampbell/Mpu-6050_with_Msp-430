// LCD lib

#ifndef LCD_H_
#define LCD_H_

#define LCD_GUARD_TIME  5
#define LCD_ENABLE_TIME 2
#define LCD_RESET_TIME  2

#define INSTRUCTION 1
#define CHARACTER   0

void lcdInit();

void lcdWriteStr(unsigned char str[]);
void lcdWriteByte(unsigned char instr, unsigned char word);
void lcdWriteNibble(unsigned char regSelect, unsigned char halfWord);

void lcdClear();
void lcdHome();
void lcdGoTo(unsigned char line, unsigned char column);

#endif /* LCD_H_ */
