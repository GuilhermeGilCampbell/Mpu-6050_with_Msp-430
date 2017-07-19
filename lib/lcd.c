#include <msp430.h>
#include "lcd.h"
#include "ports.h"
#include "timer.h"

void lcdInit()
{
    lcdWriteNibble(INSTRUCTION, 0x3);       // Set 8-bits mode
                                            // (required if LCD starts in 4-bit mode)
    lcdWriteNibble(INSTRUCTION, 0x3);       // Set 8-bits mode
                                            // Why twice? This considers the case where
                                            // 4-bit mode was set and first word is LSB
    lcdWriteNibble(INSTRUCTION, 0x3);       // Set 8-bits mode (can't explain why)

    lcdWriteNibble(INSTRUCTION, 0x2);       // Set 4-bits mode
    lcdWriteByte(INSTRUCTION, 0x2C);        // Setup : N : 2-line cursor
                                            //         F : big chars 5x10
    lcdWriteByte(INSTRUCTION, 0x0E);        // Cursor on, static
    lcdClear();
}

void lcdWriteStr(unsigned char str[])
{
    unsigned char c;
    unsigned char i = 0;

    while (c = str[i++])
        lcdWriteByte(CHARACTER,c);
}

void lcdWriteByte(unsigned char instr, unsigned char word)
{
    // Send the first 4 bits (MSBs)
    lcdWriteNibble(instr, word >> 4);

    // Send the last 4 bits  (LSBs)
    lcdWriteNibble(instr, word & 0x0F);

}

void lcdWriteNibble(unsigned char instr, unsigned char halfWord) {

    EN_OUT &= ~EN_BIT;                      // Clear Enable

    if (instr)                              // Selects the register
        RS_OUT &= ~RS_BIT;                  // RS = 0 : Instruction
    else                                    // RS = 1 : DDRAM/CGRAM
        RS_OUT |=  RS_BIT;                  //

    portsWriteLCDBUS(halfWord);             // Output data to bus
    waitFor(LCD_GUARD_TIME);                // and wait a bit

    EN_OUT |=  EN_BIT;                      // This will generate a
    waitFor(LCD_ENABLE_TIME);               // pulse in enable output
    EN_OUT &= ~EN_BIT;                      //
}

void lcdClear() {
    // LCD Clear
    lcdWriteByte(INSTRUCTION, 0x01);
}

void lcdHome() {
    // Home
    lcdWriteByte(INSTRUCTION,0x02);
}

void lcdGoTo(unsigned char line, unsigned char column) {
    // Line feed
    lcdWriteByte(INSTRUCTION,0x80+(line<<6)+column);
}
