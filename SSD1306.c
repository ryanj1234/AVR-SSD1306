#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "i2c.h"

#define DISPLAY_WIDTH        128
#define DISPLAY_HEIGHT        64

#define SSD1306        0x78
#define I2C_WRITE       0x00
#define I2C_READ        0x01

#define CTRL_FRAME      0x00
#define CONT_BIT        0b10000000
#define CMD_BIT         0b00000000
#define GDD_BIT         0b01000000

/** Commands */
// 1 fundamental command table
#define SET_CONTRAST_CMD    0x81
#define RESUME_DISP_CMD     0xA4
#define DISP_ON_RESUME_CMD  0xA4
#define SET_NORM_DISP       0xA6
#define SET_DISP_OFF_CMD    0xAE
#define SET_DISP_ON_CMD     0xAF

// 2 scrolling command table
#define DEACT_SCROLL_CMD    0x2E

// 3 addressing setting command table
#define SET_MEM_ADDR_MODE_CMD   0x20

// 4 hardware configuration
#define SET_DISP_START_LINE_CMD 0x40 // OR with value 0-63
#define SET_SEG_REMAP               0xA0
#define SET_MUX_RATIO   0xA8
#define SET_COM_SCAN_DEC    0xC8
#define SET_DISPLAY_OFFSET_CMD  0xD3
#define SET_COM_PINS    0xDA

// 5. Timing & Driving Scheme Setting Command Table
#define SET_CLK_DIV         0xD5
#define SET_PRE_CHARGE_PER  0xD9
#define SET_VCOM_DESELECT   0xDB

#define SET_CHARGE_PUMP     0x8D

void ssd1306_write_byte(uint8_t cmd)
{
    I2C_start();
    I2C_write(SSD1306);
    I2C_write(CTRL_FRAME | CMD_BIT);
    I2C_write(cmd);
    I2C_stop();
}

void ssd1306_write_param(uint8_t cmd, uint8_t dat)
{
    I2C_start();
    I2C_write(SSD1306);
    I2C_write(CTRL_FRAME | CMD_BIT);
    I2C_write(cmd);
    I2C_write(dat);
    I2C_stop();
}

void send_dat(uint8_t* dat, uint16_t size)
{
    I2C_start();
    I2C_write(SSD1306);
    I2C_write(CTRL_FRAME | GDD_BIT);

    for(uint16_t i = 0; i < size; i++)
    {
        I2C_write(dat[i]);
    }

    I2C_stop();
}

/**
 * Turns the display off
 */
__inline__ void ssd1306_disp_off(void)
{
    ssd1306_write_byte(SET_DISP_OFF_CMD);
}

/**
 * Turns the display on
 */
__inline__ void ssd1306_disp_on(void)
{
    ssd1306_write_byte(SET_DISP_ON_CMD);
}

/**
 * Set the display clock divide ratio and oscillator frequency
 * 
 * bits 3:0 are the divide ratio
 * 
 * bits 7:4 are the oscillator frequency. Higher values result in a higher 
 * frequency
 */
__inline__ void ssd1306_set_clk(uint8_t dat)
{
    ssd1306_write_param(SET_CLK_DIV, dat);
}

/**
 * Set MUX ratio to N+1 MUX
 * N=A[5:0] : from 16MUX to 64MUX, RESET=
 * 111111b (i.e. 63d, 64MUX)
 * A[5:0] from 0 to 14 are invalid entry.
 */
__inline__ void ssd1306_set_mux_ratio(uint8_t dat)
{
    ssd1306_write_param(SET_MUX_RATIO, dat);
}

/**
 * Set vertical shift by COM from 0d~63d
 * The value is reset to 00h after RESET.
 */
__inline__ void ssd1306_set_display_offset(uint8_t dat)
{
    ssd1306_write_param(SET_DISPLAY_OFFSET_CMD, dat);
}

/**
 * Set display RAM display start line register from
 * 0-63 using X 5 X 3 X 2 X 1 X 0 .
 * Display start line register is reset to 000000b
 * during RESET.
 */
__inline__ void ssd1306_set_disp_startline(uint8_t dat)
{
    ssd1306_write_byte(SET_DISP_START_LINE_CMD);
}

/**
 * Enable the charge pump... bump?
 * 
 * The Charge Pump must be
 * enabled by the following command:
 * 8Dh ; Charge Pump Setting
 * 14h ; Enable Charge Pump
 * AFh; Display ON
 */
__inline__ void ssd1306_charge_pump_enable(void)
{
    ssd1306_write_param(SET_CHARGE_PUMP, 0x14);
}

#define HORZ_ADDR_MODE  0b00
#define VERT_ADDR_MODE  0b01
#define PAGE_ADDR_MODE  0b10 // (RESET)
/**
 * A[1:0] = 00b, Horizontal Addressing Mode
 * A[1:0] = 01b, Vertical Addressing Mode
 * A[1:0] = 10b, Page Addressing Mode (RESET)
 * A[1:0] = 11b, Invalid
 * 
 * HORZ_ADDR_MODE
 * VERT_ADDR_MODE
 * PAGE_ADDR_MODE // (RESET)
 */
__inline__ void ssd1306_set_mem_addr_mode(uint8_t dat)
{
    ssd1306_write_param(SET_MEM_ADDR_MODE_CMD, dat);
}

#define SEG_NO_REMAP    0x00
#define SEG_INVERT      0x01
/**
 * A0h, X[0]=0b: column address 0 is mapped to
 * SEG0 (RESET)
 * A1h, X[0]=1b: column address 127 is mapped to
 * SEG0
 * 
 * SEG_NO_REMAP
 * SEG_INVERT
 */
__inline__ void ssd1306_set_seg_remap(uint8_t dat)
{
    ssd1306_write_byte(SET_SEG_REMAP| dat);
}

#define COM_SCAN_NORM   0xC0
#define COM_SCAN_INV    0xC8
/**
 * C0h, X[3]=0b: normal mode (RESET) Scan from
 * COM0 to COM[N –1]
 * C8h, X[3]=1b: remapped mode. Scan from
 * COM[N-1] to COM0
 * Where N is the Multiplex ratio.
 * 
 * COM_SCAN_NORM
 * COM_SCAN_REMAP
 */
__inline__ void ssd1306_set_com_scan_dir(uint8_t dat)
{
    ssd1306_write_byte(dat);
}

enum com_pin_config { 
    SEQ_COM_PIN  = 0b00000010, 
    ALT_COM_PIN  = 0b00010010, 
    DIS_LR_REMAP = 0b00000010, 
    EN_LR_REMAP  = 0b00100010
};
/**
 * A[4]=0b, Sequential COM pin configuration
 * A[4]=1b(RESET), Alternative COM pin
 * configuration
 * A[5]=0b(RESET), Disable COM Left/Right
 * remap
 * A[5]=1b, Enable COM Left/Right remap
 */
__inline__ void ssd1306_set_com_pins(enum com_pin_config dat)
{
    ssd1306_write_param(SET_COM_PINS, dat);
}

/**
 * Double byte command to select 1 out of 256
 * contrast steps. Contrast increases as the value
 * increases.
 * (RESET = 7Fh )
 */
__inline__ void ssd1306_set_contrast(uint8_t dat)
{
    ssd1306_write_param(SET_CONTRAST_CMD, dat);
}

/**
 * A[3:0] : Phase 1 period of up to 15 DCLK
 * clocks 0 is invalid entry
 * (RESET=2h)
 * A[7:4] : Phase 2 period of up to 15 DCLK
 * clocks 0 is invalid entry
 * (RESET=2h)
 */
__inline__ void ssd1306_set_pre_charge_period(uint8_t dat)
{
    ssd1306_write_param(SET_PRE_CHARGE_PER, dat);
}

#define VCOM_DES_65     0x00
#define VCOM_DES_77     0x20
#define VCOM_DES_83     0x30
/**
 * VCOM_DES_65 ~ 0.65 x Vcc
 * VCOM_DES_77 ~ 0.77 x Vcc
 * VCOM_DES_83 ~ 0.83 x Vcc
 */
__inline__ void ssd1306_set_vcom_detect(uint8_t dat)
{
    ssd1306_write_param(SET_VCOM_DESELECT, dat);
}

/**
 * A4h, X 0 =0b: Resume to RAM content display
 * (RESET)
 */
__inline__ void ssd1306_display_resume(void)
{
    ssd1306_write_byte(RESUME_DISP_CMD);
}

#define RAM_OFF_DISP    0x00
#define RAM_ON_DISP     0x01
/**
 * A6h, X[0]=0b: Normal display (RESET)
 * 0 in RAM: OFF in display panel
 * 1 in RAM: ON in display panel
 */
__inline__ void ssd1306_set_norm_disp(uint8_t ram_opt)
{
    ssd1306_write_byte(SET_NORM_DISP | ram_opt);
}

/**
 * Deactivate scroll
 */
__inline__ void ssd1306_deactivate_scroll(void)
{
    ssd1306_write_byte(DEACT_SCROLL_CMD);
}

int main(void)
{
    ssd1306_disp_off();
    ssd1306_set_clk(0x80);
    ssd1306_set_mux_ratio(DISPLAY_HEIGHT - 1);

    ssd1306_set_display_offset(0x00);
    ssd1306_set_disp_startline(0x00);
    ssd1306_charge_pump_enable();
    ssd1306_set_mem_addr_mode(HORZ_ADDR_MODE);
    ssd1306_set_seg_remap(SEG_INVERT);
    ssd1306_write_byte(COM_SCAN_INV);

    ssd1306_set_com_pins(SEQ_COM_PIN);
    ssd1306_set_pre_charge_period(0xF1);
    ssd1306_set_vcom_detect(VCOM_DES_77); // different from adafruit
    ssd1306_display_resume();
    ssd1306_set_norm_disp(RAM_OFF_DISP);

    ssd1306_disp_on();

    while(1) {
        ssd1306_set_contrast(0x00);
        _delay_ms(500);
        ssd1306_set_contrast(128);
        _delay_ms(500);
        ssd1306_set_contrast(255);
        _delay_ms(500);
    }
    // {
    //     for(uint8_t i = 0; i < 255; i++)
    //     {
    //         ssd1306_set_contrast(i);
    //         _delay_ms(10);
    //     }
    // }
}
