#include <avr/io.h>
#include <util/delay.h>
#include "SSD1306.h"
#include "i2c.h"
#include "font.h"
#include <string.h> // memset

#define PAGE_SIZE       8
#define NUM_PAGES       (DISPLAY_HEIGHT/PAGE_SIZE)

#define SSD1306         0x78
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
#define SET_DISP_START_LINE_CMD 0x40
#define SET_SEG_REMAP           0xA0
#define SET_MUX_RATIO           0xA8
#define SET_COM_SCAN_DEC        0xC8
#define SET_DISPLAY_OFFSET_CMD  0xD3
#define SET_COM_PINS            0xDA

// 5. Timing & Driving Scheme Setting Command Table
#define SET_CLK_DIV         0xD5
#define SET_PRE_CHARGE_PER  0xD9
#define SET_VCOM_DESELECT   0xDB
#define NOOP_CMD            0xE3

#define SET_CHARGE_PUMP     0x8D

static uint8_t disp_buf[NUM_PAGES][DISPLAY_WIDTH];

static __inline__ void _setup_cmd_write(void)
{
    i2c_start();
    i2c_write(SSD1306);
    i2c_write(CTRL_FRAME | CMD_BIT);
}

static __inline__ void _stop_write(void)
{
    i2c_stop();
}

static __inline__ void _write_byte(uint8_t b)
{
    i2c_write(b);
}

static __inline__ void _setup_dat_write(void)
{
    i2c_start();
    i2c_write(SSD1306);
    i2c_write(CTRL_FRAME | GDD_BIT);
}

void ssd1306_write_byte(uint8_t cmd)
{
    _setup_cmd_write();
    _write_byte(cmd);
    _stop_write();
}

void ssd1306_write_param(uint8_t cmd, uint8_t dat)
{
    _setup_cmd_write();
    _write_byte(cmd);
    _write_byte(dat);
    _stop_write();
}

void ssd1306_write_dat(uint8_t* dat, uint16_t size)
{
    _setup_dat_write();

    for(uint16_t i = 0; i < size; i++)
    {
        _write_byte(dat[i]);
    }

    _stop_write();
}

void ssd1306_command_seq(uint8_t* dat, uint8_t size)
{
    _setup_cmd_write();

    for(uint8_t i = 0; i < size; i++)
    {
        _write_byte(dat[i]);
    }
    
    _stop_write();
}

/**
 * Turns the display off
 */
static __inline__ void ssd1306_disp_off(void)
{
    ssd1306_write_byte(SET_DISP_OFF_CMD);
}

/**
 * Turns the display on
 */
static __inline__ void ssd1306_disp_on(void)
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
static __inline__ void ssd1306_set_clk(uint8_t dat)
{
    ssd1306_write_param(SET_CLK_DIV, dat);
}

/**
 * Set MUX ratio to N+1 MUX
 * N=A[5:0] : from 16MUX to 64MUX, RESET=
 * 111111b (i.e. 63d, 64MUX)
 * A[5:0] from 0 to 14 are invalid entry.
 */
static __inline__ void ssd1306_set_mux_ratio(uint8_t dat)
{
    ssd1306_write_param(SET_MUX_RATIO, dat);
}

/**
 * Set vertical shift by COM from 0d~63d
 * The value is reset to 00h after RESET.
 */
static __inline__ void ssd1306_set_display_offset(uint8_t dat)
{
    ssd1306_write_param(SET_DISPLAY_OFFSET_CMD, dat);
}

/**
 * Set display RAM display start line register from
 * 0-63 using X 5 X 3 X 2 X 1 X 0 .
 * Display start line register is reset to 000000b
 * during RESET.
 */
static __inline__ void ssd1306_set_disp_startline(uint8_t dat)
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
static __inline__ void ssd1306_charge_pump_enable(void)
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
static __inline__ void ssd1306_set_mem_addr_mode(uint8_t dat)
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
static __inline__ void ssd1306_set_seg_remap(uint8_t dat)
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
static __inline__ void ssd1306_set_com_scan_dir(uint8_t dat)
{
    ssd1306_write_byte(dat);
}

enum com_pin_config 
{ 
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
static __inline__ void ssd1306_set_com_pins(enum com_pin_config dat)
{
    ssd1306_write_param(SET_COM_PINS, dat);
}

/**
 * Double byte command to select 1 out of 256
 * contrast steps. Contrast increases as the value
 * increases.
 * (RESET = 7Fh )
 */
static __inline__ void ssd1306_set_contrast(uint8_t dat)
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
static __inline__ void ssd1306_set_pre_charge_period(uint8_t dat)
{
    ssd1306_write_param(SET_PRE_CHARGE_PER, dat);
}

#define VCOM_DES_65     0b00000000
#define VCOM_DES_77     0b00100000
#define VCOM_DES_83     0b00110000
/**
 * VCOM_DES_65 ~ 0.65 x Vcc
 * VCOM_DES_77 ~ 0.77 x Vcc
 * VCOM_DES_83 ~ 0.83 x Vcc
 */
static __inline__ void ssd1306_set_vcom_detect(uint8_t dat)
{
    ssd1306_write_param(SET_VCOM_DESELECT, dat);
}

/**
 * A4h, X 0 =0b: Resume to RAM content display
 * (RESET)
 */
static __inline__ void ssd1306_display_resume(void)
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
static __inline__ void ssd1306_set_norm_disp(uint8_t ram_opt)
{
    ssd1306_write_byte(SET_NORM_DISP | ram_opt);
}

/**
 * Deactivate scroll
 */
static __inline__ void ssd1306_deactivate_scroll(void)
{
    ssd1306_write_byte(DEACT_SCROLL_CMD);
}

/**
 * Cause why not?
 */
static __inline__ void ssd1306_noop(void)
{
    ssd1306_write_byte(NOOP_CMD);
}

/**
 * Initialize the device
 */
void ssd1306_init()
{
    i2c_init();

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
}

/**
 * Clear the display buffer and clear all bits in GDDRAM
 */
void ssd1306_clear_screen()
{
    _setup_dat_write();

    for(uint16_t i = 0; i < NUM_PAGES*DISPLAY_WIDTH; i++)
    {
        _write_byte(0x00);
    }

    _stop_write();

}

/**
 * Set a pixel in the disp_buf. Will need a call to display for the change
 * to take place
 */
void ssd1306_draw_pixel(uint8_t x, uint8_t y)
{    
    disp_buf[(y/NUM_PAGES)][x] |= (1 << (y % 8));
}

/**
 * Clear a pixel in the disp_buf. Will need a call to display for the change
 * to take place
 */
void ssd1306_clear_pixel(uint8_t x, uint8_t y)
{    
    disp_buf[(y/NUM_PAGES)][x] &= ~(1 << (y % 8));
}

/**
 * Write the entire buffer contents to GDDRAM
 */
void ssd1306_display()
{
    ssd1306_write_dat(&disp_buf[0][0], sizeof disp_buf);
}

int main(void)
{
    ssd1306_init();
    ssd1306_clear_screen();

    uint8_t x = 0, y = 16;
    int8_t xdir = 1, ydir = 0;

    for(uint16_t i = 0; i < 256; i++)
    {
        if(x >= (DISPLAY_WIDTH - 1)) { xdir *= -1; ydir = 1; }
        if(y >= (DISPLAY_HEIGHT/2 - 1)) { ydir *= -1; }

        ssd1306_draw_pixel(x, y);
        ssd1306_display();
        _delay_ms(5);

        x += xdir;
        y += ydir;
    }

    ssd1306_clear_screen();

    while(1) {}
}
