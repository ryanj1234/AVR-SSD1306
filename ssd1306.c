#include <avr/io.h>
#include <util/delay.h>
#include "ssd1306.h"
#include "i2c.h"
#include "font.h"
#include <string.h> // memset

#define PAGE_SIZE       8
#define NUM_PAGES       (SSD1306_DISPLAY_HEIGHT/PAGE_SIZE)

typedef uint8_t ssd1306_page_type;
typedef uint8_t ssd1306_seq_type;

ssd1306_page_type curr_page = 0;
ssd1306_seq_type curr_seq = 0;

#define SSD1306_I2C_ADDR    0x78
#define I2C_WRITE_BIT       0x00
#define I2C_READ_BIT        0x01

#define CTRL_FRAME          0x00
#define CONT_BIT            0b10000000
#define CMD_BIT             0b00000000
#define GDD_BIT             0b01000000

/** Commands */
// 1 fundamental command table
#define SET_CONTRAST_CMD    0x81
#define RESUME_DISP_CMD     0xA4
#define DISP_ON_RESUME_CMD  0xA4
#define SET_NORM_DISP       0xA6
#define SET_DISP_OFF_CMD    0xAE
#define SET_DISP_ON_CMD     0xAF

// 2 scrolling command table
#define RIGHT_HORZ_SCROLL   0x26
#define LEFT_HORZ_SCROLL    0x27
#define VERT_RIGHT_SCROLL   0x29
#define VERT_LEFT_SCROLL    0x2A
#define DEACT_SCROLL_CMD    0x2E
#define ACTIV_SCROLL_CMD    0x2F

// 3 addressing setting command table
#define SET_MEM_ADDR_MODE_CMD   0x20
#define SET_COLUMN_ADDR         0x21
#define SET_PAGE_ADDR           0x22

// 4 hardware configuration
#define SET_DISP_START_LINE_CMD 0x40
#define SET_SEG_REMAP           0xA0
#define SET_MUX_RATIO           0xA8
#define SET_COM_SCAN_DEC        0xC8
#define SET_DISPLAY_OFFSET_CMD  0xD3
#define SET_COM_PINS            0xDA

// 5. Timing & Driving Scheme Setting Command Table
#define SET_CLK_DIV             0xD5
#define SET_PRE_CHARGE_PER      0xD9
#define SET_VCOM_DESELECT       0xDB
#define NOOP_CMD                0xE3

#define SET_CHARGE_PUMP         0x8D

#if SSD1306_GRAPHICS_MODE
static uint8_t disp_buf[NUM_PAGES][SSD1306_DISPLAY_WIDTH];
#endif

static __inline__ void _setup_cmd_write(void)
{
    i2c_start();
    i2c_write(SSD1306_I2C_ADDR);
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
    i2c_write(SSD1306_I2C_ADDR);
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

void ssd1306_write_param2(uint8_t cmd, uint8_t dat1, uint8_t dat2)
{
    _setup_cmd_write();
    _write_byte(cmd);
    _write_byte(dat1);
    _write_byte(dat2);
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
static __inline__ void _set_clk(uint8_t dat)
{
    ssd1306_write_param(SET_CLK_DIV, dat);
}

/**
 * Set MUX ratio to N+1 MUX
 * N=A[5:0] : from 16MUX to 64MUX, RESET=
 * 111111b (i.e. 63d, 64MUX)
 * A[5:0] from 0 to 14 are invalid entry.
 */
static __inline__ void _set_mux_ratio(uint8_t dat)
{
    ssd1306_write_param(SET_MUX_RATIO, dat);
}

/**
 * Set vertical shift by COM from 0d~63d
 * The value is reset to 00h after RESET.
 */
static __inline__ void _set_display_offset(uint8_t dat)
{
    ssd1306_write_param(SET_DISPLAY_OFFSET_CMD, dat);
}

/**
 * Set display RAM display start line register from
 * 0-63 using X 5 X 3 X 2 X 1 X 0 .
 * Display start line register is reset to 000000b
 * during RESET.
 */
static __inline__ void _set_disp_startline(uint8_t dat)
{
    ssd1306_write_byte(SET_DISP_START_LINE_CMD | dat);
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
static __inline__ void _charge_pump_enable(void)
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
static __inline__ void _set_mem_addr_mode(uint8_t dat)
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
static __inline__ void _set_seg_remap(uint8_t dat)
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
static __inline__ void _set_com_scan_dir(uint8_t dat)
{
    ssd1306_write_byte(dat);
}


#define SEQ_COM_PIN     0b00000010
#define ALT_COM_PIN     0b00010010
#define DIS_LR_REMAP    0b00000010
#define EN_LR_REMAP     0b00100010
/**
 * A[4]=0b, Sequential COM pin configuration
 * A[4]=1b(RESET), Alternative COM pin
 * configuration
 * A[5]=0b(RESET), Disable COM Left/Right
 * remap
 * A[5]=1b, Enable COM Left/Right remap
 */
static __inline__ void _set_com_pins(uint8_t dat)
{
    ssd1306_write_param(SET_COM_PINS, dat);
}

/**
 * Double byte command to select 1 out of 256
 * contrast steps. Contrast increases as the value
 * increases.
 * (RESET = 7Fh )
 */
static __inline__ void _set_contrast(uint8_t dat)
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
static __inline__ void _set_pre_charge_period(uint8_t dat)
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
static __inline__ void _set_vcom_detect(uint8_t dat)
{
    ssd1306_write_param(SET_VCOM_DESELECT, dat);
}

/**
 * A4h, X 0 =0b: Resume to RAM content display
 * (RESET)
 */
static __inline__ void _display_resume(void)
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
static __inline__ void _set_norm_disp(uint8_t ram_opt)
{
    ssd1306_write_byte(SET_NORM_DISP | ram_opt);
}

static __inline__ void 
_cont_horz_scroll(uint8_t dir, uint8_t start, uint8_t stop, uint8_t interval)
{
    _setup_cmd_write();
    _write_byte(dir);
    _write_byte(0x00); // dummy byte
    _write_byte(start);
    _write_byte(interval);
    _write_byte(stop);
    _write_byte(0x00); // dummy byte
    _write_byte(0xFF); // dummy byte
    _stop_write();
}

/**
 * Continuous left horizontal scroll from page start to page end (0 - 7).
 * 
 * Intervals:
 *  000b – 5 frames
 *  100b – 3 frames
 *  001b – 64 frames
 *  101b – 4 frames
 *  010b – 128 frames
 *  110b – 25 frame
 *  011b – 256 frames
 *  111b – 2 frame
 */
void ssd1306_scroll_right(uint8_t start, uint8_t stop, uint8_t interval)
{
    _cont_horz_scroll(LEFT_HORZ_SCROLL, start, stop, interval);
}

/**
 * Continuous right horizontal scroll from page start to page end (0 - 7).
 * 
 * Intervals:
 *  000b – 5 frames
 *  100b – 3 frames
 *  001b – 64 frames
 *  101b – 4 frames
 *  010b – 128 frames
 *  110b – 25 frame
 *  011b – 256 frames
 *  111b – 2 frame
 */
void ssd1306_scroll_left(uint8_t start, uint8_t stop, uint8_t interval)
{
    _cont_horz_scroll(RIGHT_HORZ_SCROLL, start, stop, interval);
}

static __inline__ void 
_cont_hv_scroll(uint8_t dir, uint8_t start, uint8_t stop, uint8_t interval, uint8_t offset)
{
    _setup_cmd_write();
    _write_byte(dir);
    _write_byte(0x00); // dummy byte
    _write_byte(start);
    _write_byte(interval);
    _write_byte(stop);
    _write_byte(offset);
    _stop_write();
}

void ssd1306_vert_right_scroll(uint8_t start, uint8_t stop, uint8_t interval, uint8_t offset)
{
    _cont_hv_scroll(VERT_RIGHT_SCROLL, start, stop, interval, offset);
}

void ssd1306_vert_left_scroll(uint8_t start, uint8_t stop, uint8_t interval, uint8_t offset)
{
    _cont_hv_scroll(VERT_LEFT_SCROLL, start, stop, interval, offset);
}

void ssd1306_activate_scroll(void)
{
    ssd1306_write_byte(ACTIV_SCROLL_CMD);
}

void ssd1306_deactivate_scroll(void)
{
    ssd1306_write_byte(DEACT_SCROLL_CMD);
}

/**
 * Cause why not?
 */
static __inline__ void _noop(void)
{
    ssd1306_write_byte(NOOP_CMD);
}

/**
 * Setup column start and end address
 * A[6:0] : Column start address, range : 0-127d,
 * (RESET=0d)
 * B[6:0]: Column end addres, range : 0-127d,
 * (RESET =127d)
 */
static __inline__ void _set_column_addr(uint8_t c)
{
    ssd1306_write_param2(SET_COLUMN_ADDR, c, SSD1306_DISPLAY_WIDTH - 1);
    curr_seq = c;
}

/**
 * Setup page start and end address
 * A[2:0] : Page start Address, range : 0-7d,
 * (RESET = 0d)
 * B[2:0] : Page end Address, range : 0-7d,
 * (RESET = 7d)
 */
static __inline__ void _set_page_addr(uint8_t p)
{
    ssd1306_write_param2(SET_PAGE_ADDR, p, NUM_PAGES - 1);
    curr_page = p;
}

/**
 * Initialize the device. Startup sequence and parameters taken mostly from
 * https://github.com/Sylaina/oled-display/blob/master/lcd.c
 * and
 * https://github.com/adafruit/Adafruit_SSD1306/blob/master/Adafruit_SSD1306.cpp
 */
void ssd1306_init()
{
    i2c_init();

    ssd1306_disp_off();
    _set_clk(0x80);
    _set_mux_ratio(SSD1306_DISPLAY_HEIGHT - 1);

    _set_display_offset(0x00);
    _set_disp_startline(0x00);
    _charge_pump_enable();
    _set_mem_addr_mode(HORZ_ADDR_MODE);
    _set_seg_remap(SEG_INVERT);
    _set_com_scan_dir(COM_SCAN_INV);

    #if SSD1306_USE_BIG_CHARS
    _set_com_pins(SEQ_COM_PIN);
    #else
    _set_com_pins(ALT_COM_PIN);
    #endif

    _set_pre_charge_period(0xF1);
    _set_vcom_detect(VCOM_DES_77); // different from adafruit, investigate
    ssd1306_deactivate_scroll();
    _display_resume();
    _set_norm_disp(RAM_OFF_DISP);

    ssd1306_disp_on();
}

void ssd1306_clear_line(uint8_t p)
{
    ssd1306_set_cursor(0, p);
    _setup_dat_write();
    for(uint16_t j = 0; j < SSD1306_DISPLAY_WIDTH; j++)
    {
        _write_byte(0x00);
    }

    _stop_write();
}

/**
 * Clear the display and remember the previous location of the cursor
 */
void ssd1306_clear_screen()
{
    ssd1306_page_type old_page = curr_page;
    ssd1306_seq_type old_seq = curr_seq;
    ssd1306_set_cursor(0, 0);

    uint8_t dat = 0;
    for(uint16_t i = 0; i < NUM_PAGES*SSD1306_DISPLAY_WIDTH; i++)
    {
        ssd1306_write_dat(&dat, 1);
    }

    ssd1306_set_cursor(old_seq, old_page);
}

/**
 * Set a pixel in the disp_buf. Will need a call to display for the change
 * to take place
 */
void ssd1306_draw_pixel(uint8_t x, uint8_t y)
{
    #if SSD1306_GRAPHICS_MODE
    disp_buf[(y/NUM_PAGES)][x] |= (1 << (y % 8));
    #endif
}

/**
 * Clear a pixel in the disp_buf. Will need a call to display for the change
 * to take place
 */
void ssd1306_clear_pixel(uint8_t x, uint8_t y)
{
    #if SSD1306_GRAPHICS_MODE
    disp_buf[(y/NUM_PAGES)][x] &= ~(1 << (y % 8));
    #endif
}

/**
 * Write the entire buffer contents to GDDRAM
 */
void ssd1306_display()
{
    #if SSD1306_GRAPHICS_MODE
    ssd1306_write_dat(&disp_buf[0][0], sizeof disp_buf);
    #endif
}

/**
 * Display a character on the screen at the current page/sequence location. If
 * character will not fit will wrap to the next line.
 * 
 * Additionally, newline characters are recongnized and will cause the
 * current page number to increment
 */
void ssd1306_putc(char c)
{
    if(c == '\n')
    {
        ssd1306_set_cursor(0, curr_page + 1);
        return;
    }

    if(curr_seq >= (SSD1306_DISPLAY_WIDTH - FONT_WIDTH))
    {
        ssd1306_set_cursor(0, curr_page + 1);
    }

    uint8_t data[FONT_WIDTH];
    char2pixels(c, data);

    #if SSD1306_USE_BIG_CHARS
    ssd1306_page_type old_page = curr_page;
    ssd1306_seq_type old_seq = curr_seq;
    ssd1306_write_dat(data, FONT_WIDTH);
    ssd1306_set_cursor(old_seq, old_page + NUM_PAGES/2);
    ssd1306_write_dat(data, FONT_WIDTH);
    ssd1306_set_cursor(old_seq+FONT_WIDTH, old_page);
    #else
    ssd1306_write_dat(data, FONT_WIDTH);
    curr_seq += FONT_WIDTH;
    #endif
}

/**
 * Print a string to the screen
 */
void ssd1306_puts(const char* s)
{
    while(*s != 0)
    {
        ssd1306_putc(*s++);
    }
}

/**
 * Set location on display where characters will be printed. x_pix is a pixel
 * ranging from 0 to display width - 1. page is a y location ranging from 0 to
 * number of pages - 1 (0-7 for a 64 segment display)
 */
void ssd1306_set_cursor(uint8_t x_pix, uint8_t page)
{
    _set_page_addr(page);
    _set_column_addr(x_pix);
}

int main(void)
{
    ssd1306_init();
    ssd1306_clear_screen();
    for(uint8_t i = 0; i < 7; i++)
    {
        ssd1306_puts("FUCK YOU FUCK YOU\n");
    }    

    _delay_ms(10);
    ssd1306_activate_scroll();

    while(1) 
    {
        ssd1306_vert_right_scroll(0, 7, 6, 63);
        _delay_ms(1000);
        ssd1306_vert_left_scroll(0, 7, 6, 63);
        _delay_ms(1000);
    }
}
