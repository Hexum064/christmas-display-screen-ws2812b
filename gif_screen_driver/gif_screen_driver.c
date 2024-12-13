#include <stdio.h>
#include <stdlib.h>
//
#include "f_util.h"
#include "ff.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"

#include "ssd1306.h"

#include "hw_config.h"
#include <string.h>
#include "gifdec.h"


//Generate frame
//Shift MSB of each current byte of 48 scan lines into 6 bytes, bottom first
//--bottom first means, the first bit to go out over spi will be the MSB of the 48th line's current byte
//Output, via spi1, all 1s for 6 bytes  (48 bits worth);
//Output, via spi1, the 6 bytes that the actual data was shifted into
//Pulse clear pin to reset to 0's
//--next set of bits should be shifted in right after this as the next data should go out within about 200ns

//for frame decoding, when gif is loaded, create 2 frame buffers, w*48*3 in size.
//--the image will always be shown at the top of the screen. Making the buffer 48 in height makes
//--it easier to output (there will always be 48 lines)
//--the number of bytes to output per line will always be size/48
//Calculate the number of bytes that need to be output to center the image based on w.
//--these bytes will be output x number of times to center the image before the frame buffer is output
//Clear the screen
//--have a function that outputs 48*80 bytes of 0
//When calling the screen output function, pass the pointer to the frame buffer.

//let the second core handle the output
//The assumption is that one frame will always be able to be output faster than the next frame will be ready
//Min time between frames for a gif is 10ms
//max time to output a frame is: (80(w)*48(h)*3(rgb)*8(bits)) = 92160 bits to output
//--48 bits will be output via spi at 24mhz. A pattern of all 1's the the actual data will be output. 
//--theoretically, 48 bits should take 0.000000083 seconds or 83ns, which falls well within the roughly 
//--250ns timing for each part of the output for a bit for the ws2812 
//---- a period of about 250ns hi, then 250ns hi or low for the actual data, then about 300ns low.
//--So each bit for a column of data will take around 800ns. 800ns * (80*3*8) is about 1.6ms for each frame
//---- 80cols with 3 bytes of color data at 8 bits each. 

#define PICO_CLK_KHZ 133000UL 
#define FRAME_ALARM_MS 10
#define FRAME_OUTPUT_BAUD 24000000UL

#define SPI1_SCK_GPIO 10
#define SPI1_MOSI_GPIO 11
#define SPI1_MISO_GPIO 8 //probably not going to be used
#define SPI1_LATCH_GPIO 9
#define SPI1_OE_GPIO 8

#define DISP_SDA 2
#define DISP_SCL 3
#define DISP_I2C_FREQ 400000

#define SCREEN_ROWS 48
#define GIF_CYCLE_MS 30000 //(30s)

ssd1306_t disp;

alarm_pool_t *core_0_alarm;
repeating_timer_t frame_alarm;

uint8_t * frame_buff_0 = 0;
uint8_t * frame_buff_1 = 0;
uint8_t * output_buff = 0;
uint8_t * fill_buff = 0;


uint8_t * col_data = 0;
uint8_t all_on_col_data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t frame_col_data[6] = {0};

uint16_t frame_delay_count = 0xFFFF;
uint16_t gif_cycle_count = 0;


uint spi1_dma_channel;
dma_channel_config spi1_dma_config;
gd_GIF * cur_gif = 0;

FIL fp;
DIR dir;
FILINFO fno;

//Handles:
//  outputting a frame based on frame delay
//  counting how many seconds a gif has been playing, so to signal next gif
//  debouncing the "next gif" button
bool frame_alarm_handler(repeating_timer_t * fa);

inline void sleep_100ns()
{
    //@133mHz, each clock is 7.52ns
    //13 nops = 13 * 7.52 = 97.8ns
    __asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
   
}

void alarm_init()
{
    core_0_alarm = alarm_pool_create_with_unused_hardware_alarm(1);
    alarm_pool_add_repeating_timer_ms(core_0_alarm, FRAME_ALARM_MS, &frame_alarm_handler, 0, &frame_alarm);
}

void io_init()
{
    // Initialize Latch pin low
    gpio_init(SPI1_LATCH_GPIO);
    gpio_set_dir(SPI1_LATCH_GPIO, GPIO_OUT);
    gpio_put(SPI1_LATCH_GPIO, 0);

    //Initialize OE pin high
    gpio_init(SPI1_OE_GPIO);
    gpio_set_dir(SPI1_OE_GPIO, GPIO_OUT);
    gpio_put(SPI1_OE_GPIO, 0);

    //Init "next button" pin
}

void spi1_init()
{
    spi_init(spi1, FRAME_OUTPUT_BAUD);
    spi_set_format(spi1, 8, SPI_CPOL_0 , SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(spi1, false);

    // Initialize SPI pins
    gpio_set_function(SPI1_SCK_GPIO, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MOSI_GPIO, GPIO_FUNC_SPI);
    // gpio_set_function(SPI1_MISO_GPIO, GPIO_FUNC_SPI); //don't need input

}

void spi1_dma_init()
{
    spi1_dma_channel = dma_claim_unused_channel(true);
    spi1_dma_config = dma_channel_get_default_config(spi1_dma_channel);
    channel_config_set_transfer_data_size(&spi1_dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&spi1_dma_config, true);
    channel_config_set_write_increment(&spi1_dma_config, false);
    channel_config_set_dreq(&spi1_dma_config, DREQ_SPI1_TX);
    dma_channel_configure(spi1_dma_channel, &spi1_dma_config, &spi1_hw->dr, col_data, 6, false);
}

void display_i2c_init()
{
    i2c_init(i2c1, DISP_I2C_FREQ);
    gpio_set_function(DISP_SDA, GPIO_FUNC_I2C);
    gpio_set_function(DISP_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(DISP_SDA);
    gpio_pull_up(DISP_SCL);
}

void display_init()
{
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 32, 0x3C, i2c1);
    ssd1306_clear(&disp);    
}

void output_column()
{
    #ifdef DEBUG
        //This will screw up the timing
        printf("%02x, %02x, %02x, %02x, %02x, %02x\n", col_data[0], col_data[1], col_data[2], col_data[3], col_data[4], col_data[5]);
    #endif

    //Set up for the WS2812b timings

    //--First part of bit: all on for 250ns
    col_data = all_on_col_data;    
    dma_start_channel_mask(spi1_dma_channel);
    //wait for about 150-200ns.
    sleep_100ns();
    
    //  This is so this function can be called immediately after the previous run
    //  This is a total of approx. 300ns of low period before the next byte
    //pulse latch pin
    gpio_put(SPI1_LATCH_GPIO, 1);
    sleep_100ns();
    gpio_put(SPI1_LATCH_GPIO, 0);

    //set oe pin low. All outputs should turn on
    gpio_put(SPI1_OE_GPIO, 0);
    dma_channel_wait_for_finish_blocking(spi1_dma_channel);
    //---

    //--Second part of bit: the actual data
    col_data = frame_col_data;
    dma_start_channel_mask(spi1_dma_channel);
    //wait for about 150-200ns (this wait is for the first hi part of the bit)
    sleep_100ns();
    sleep_100ns();

    //pulse latch pin
    //Since the OE is already low(enabled), this latch should show up on the outputs
    //of the shift registers immediately 
    //And wait about 200ns
    gpio_put(SPI1_LATCH_GPIO, 1);
    sleep_100ns();
    gpio_put(SPI1_LATCH_GPIO, 0);    
    sleep_100ns();
    //---

    //--Third part of bit: the low part of the bit
    //set oe pin hi (disable outputs)
    //the wait will happen when this function is called again
    gpio_put(SPI1_OE_GPIO, 1);
    //---

    
}

void output_left_blank(uint8_t len)
{
    frame_col_data[0] = 0;
    frame_col_data[1] = 0;
    frame_col_data[2] = 0;
    frame_col_data[3] = 0;
    frame_col_data[4] = 0;
    frame_col_data[5] = 0;

    for(uint16_t i = 0; i < (len * 8 * 3); i++)
    {
        output_column();
    }
}

void shift_next_bits(uint8_t col_and_color, uint8_t width, uint8_t bit)
{
    uint8_t mask = 1 << bit;
    uint16_t i = col_and_color;
    uint8_t offset = width * 3; // * 3 for 3 bytes per pixel (rgb)

    for (uint8_t b = 0; b < 6; b++)
    {
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x01 : 0;
        i += offset;
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x02 : 0;
        i += offset;
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x04 : 0;
        i += offset;
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x08 : 0;
        i += offset;
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x10 : 0;
        i += offset;
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x20 : 0;
        i += offset;
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x40 : 0;
        i += offset;
        frame_col_data[b] |= (output_buff[i] & mask) ? 0x80 : 0;
        i += offset;
    }

    output_column();
}

//call on core1
void draw_screen()
{
    uint8_t width = cur_gif->width * 3;
    
    for (uint16_t c = 0; c < width; c++)
    {   
        shift_next_bits(c, width, 0);
        shift_next_bits(c, width, 1);
        shift_next_bits(c, width, 2);
        shift_next_bits(c, width, 3);
        shift_next_bits(c, width, 4);
        shift_next_bits(c, width, 5);
        shift_next_bits(c, width, 6);
        shift_next_bits(c, width, 7);
    }
    multicore_reset_core1();
}

void clear_screen()
{
    frame_col_data[0] = 0;
    frame_col_data[1] = 0;
    frame_col_data[2] = 0;
    frame_col_data[3] = 0;
    frame_col_data[4] = 0;
    frame_col_data[5] = 0;

    for(uint16_t i = 0; i < (80 * 8 * 3); i++)
    {
        output_column();
    }
}

void gif_load()
{
    while(1)
    {

        if (fno.fname[0] = 0)
        {
            f_findfirst(&dir, &fno, "/", "*.gif");
        }
        else 
        {
            f_findnext(&dir, &fno);
        }
        
        if (cur_gif)
        {
            gd_close_gif(cur_gif);
        }

        f_open(&fp, fno.fname, FA_READ);
        cur_gif = gd_open_gif(&fp);

        if (cur_gif)
        {
            if (fill_buff)
            {
                free(fill_buff);
            }

            if (output_buff)
            {
                free(output_buff);
            }

            fill_buff = (uint8_t *)malloc(cur_gif->width * cur_gif->height * 3);
            output_buff = (uint8_t *)malloc(cur_gif->width * cur_gif->height * 3);
            //TODO: show current gif file name on OLED
            ssd1306_clear_square(&disp, 0, 8, 128, 32);
            ssd1306_draw_string(&disp, 0, 8, 1, fno.fname);
            ssd1306_show(&disp);            
            break;
        }
    }
}

void get_next_frame()
{
    uint8_t * temp;
    if (gd_get_frame(cur_gif))
    {
        gd_rewind(cur_gif);
        gd_get_frame(cur_gif);
    }

    gd_render_frame(cur_gif, fill_buff);
    //May have to subtract 1 from frame_delay_count first
    frame_delay_count = cur_gif->gce.delay;

    //swap pointers
    temp = fill_buff;
    fill_buff = output_buff;
    output_buff = temp;
   
    multicore_launch_core1(draw_screen);
}

//Reset the long counter
//Clear the screen
//Get the next gif from the SD and open it
//TODO: Start the timer (maybe)
void gif_animation_init()
{    
    gif_cycle_count = GIF_CYCLE_MS;
    gif_load();
    clear_screen();
    frame_delay_count = 0; //start immediately
}

bool frame_alarm_handler(repeating_timer_t * fa)
{
    //TODO button debounce

    if (gif_cycle_count == 0)
    {
        gif_cycle_count = GIF_CYCLE_MS;
        gif_animation_init();
        return true;
    }

    gif_cycle_count -= 10; //this is a 10ms timer
    //TODO: show time left on cycle on OLED

    if (frame_delay_count == 0)
    {
        get_next_frame();

    }

    frame_delay_count--;


    
    

    return true;
}

int main() {
    set_sys_clock_khz(PICO_CLK_KHZ, true);

    stdio_init_all();
    display_i2c_init();    
    io_init();
    spi1_init();
    spi1_dma_init();
    display_init();
    //TODO: show "Loading" message on OLED

    ssd1306_draw_string(&disp, 0, 0, 1, "Loading");
    ssd1306_show(&disp);
sleep_ms(2000);

    puts("Hello, world!");

    // See FatFs - Generic FAT Filesystem Module, "Application Interface",
    // http://elm-chan.org/fsw/ff/00index_e.html
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);

    if (fr != FR_OK)
    {
        //TODO: show error on OLED
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr); 
        return fr;
    }
    
    fr = f_opendir(&dir, "/");

    if (fr != FR_OK)
    {
        //TODO: show error on OLED
        printf("f_opendir error: %s (%d)\n", FRESULT_str(fr), fr); 
        return fr;
    }

    //TODO: show "displaying gifs" message on OLED
    ssd1306_clear_square(&disp, 0, 0, 128, 32);
    ssd1306_draw_string(&disp, 0, 0, 1, "displaying gifs");
    ssd1306_show(&disp);
    //Start the timer and let it run
    alarm_init();
    
/*

    if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    FIL fil;
    const char* const filename = "filename.txt";
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&fil, "Hello, world!\n") < 0) {
        printf("f_printf failed\n");
    }

    DIR dir;
    

    if (fr != FR_OK) {
        printf("f_opendir error: %s (%d)\n", FRESULT_str(fr), fr);        
    }
    else {

        FILINFO fno;

        f_findfirst(&dir, &fno, "/", "*.gif");

        if (fno.fname[0] != 0)
        {
            gd_GIF *gif;
            FIL fp;
           
            fr = f_open(&fp, fno.fname, FA_READ);
            
            if (fr != FR_OK)
            {
                printf("Could not open file: %s\n\t%s (%d)\n", fno.fname, FRESULT_str(fr), fr);
            }
            else
            {
                 printf("Opening %s\n", fno.fname);
                gif = gd_open_gif(&fp);

                if (!gif)
                {
                    printf("Could not open gif.\n");
                }
                else
                {
                    printf("w: %d, h: %d\n", gif->width, gif->height);
                    uint8_t *buffer = (uint8_t*)malloc(gif->width * gif->height * 3);
                    uint16_t frame_i = 0;
                    for (uint16_t looped = 1;; looped++) {
                        frame_i = 0;
                        while (gd_get_frame(gif)) {
                            gd_render_frame(gif, buffer);

                            printf("frame %d\n", frame_i++);
                            sleep_ms(gif->gce.delay * 10);
                        }
                        if (looped == gif->loop_count)
                            break;
                        gd_rewind(gif);
                    }
                    free(buffer);
                    gd_close_gif(gif);
                }

            }

            // gd_open_gif()
        }

        // while(fno.fname[0] != 0)
        // {
        //     printf("\tgif: %s\n", fno.fname); // Print File Name
        //     f_findnext(&dir, &fno);
        // }
            
    }

    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
    f_unmount(pSD->pcName);

    puts("Goodbye, world!");
    */
    for (;;);
}
