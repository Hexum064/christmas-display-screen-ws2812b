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
//look to the output_column function for the output pattern

#define PICO_CLK_KHZ 133000UL 
#define FRAME_ALARM_MS 10 //TODO: change back to 10

#define DISP_IO_0 8
#define DISP_IO_1 9
#define DISP_IO_2 10
#define DISP_IO_3 11
#define DISP_IO_4 12
#define DISP_IO_5 13
#define DISP_IO_6 14
#define DISP_IO_7 15

#define DISP_OE_IO_A 2
#define DISP_OE_IO_B 3
#define DISP_OE_IO_C 4
#define DISP_OE_IO_D 5
#define DISP_OE_IO_E 6
#define DISP_OE_IO_F 7

#define BUTTON_IO 28

#define DISP_SDA 0
#define DISP_SCL 1
#define DISP_I2C_FREQ 400000
#define DISP_I2C i2c0

#define SCREEN_ROWS 48
#define SCREEN_COLS 80
#define GIF_CYCLE_MS 60000 //(30s)
#define SCREEN_GROUPS 6

#define BUTTON_DEBOUNCE 5 //(50ms)

//#define USE_MULTICORE
// #define DEBUG

uint32_t disp_io_mask = ( 1 << DISP_IO_0) |
                        ( 1 << DISP_IO_1) |
                        ( 1 << DISP_IO_2) |
                        ( 1 << DISP_IO_3) |
                        ( 1 << DISP_IO_4) |
                        ( 1 << DISP_IO_5) |
                        ( 1 << DISP_IO_6) |
                        ( 1 << DISP_IO_7);

uint16_t disp_oe_io_mask =  ( 1 << DISP_OE_IO_A) |                       
                            ( 1 << DISP_OE_IO_B) |
                            ( 1 << DISP_OE_IO_C) |
                            ( 1 << DISP_OE_IO_D) |
                            ( 1 << DISP_OE_IO_E) |
                            ( 1 << DISP_OE_IO_F);

uint32_t disp_io_arr[8] = {DISP_IO_0, DISP_IO_1, DISP_IO_2, DISP_IO_3, DISP_IO_4, DISP_IO_5, DISP_IO_6, DISP_IO_7};
uint32_t disp_oe_io_arr[6] = {DISP_OE_IO_A, DISP_OE_IO_B, DISP_OE_IO_C, DISP_OE_IO_D, DISP_OE_IO_E, DISP_OE_IO_F};
uint8_t line_bit_mask[] = {
    0x01,
    0x02,
    0x04,
    0x08,
    0x10,
    0x20,
    0x40,
    0x80
};

uint8_t gamma_correction[] = {0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
1,
2,
2,
2,
2,
2,
2,
2,
2,
2,
2,
3,
3,
3,
3,
3,
3,
3,
3,
4,
4,
4,
4,
4,
4,
4,
5,
5,
5,
5,
5,
6,
6,
6,
6,
6,
7,
7,
7,
7,
7,
8,
8,
8,
8,
9,
9,
9,
9,
10,
10,
10,
10,
11,
11,
11,
12,
12,
12,
12,
13,
13,
13,
14,
14,
14,
15,
15,
16,
16,
16,
17,
17,
17,
18,
18,
19,
19,
19,
20,
20,
21,
21,
22,
22,
22,
23,
23,
24,
24,
25,
25,
26,
26,
27,
27,
28,
28,
29,
30,
30,
31,
31,
32,
32,
33,
34,
34,
35,
35,
36,
37,
37,
38,
38,
39,
40,
40,
41,
42,
42,
43,
44,
45,
45,
46,
47,
48,
48,
49,
50,
51,
51,
52,
53,
54,
55,
55,
56,
57,
58,
59,
60,
60,
61,
62,
63,
64,
65,
66,
67,
68,
69,
69,
70,
71,
72,
73,
74,
75,
76,
77,
78,
79,
80,
81,
83,
84,
85,
86,
87,
88,
89,
90,
91,
92,
94,
95,
96,
97,
98,
99,
101,
102,
103,
104,
106,
107,
108,
109,
111,
112,
113,
114,
116,
117,
118,
120,
121,
122,
124,
125,
127,
128
};

uint32_t bit_vals[8] = {0};


ssd1306_t disp;

alarm_pool_t *core_0_alarm;
repeating_timer_t frame_alarm;


uint8_t screen_buff[48][80*3] = {0};
uint8_t * frame_buff = 0;

uint16_t frame_delay_count = 0xFFFF;
uint16_t gif_cycle_count = 0;

gd_GIF * cur_gif = 0;

FIL fp;
DIR dir;
FILINFO fno;

bool process_next_frame = false;
bool button_down = false;
bool force_next_gif = false;
uint8_t button_down_count = 0;

//Handles:
//  outputting a frame based on frame delay
//  counting how many seconds a gif has been playing, so to signal next gif
//  debouncing the "next gif" button
bool frame_alarm_cb(repeating_timer_t * fa);

inline void sleep_100ns()
{
    //@133mHz, each clock is 7.52ns
    //13 nops = 13 * 7.52 = 97.8ns
    __asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
   
}

inline void sleep_40ns()
{
    //@133mHz, each clock is 7.52ns
    //13 nops = 5 * 7.52 = 37.6ns
    __asm volatile ("nop\nnop\nnop\nnop\nnop\n");
   
}

void bit_vals_init()
{
    for (uint8_t i = 0; i < 8; i++)
    {
        bit_vals[i] = (1 << disp_io_arr[i]);
    }
}

void alarm_init()
{
    #ifdef DEBUG
    printf("Alarm initializing\n");
#endif
    core_0_alarm = alarm_pool_create_with_unused_hardware_alarm(1);
    alarm_pool_add_repeating_timer_ms(core_0_alarm, FRAME_ALARM_MS, &frame_alarm_cb, 0, &frame_alarm);
}

void io_init()
{
    //Display data and output enable
    gpio_init_mask(disp_io_mask);
    gpio_init_mask(disp_oe_io_mask);
    gpio_set_dir_out_masked(disp_io_mask);
    gpio_set_dir_out_masked(disp_oe_io_mask);
    gpio_put_masked(disp_io_mask, 0),
    gpio_put_masked(disp_oe_io_mask, 1),
    
    //"Next" Button
    gpio_init(BUTTON_IO);
    gpio_set_dir(BUTTON_IO, false);
    gpio_pull_down(BUTTON_IO);
    
}

void display_i2c_init()
{
    i2c_init(DISP_I2C, DISP_I2C_FREQ);
    gpio_set_function(DISP_SDA, GPIO_FUNC_I2C);
    gpio_set_function(DISP_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(DISP_SDA);
    gpio_pull_up(DISP_SCL);
}

void display_init()
{
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 32, 0x3C, DISP_I2C);
    ssd1306_clear(&disp);    
}



// void output_column()
// {

// }

// void output_left_blank(uint8_t len)
// {
//     frame_col_data[0] = 0;
//     frame_col_data[1] = 0;
//     frame_col_data[2] = 0;
//     frame_col_data[3] = 0;
//     frame_col_data[4] = 0;
//     frame_col_data[5] = 0;

//     for(uint16_t i = 0; i < (len * 8 * 3); i++)
//     {
//         output_column();
//     }
// }

// void shift_next_bits(uint8_t col_and_color, uint8_t width, uint8_t bit)
// {
//     uint8_t mask = 1 << bit;
//     uint16_t i = col_and_color;
//     uint8_t offset = width * 3; // * 3 for 3 bytes per pixel (rgb)

//     //Gotta clear the frame_col_data buffer or we will just keep adding data to it 
//     frame_col_data[0] = 0;
//     frame_col_data[1] = 0;
//     frame_col_data[2] = 0;
//     frame_col_data[3] = 0;
//     frame_col_data[4] = 0;
//     frame_col_data[5] = 0;

//     for (uint8_t b = 0; b < 6; b++)
//     {
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x01 : 0;
//         i += offset;
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x02 : 0;
//         i += offset;
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x04 : 0;
//         i += offset;
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x08 : 0;
//         i += offset;
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x10 : 0;
//         i += offset;
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x20 : 0;
//         i += offset;
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x40 : 0;
//         i += offset;
//         frame_col_data[b] |= (output_buff[i] & mask) ? 0x80 : 0;
//         i += offset;
//     }

//     output_column();
// }

// //call on core1
// void draw_screen()
// {
//     uint8_t width = cur_gif->width * 3;
    
// #ifdef DEBUG
//     printf("drawing left blank\n");
// #endif

//     output_left_blank((80 - cur_gif->width) / 2);

// #ifdef DEBUG
//     printf("drawing frame\n");
// #endif



//     for (uint16_t c = 0; c < width; c++)
//     {   
//         shift_next_bits(c, width, 0);
//         shift_next_bits(c, width, 1);
//         shift_next_bits(c, width, 2);
//         shift_next_bits(c, width, 3);
//         shift_next_bits(c, width, 4);
//         shift_next_bits(c, width, 5);
//         shift_next_bits(c, width, 6);
//         shift_next_bits(c, width, 7);
//     }
//     multicore_reset_core1();
// }

// void clear_screen()
// {
//     frame_col_data[0] = 0;
//     frame_col_data[1] = 0;
//     frame_col_data[2] = 0;
//     frame_col_data[3] = 0;
//     frame_col_data[4] = 0;
//     frame_col_data[5] = 0;

//     for(uint16_t i = 0; i < (80 * 8 * 3); i++)
//     {
//         output_column();
//     }
// }

void clear_screen()
{
    gpio_put_masked(disp_oe_io_mask, 0); //enable all outputs
    for(uint16_t i = 0; i < (8 * 3 * 81); i++)
    {
        //first all hi
        gpio_put_masked(disp_io_mask, disp_io_mask);
        sleep_100ns();
        sleep_100ns();
        //then all low for the rest of the time
        gpio_put_masked(disp_io_mask, 0);
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
    }
    gpio_put_masked(disp_oe_io_mask, disp_oe_io_mask); //disable all outputs
}

void output_group(uint8_t group)
{
    uint16_t col = 0;
    uint8_t bit = 0;
    uint8_t line_offset = group * 8;
    uint32_t data;
    uint8_t byte_0 = 0;
    uint8_t byte_1 = 0;
    uint8_t byte_2 = 0;
    uint8_t byte_3 = 0;
    uint8_t byte_4 = 0;
    uint8_t byte_5 = 0;
    uint8_t byte_6 = 0;
    uint8_t byte_7 = 0;

    uint32_t irq_status = 0;

    //First enable the ouput for the specific group with all io's low
    gpio_put_masked(disp_io_mask, 0);
    //output bits are inverted because output is enabled low
    gpio_put_masked(disp_oe_io_mask, ~(1 << disp_oe_io_arr[group]));

    for (col = 0; col < 80 * 3; col++ )
    {
        bit = 8;

        byte_0 = screen_buff[line_offset + 0][col];
        byte_1 = screen_buff[line_offset + 1][col];
        byte_2 = screen_buff[line_offset + 2][col];
        byte_3 = screen_buff[line_offset + 3][col];
        byte_4 = screen_buff[line_offset + 4][col];
        byte_5 = screen_buff[line_offset + 5][col];
        byte_6 = screen_buff[line_offset + 6][col];
        byte_7 = screen_buff[line_offset + 7][col];

        data = 0;

        irq_status = save_and_disable_interrupts();
        while(bit--)
        {

            //The way this works. First, it is not in a for loop to make it run a bit faster
            //data starts at 0. Then, for each line that is going to be output, if the bit in
            //the screen buff at the current bit pos is 1, then the bit number in the data byte
            //that corresponds to the line number, is set. This is achieved by anding the current
            //screen byte with the mask for the current bit pos, instead of shifting the screen byte
            //to the current bit, which could take more cycles. 
            //It might be possible to speed things up by getting the 8 screen buff bytes ahead of time.
            data += (byte_0 & line_bit_mask[bit]) ? bit_vals[0] : 0;
            data += (byte_1 & line_bit_mask[bit]) ? bit_vals[1] : 0;
            data += (byte_2 & line_bit_mask[bit]) ? bit_vals[2] : 0;
            data += (byte_3 & line_bit_mask[bit]) ? bit_vals[3] : 0;
            data += (byte_4 & line_bit_mask[bit]) ? bit_vals[4] : 0;
            data += (byte_5 & line_bit_mask[bit]) ? bit_vals[5] : 0;
            data += (byte_6 & line_bit_mask[bit]) ? bit_vals[6] : 0;
            data += (byte_7 & line_bit_mask[bit]) ? bit_vals[7] : 0;

            //start off with the outputs all hi
            gpio_put_masked(disp_io_mask, disp_io_mask);
            sleep_100ns();
            sleep_100ns();
            sleep_100ns();
            //set the outputs to the actual data
            gpio_put_masked(disp_io_mask, data);
            sleep_100ns();
            sleep_100ns();
            sleep_100ns();
            sleep_100ns();
            //ending low
            gpio_put_masked(disp_io_mask, 0);
            // sleep_100ns();
            // sleep_100ns();                        
            // sleep_40ns();
        }
         restore_interrupts(irq_status);
    
    }

}

void gif_load()
{
    FRESULT fr;

    while(1)
    {


        if (dir.pat[0] == 0)
        {
#ifdef DEBUG
    printf("finding first gif file\n");
#endif  
            fr = f_findfirst(&dir, &fno, "/", "*.gif");

            if (fr != FR_OK)
            {
                ssd1306_clear_square(&disp, 0, 1, 128, 32);
                ssd1306_draw_string(&disp, 0, 1, 1, "can't find gifs");
                ssd1306_show(&disp);                  
                printf("Could not find gifs: \n\t%s (%d)\n", FRESULT_str(fr), fr);
                panic("Could not find any gifs.");
            }
        }
        else 
        {
#ifdef DEBUG
    printf("finding next gif file\n");
#endif  

            f_findnext(&dir, &fno);

            if (fr != FR_OK)
            {
                ssd1306_clear_square(&disp, 0, 1, 128, 32);
                ssd1306_draw_string(&disp, 0, 1, 1, "can't load gif");
                ssd1306_show(&disp); 
                printf("Could not find more gifs: \n\t%s (%d)\n", FRESULT_str(fr), fr);
                panic("Could not find more gifs.");
            }

            if (fno.fname[0] == 0)
            {
                //No more files were found. Start over
                dir.pat = "";
                continue;
            }
        }
        
#ifdef DEBUG
    printf("gif %s found\n", fno.fname);    
#endif

        if (cur_gif)
        {
            gd_close_gif(cur_gif);
        }

        fr = f_open(&fp, fno.fname, FA_READ);

        if (fr != FR_OK)
        {
            printf("Could not open file %s\n\t%s (%d)\n", FRESULT_str(fr), fr);
            printf("could not open file %s\n", fno.fname);             
            continue;
        }

        cur_gif = gd_open_gif(&fp);



        if (cur_gif)
        {
#ifdef DEBUG
    printf("gif opened\n");
#endif

            if (frame_buff)
            {
                free(frame_buff);
            }



            frame_buff = (uint8_t *)malloc(cur_gif->width * cur_gif->height * 3);
            

#ifdef DEBUG
    printf("buffs created. Size: %d\n", cur_gif->width * cur_gif->height * 3);
#endif

            ssd1306_clear_square(&disp, 0, 8, 128, 32);
            ssd1306_draw_string(&disp, 0, 8, 1, fno.fname);
            ssd1306_show(&disp);            
            break;
        }
    }
}

//TODO:should the frame buff be written to the screen buff immediately, or just after the screen buff is used
void write_frame_to_screen_buff()
{
    uint8_t left_offset = ((SCREEN_COLS - cur_gif->width) / 2) * 3;
    uint16_t frame_y_offset = 0;
    //clear the screen first so we only have to copy the area that the frame takes up
    memset(screen_buff, 0, sizeof(screen_buff));

    //We will assume for this version that the gif height will always be <= the actual screen height
    for (uint8_t y = 0; y < cur_gif->height; y++)
    {
        
        frame_y_offset = y * cur_gif->width * 3;
        //data in the row (x) direction will account for 3 bytes of color
        for (uint16_t x = 0; x < cur_gif->width * 3; x+=3)
        {
            //need to swap red and green
            screen_buff[y][x + left_offset + 0] = frame_buff[x + frame_y_offset + 1] /2;// gamma_correction[frame_buff[x + frame_y_offset]];
            screen_buff[y][x + left_offset + 1] = frame_buff[x + frame_y_offset + 0] /2;
            screen_buff[y][x + left_offset + 2] = frame_buff[x + frame_y_offset + 2] /2;
        }
    }

}

void draw_screen()
{
    //copy the screen here because the previous draw should be done    
    write_frame_to_screen_buff();

    gpio_put_masked(disp_io_mask, 0);
    gpio_put_masked(disp_oe_io_mask, 0);

    for (uint8_t i = 0; i < SCREEN_GROUPS; i++)
    {
        output_group(i);
    }

    gpio_put_masked(disp_io_mask, 0);
    gpio_put_masked(disp_oe_io_mask, 0);

    
    
}

void get_next_frame()
{
    uint8_t * temp;
    if (!gd_get_frame(cur_gif))
    {
        
        gd_rewind(cur_gif);
        gd_get_frame(cur_gif);
    }
    
    gd_render_frame(cur_gif, frame_buff);
    //May have to subtract 1 from frame_delay_count first
    frame_delay_count = cur_gif->gce.delay;

#ifdef USE_MULTICORE
    multicore_reset_core1();
    multicore_launch_core1(draw_screen);
#else
    draw_screen();
#endif

#ifdef DEBUG
    printf("ready to prep next frame\n");
#endif  
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

void frame_handler()
{

    static uint32_t prev_cycles = 0;
    char cycles[32];
#ifdef DEBUG

    // f_findfirst(&dir, &fno, "/", "*.gif");
    // printf("first gif: %s\n", fno.fname);
    // f_open(&fp, fno.fname, FA_READ);
    // printf("file opened. size: %d\n", fp.obj.fs->fsize);
    // cur_gif = gd_open_gif(&fp);
    // printf("gif open: w: %d, h: %d\n", cur_gif->width, cur_gif->height);
    printf("Alarm handler running. cycle count: %d, frame count: %d\n", gif_cycle_count, frame_delay_count);
    // return;
#endif
//  printf("Alarm handler running. cycle count: %d, frame count: %d\n", gif_cycle_count, frame_delay_count);

    if (prev_cycles != gif_cycle_count / 1000)
    {
        prev_cycles = gif_cycle_count / 1000;
        ssd1306_clear_square(&disp, 0, 16, 128, 32);
        sprintf(cycles, "time left: %d\0", prev_cycles);
        ssd1306_draw_string(&disp, 0, 16, 1, cycles);
        ssd1306_show(&disp); 
    }


    if (gif_cycle_count == 0 )
    {
        
        gif_cycle_count = GIF_CYCLE_MS;
#ifdef DEBUG
    printf("starting gif animation. %d\n", fno.fname[0]);
#endif        
        gif_animation_init();
        get_next_frame();
        return;
    }

    gif_cycle_count -= 10; //this is a 10ms timer
    //TODO: show time left on cycle on OLED

    if (frame_delay_count == 0)
    {    
        //forcing the reset here helps to reduce lock-ups        
        if (force_next_gif)
        {
            force_next_gif = false;
            gif_animation_init();
            get_next_frame();
            return;
        }
        else
        {
            get_next_frame();
        }
    }

    frame_delay_count--;

}



bool frame_alarm_cb(repeating_timer_t * fa)
{
    //Instead of calling the code to run the next frame here, a flag
    //is set that is handled in the main loop. This is because of
    //issues running the SPI and I2C from within the callback.
    process_next_frame = true;
    // return false;

    //going to handle button debounce here
    //the button_down flag is used so the jump to the next gif only happens once per press
    if (!gpio_get(BUTTON_IO))
    {
        button_down_count = 0;
        button_down = false;
    }
    else 
    {
        if (!button_down)
        {
            button_down_count++;
        }

        if (button_down_count >= BUTTON_DEBOUNCE && !button_down)
        {
            button_down = true;
            force_next_gif = true;
        }

    }
    


    return true;
}

void test_blank()
{
    for(uint8_t i = 0; i < (8 * 3 * 5); i++)
    {
        //first all hi
        gpio_put_masked(disp_io_mask, disp_io_mask);
        sleep_100ns();
        sleep_100ns();
        //then all low for the rest of the time
        gpio_put_masked(disp_io_mask, 0);
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
    }
}

void test()
{
    uint8_t i = 0;
    uint8_t offset = 0;
    uint8_t groups[6][3] = {
        {0,32,0},
        {32,0,0},
        {0,0,32},
        {0,16,16},
        {16,0,16},
        {16,16,0}                
    };
    uint8_t masks[] = {
        0x01,
        0x02,
        0x04,
        0x08,
        0x10,
        0x20,
        0x40,
        0x80
    };

    uint32_t ios[6] = {DISP_IO_0, DISP_IO_1, DISP_IO_2, DISP_IO_3, DISP_IO_4, DISP_IO_5};

    //Cycle through each group, outputting all bits for one group, then wait 1ms and do the next group
    //--each group will repeat the color 5 times
    //Wait 1000ms when all groups are done, blank the screen for 250ms, and start again, changing the starting index
    //Not worried about the oe in this test (that's next)
    uint8_t b;
    char buff[16];

    ssd1306_clear_square(&disp, 0, 0, 128, 32);
    ssd1306_draw_string(&disp, 0, 0, 1, "testing");
    ssd1306_show(&disp);

    test_blank();
    sleep_ms(100);
    while (1)
    {

        ssd1306_clear_square(&disp, 0, 8, 128, 32);
        sprintf(buff, "offset %d\0", offset);
        ssd1306_draw_string(&disp, 0, 8, 1, buff);
        ssd1306_show(&disp);
        
        //Group output
        for (uint8_t g = 0; g < 6; g++)
        {
            i = (g + offset) % 6;

            //Repeat color loop (once per pixel in current group)
            for (uint8_t p = 0; p < 80; p++)
            {
                //Color byte loop
                for (uint8_t c = 0; c < 3; c++)
                {
                    b = 8;

                    //Output the bytes
                    while(b--)
                    {
                        //initial hi val
                        gpio_put(ios[g], 1);
                        sleep_40ns();
                  
                        //actual value
                        //any non-zero value should be true
                        gpio_put(ios[g], (groups[i][c] & masks[b]));
                        sleep_100ns();
                        sleep_100ns();
                        sleep_100ns();
                        //ending low
                        gpio_put(ios[g], 0);
                        sleep_100ns();
                        sleep_100ns();                        
                        sleep_40ns();
                    }
                }

            }
            // sleep_ms(2);
        }
        sleep_ms(1000);
        offset++;
        if (offset == 6)
        {
            offset = 0;
        }        
    }


}

//For test2, lines will be output onto the actual buffers
//Set all but 1 buffer's oe = 1 (disabled)
//Output 80 cols for 8 lines

//For blank, since all outputs are going low, we can enable all buffs
void test2_blank()
{
    gpio_put_masked(disp_oe_io_mask, 0); //enable all outputs
    for(uint8_t i = 0; i < (8 * 3 * 5); i++)
    {
        //first all hi
        gpio_put_masked(disp_io_mask, disp_io_mask);
        sleep_100ns();
        sleep_100ns();
        //then all low for the rest of the time
        gpio_put_masked(disp_io_mask, 0);
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
        sleep_100ns();
    }
    gpio_put_masked(disp_oe_io_mask, disp_oe_io_mask); //disable all outputs
}

void test2()
{
    uint8_t i = 0;
    uint8_t offset = 0;
    uint8_t groups[6][3] = {
        {0,32,0},
        {32,0,0},
        {0,0,32},
        {0,16,16},
        {16,0,16},
        {16,16,0}                
    };
    uint8_t masks[] = {
        0x01,
        0x02,
        0x04,
        0x08,
        0x10,
        0x20,
        0x40,
        0x80
    };

    uint32_t ios[8] = {DISP_IO_0, DISP_IO_1, DISP_IO_2, DISP_IO_3, DISP_IO_4, DISP_IO_5, DISP_IO_6, DISP_IO_7};
    uint32_t oes[6] = {DISP_OE_IO_A, DISP_OE_IO_B, DISP_OE_IO_C, DISP_OE_IO_D, DISP_OE_IO_E, DISP_OE_IO_F};
    //Cycle through each group, outputting all bits for one group, then wait 1ms and do the next group
    //--each group will repeat the color 80 times (full screen)
    //--will show same color for entire group (8 lines)
    //Wait 1000ms when all groups are done, blank the screen for 250ms, and start again, changing the starting index
    //Not worried about the oe in this test (that's next)
    uint8_t b;
    char buff[16];

    ssd1306_clear_square(&disp, 0, 0, 128, 32);
    ssd1306_draw_string(&disp, 0, 0, 1, "testing");
    ssd1306_show(&disp);

    test2_blank();

    gpio_put_masked(disp_io_mask, 0);
    gpio_put_masked(disp_oe_io_mask, 0);

    sleep_ms(100);
    while (1)
    {

        ssd1306_clear_square(&disp, 0, 8, 128, 32);
        sprintf(buff, "offset %d\0", offset);
        ssd1306_draw_string(&disp, 0, 8, 1, buff);
        ssd1306_show(&disp);
        
        //Group output
        for (uint8_t g = 0; g < 6; g++)
        {
            i = (g + offset) % 6;

            //make sure the outputs are low
            gpio_put_masked(disp_io_mask, 0);
            //then enable just the current group's outputs (low)
            gpio_put_masked(disp_oe_io_mask, ~(1 << oes[g]));

            //Repeat color loop (once per pixel in current group)
            for (uint8_t p = 0; p < 80; p++)
            {
                //Color byte loop
                for (uint8_t c = 0; c < 3; c++)
                {
                    b = 8;

                    //Output the bytes
                    while(b--)
                    {
                        //initial hi val                        
                        gpio_put_masked(disp_io_mask, disp_io_mask);

                        sleep_40ns();
                  
                        //actual value
                        //any non-zero value should be true
                        //This will set all 8 lines to the same value
                        //In the actual code, each line would have its own value from the data
                        gpio_put_masked(disp_io_mask, ((groups[i][c] & masks[b]) ? disp_io_mask : 0));
                        sleep_100ns();
                        sleep_100ns();
                        sleep_100ns();
                        //ending low
                        gpio_put_masked(disp_io_mask, 0);
                        sleep_100ns();
                        sleep_100ns();                        
                        sleep_40ns();
                    }
                }

            }
            // sleep_ms(2);
        }

        //for safety, make all IO's low and enable all outputs
        gpio_put_masked(disp_io_mask, 0);
        gpio_put_masked(disp_oe_io_mask, 0);

        sleep_ms(1000);
        offset++;
        if (offset == 6)
        {
            offset = 0;
        }        
    }


}



int main() {
    set_sys_clock_khz(PICO_CLK_KHZ, true);

    stdio_init_all();
    bit_vals_init();
    display_i2c_init();    
    io_init();
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
        ssd1306_clear_square(&disp, 0, 1, 128, 32);
        ssd1306_draw_string(&disp, 0, 1, 1, "can't mount SD");
        ssd1306_show(&disp);          
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr); 
        return fr;
    }
    
    fr = f_opendir(&dir, "/");

    if (fr != FR_OK)
    {
        ssd1306_clear_square(&disp, 0, 1, 128, 32);
        ssd1306_draw_string(&disp, 0, 1, 1, "can't open root");
        ssd1306_show(&disp);  
        printf("f_opendir error: %s (%d)\n", FRESULT_str(fr), fr); 
        return fr;
    }

    dir.pat = "";

    //TODO: show "displaying gifs" message on OLED
    ssd1306_clear_square(&disp, 0, 0, 128, 32);
    ssd1306_draw_string(&disp, 0, 0, 1, "displaying gifs");
    ssd1306_show(&disp);


    //Start the timer and let it run
    alarm_init();
//    test2();

    while(1)
    {
        if (process_next_frame)
        {
            process_next_frame = false;
            frame_handler();
        }
    }
}
