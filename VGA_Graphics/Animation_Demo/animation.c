/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 -> Pin 21---> VGA Hsync
 *  - GPIO 17 -> Pin 22 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

/*

General todo:
Add conditional checking if spare_time < 0. If that's the case, we're missing timing constraints and a LED should turn on (or off)

*/

// Include the VGA grahics library
#include "vga16_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"

// ROSC setup
#include "hardware/regs/rosc.h"
#include <math.h>

#include "hardware/irq.h"
#include "hardware/pwm.h"  // pwm 
#include "hardware/sync.h"
#include "sample.h" // sound

#define camera_height 1.5
#define camera_to_plate 5

// === DMA VARIABLES============================================================
#define sine_table_size 150
// 256/2
#define AUDIO_PIN 28

// Sine table
int raw_sin[sine_table_size];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short *my_address_pointer = &DAC_data[0];
static float hold_time;
static float start_button_press;
static float release_button_press;

bool already_hit = false;
bool holding_power_button = false;
static float bat_power;
static float pitch_release;
static bool bat_swung = false;
static bool drew_count = false;
static bool outside_batters_box = false;
static int num_balls = 0;
static int num_strikes = 0;
static int num_outs = 0;
static int player_points = 0;
static float start_countdown;
static bool PLAYING = false;
int wav_position = 0; //for sound

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

// SPI configurations
// #define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0
#define POWER_BUTTON 4

// GPIO for timing the ISR
#define ISR_GPIO 2

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

// DMA channels
int data_chan;
int ctrl_chan;

// === LED VARIABLES ==========================================================
// LED stuff
#define LED_PIN 2

// === BUTTON VARIABLES ========================================================
#define BUTTONPIN 14
int buttonState = 0;

// === the fixed point macros ====================================================
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

// === the fixed point macros ==================================================
#define VGAWIDTH 640
#define VGAHEIGHT 480

// === COLLISION DETECTION =====================================================
// TODO: change BATRADIUS to something that makes sense for z condition
#define hitBall(ball_x, ball_y, ball_z, bat_x, bat_y, bat_z) (abs(fix2int15(ball_y - bat_y)) < (BATRADIUS + BALLRADIUS) && abs(fix2int15(ball_x - peg_x)) < (PEGRADIUS + BALLRADIUS) && (fix2int15(ball_z - bat_z)) < (BATRADIUS + BALLRADIUS))

// === TIME VARIABLES ==========================================================
//  uS per frame
#define FRAME_RATE 33000
#define USECS_TO_SECS .00001
int frame_counter = 0;

// === BALL VARIABLES ==========================================================
#define BALLRADIUS 0.0365 // radius of baseball in meters
// const fix15 BALL_RADIUS_FIX15 = int2fix15(BALLRADIUS);

// the color of the boid
char ball_color = WHITE;

// === BAT VARIABLES ==========================================================
#define BATRADIUS 6
#define PEGCOUNT 136

void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));    
    if (wav_position < (WAV_DATA_LENGTH<<3) - 1) { 
        // set pwm level 
        // allow the pwm value to repeat for 8 cycles this is >>3 
        pwm_set_gpio_level(AUDIO_PIN, WAV_DATA[wav_position>>3]);  
        wav_position++;
    } else {
        // reset to start
        wav_position = 0;
    }
}

void init_audio()
{
  spi_init(SPI_PORT, 20000000);
  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
  // gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

  //AUDIO SET UP
  int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

  // Setup PWM interrupt to fire when PWM cycle is complete
  pwm_clear_irq(audio_pin_slice);
  pwm_set_irq_enabled(audio_pin_slice, true);
  // set the handle function above
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
  irq_set_enabled(PWM_IRQ_WRAP, true);

  pwm_config config = pwm_get_default_config();
  /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
    * to set the interrupt rate. 
    * 
    * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
    * 
    * 
    * So clkdiv should be as follows for given sample rate
    *  8.0f for 11 KHz
    *  4.0f for 22 KHz
    *  2.0f for 44 KHz etc
    */
  pwm_config_set_clkdiv(&config, 6.0f); 
  pwm_config_set_wrap(&config, 250); 
  pwm_init(audio_pin_slice, &config, true);

  pwm_set_gpio_level(AUDIO_PIN, 0);

  // Build sine table and DAC data table
  int i;
  for (i = 0; i < (sine_table_size); i++)
  {
    raw_sin[i] = (int)(2047 * sin((float)i * 6.283 / (float)sine_table_size) + 2047); // 12 bit
    DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff);
  }
  // Select DMA channels
  data_chan = dma_claim_unused_channel(true);
  ;
  ctrl_chan = dma_claim_unused_channel(true);
  ;

  // Setup the control channel
  dma_channel_config c = dma_channel_get_default_config(ctrl_chan); // default configs
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);           // 32-bit txfers
  channel_config_set_read_increment(&c, false);                     // no read incrementing
  channel_config_set_write_increment(&c, false);                    // no write incrementing
  channel_config_set_chain_to(&c, data_chan);                       // chain to data channel

  dma_channel_configure(
      ctrl_chan,                        // Channel to be configured
      &c,                               // The configuration we just created
      &dma_hw->ch[data_chan].read_addr, // Write address (data channel read address)
      &my_address_pointer,              // Read address (POINTER TO AN ADDRESS)
      1,                                // Number of transfers
      false                             // Don't start immediately
  );

  // Setup the data channel
  dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
  channel_config_set_read_increment(&c2, true);                      // yes read incrementing
  channel_config_set_write_increment(&c2, false);                    // no write incrementing
  // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
  // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
  dma_timer_set_fraction(0, 0x0017, 0xffff);
  // 0x3b means timer0 (see SDK manual)
  channel_config_set_dreq(&c2, 0x3b); // DREQ paced by timer 0
  // chain to the controller DMA channel

  // channel_config_set_chain_to(&c2, ctrl_chan);                        // Uncomment to chain to control channel

  dma_channel_configure(
      data_chan,                 // Channel to be configured
      &c2,                       // The configuration we just created
      &spi_get_hw(SPI_PORT)->dr, // write address (SPI data register)
      DAC_data,                  // The initial read address
      sine_table_size,           // Number of transfers
      false                      // Don't start immediately.
  );
}

void play_thunk()
{
  // start the control channel
  dma_start_channel_mask(1u << ctrl_chan);
}

// ==========================================
// === Ring Osc random bit RNG
// setup for higher speed oscillator
// ==========================================
void rosc_setup(void)
{
  volatile uint32_t *rosc_div = (uint32_t *)(ROSC_BASE + ROSC_DIV_OFFSET);
  volatile uint32_t *rosc_ctl = (uint32_t *)(ROSC_BASE + ROSC_CTRL_OFFSET);
  volatile uint32_t *rosc_freqA = (uint32_t *)(ROSC_BASE + ROSC_FREQA_OFFSET);
  volatile uint32_t *rosc_freqB = (uint32_t *)(ROSC_BASE + ROSC_FREQB_OFFSET);

  // set divider to one for frequency measurement
  *rosc_div = ROSC_DIV_VALUE_PASS + 1;
  // speed up the ROSC so more cycles between reads
  // (dont use ROSC_CTRL_FREQ_RANGE_VALUE_TOOHIGH)
  // Measured at 241 MHz with theses settings
  *rosc_ctl = ROSC_CTRL_FREQ_RANGE_VALUE_HIGH; // | ROSC_CTRL_ENABLE_VALUE_ENABLE;
  *rosc_freqA = (ROSC_FREQA_PASSWD_VALUE_PASS << 16) | 0xffff;
  *rosc_freqB = (ROSC_FREQB_PASSWD_VALUE_PASS << 16) | 0xffff;
}

// ==========================================================
//  medium quality 32-bit TRNG
// ==========================================================
// Excution time is 6 uSec
// Plenty good enough for game init with the fast RSOC clock
// ==========================================================
// and a small delay in the bit extractor
// AND one pass through rand() function
uint32_t rand_rosc(void)
{
  int k, random;
  int random_bit1;
  volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);

  for (k = 0; k < 32; k++)
  {
    // at least one nop need to be here for timing
    // extractor_count++ ;
    asm("nop");
    asm("nop"); // adds one microsec to execution
    // asm("nop") ; asm("nop") ;
    random_bit1 = 0x00000001 & (*rnd_reg);
    random = (random << 1) | random_bit1;
  }
  // rand_count++ ;
  srand(random);
  random = rand();
  return random;
}

fix15 get_random()
{
  fix15 random_val = (rand() & 0xFFFF) - int2fix15(1); // need to convert this to a small value between -3 and 3
  // fix15 scaled_random = float2fix15(fix2float15(random_val) * 0.1 - 0.05); // should do that
  return random_val;
}

typedef struct
{
  int vga_x;
  int vga_y;
  float vga_z;
} vgaPoint;
// pass location in meters, converts vga screen

#define scale_factor 0.5
vgaPoint convert_to_vga(float x, float y, float z)
{
  int center_x = 320;
  int center_y = 225; // 480*1/3;
  float f = 500;      // Focal length in pixels, found with 150 degree FOV was 400, 160 400 works

  z = z + camera_to_plate; // added

  vgaPoint point = {0, 0, 0.0};

  point.vga_x = center_x + (int)((x * f) / (1 + z));                                    // maybe 1+z
  point.vga_y = center_y - (int)(((y - camera_height) * f) / ((1 + z) * scale_factor)); // added + camera height
  // point.vga_y = point.vga_y - (point.vga_y - 240);

  point.vga_z = f / z;
  return point;
}

// Create a Ball
void spawnBall(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
  // Start in center top  of screen
  *x = VGAHEIGHT / 2;
  *y = VGAWIDTH / 2; // TOP OF SCREEN

  // somehow need to figure out how to do random small x velocity
  // https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_random/index_random.html
  *vx = get_random();
  // Set the og velocity to zero
  *vy = 0;
}

void updatePower(float power)
{
  char str[20];
  sprintf(str, "%f", power);
  char *ptr = str;
  setCursor(70, VGAHEIGHT - 40);
  setTextColor2(WHITE, DARK_GREEN);
  writeString(ptr);
  int power_length = 150 * power;
  fillRect(21, VGAHEIGHT - 29, 148, 8, BLACK);
  fillRect(21, VGAHEIGHT - 29, power_length - 1, 8, RED);
}

void updateTime(int time)
{
  char str[20];
  float num = (float)(time / 10000);
  sprintf(str, "%d", num);
  char *ptr = str;
  setCursor(VGAWIDTH / 2 + 50, 70);
  setTextColor2(WHITE, BLACK);
  writeString(ptr);
}

#define max_height_batters_box 1.2 // h
#define min_height_batters_box 0.5
#define min_width_batters_box -0.2159
#define max_width_batters_box 0.2159
#define bat_radius 0.1 // m //here's what I kinda want to do. Always draw bat as a 1 pixel wide thing. But internally compute it as 0.1 m wide so its easier to hit

static float bat_position;

float get_bat_position()
{
  int result = adc_read() >> 4;

  bat_position = min_height_batters_box + (max_height_batters_box - min_height_batters_box) * (result / (float)(4096 >> 4));

  return bat_position;
}

void drawBat()
{
  vgaPoint point1 = convert_to_vga(min_width_batters_box, min_height_batters_box, 0);
  vgaPoint point2 = convert_to_vga(min_width_batters_box, max_height_batters_box, 0);
  vgaPoint point3 = convert_to_vga(max_width_batters_box, min_height_batters_box, 0);
  vgaPoint point4 = convert_to_vga(max_width_batters_box, max_height_batters_box, 0);
  drawLine(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y, RED);
  drawLine(point4.vga_x, point4.vga_y, point2.vga_x, point2.vga_y, RED);
  drawLine(point4.vga_x, point4.vga_y, point3.vga_x, point3.vga_y, RED);
  drawLine(point1.vga_x, point1.vga_y, point3.vga_x, point3.vga_y, RED);

  vgaPoint point = convert_to_vga(min_width_batters_box - 0.05, bat_position, 0); // bat position is previous bat position
  float curr_y = get_bat_position();
  point1 = convert_to_vga(max_width_batters_box + 0.05, curr_y, 0);

  vgaPoint batlower = convert_to_vga(0, curr_y - bat_radius / 2, 0);
  vgaPoint batupper = convert_to_vga(0, curr_y + bat_radius / 2, 0);
  drawRect((short)point.vga_x, (short)point.vga_y, (short)(point1.vga_x - point.vga_x), (short)(1), DARK_GREEN);
  drawRect((short)point.vga_x, (short)point1.vga_y, (short)(point1.vga_x - point.vga_x), (short)(1), YELLOW);

  // printf("batlower world: %f\n", curr_y - bat_radius/2);
  // printf("batupper world: %f\n", curr_y + bat_radius/2);
  // printf("batlower vga_y: %d\n", batlower.vga_y);
  // printf("batupper vga_y: %d\n", batupper.vga_y);
}

void drawBall(float x, float y, float z)
{
  vgaPoint point = convert_to_vga(x, y, z);
  // updateXpx(int2fix15(point.vga_x));
  // updateYpx(int2fix15(point.vga_y));
  // updateZpx(float2fix15(point.vga_z));
  fillCircle((short)point.vga_x, (short)point.vga_y, (short)point.vga_z * BALLRADIUS, ball_color);
}

void sweepBall()
{
  for (int x = -100; x < 100; x += 10)
  {
    for (int y = -100; y < 100; y += 10)
    {
      for (int z = 1; z < 100; z++)
      {
        // updateX(x);
        // updateY(y);
        // updateZ(z);
        drawBall(x, y, z);
        // vgaPoint point = convert_to_vga(x, y, z);
        // delay(5);
        // fillCircle(point.vga_x, point.vga_y, point.vga_z * BALLRADIUS, BLACK);
      }
    }
  }
}

void drawField2()
{  
  fillRect(0, 0, VGAWIDTH, VGAHEIGHT, BLACK);
  // setTextSize(1);
  // setCursor(VGAWIDTH / 2 - 20, 20);
  // setTextColor(WHITE);
  // writeString("Batting Simulator!!");
  // fill field
  vgaPoint point1 = convert_to_vga(26.43, 0, 26.43);  // first
  vgaPoint point2 = convert_to_vga(0, 0, 38.79);      // second
  vgaPoint point3 = convert_to_vga(-26.43, 0, 26.43); // third
  vgaPoint point4 = convert_to_vga(0, 0, 2.05);       // home

  fillQuad(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y,
           point3.vga_x, point3.vga_y, point4.vga_x, point4.vga_y,
           DARK_GREEN);

  point1 = convert_to_vga(28.43, 0, 27.43); // first
  point2 = convert_to_vga(28.43, 0, 0);     // second
  point3 = convert_to_vga(2.75, 0, 0);      // third
  point4 = convert_to_vga(2.75, 0, 2.05);   // home

  fillQuad(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y,
           point3.vga_x, point3.vga_y, point4.vga_x, point4.vga_y,
           DARK_GREEN);

  point1 = convert_to_vga(-28.43, 0, 27.43); // first
  point2 = convert_to_vga(-28.43, 0, 0);     // second
  point3 = convert_to_vga(-2.75, 0, 0);      // third
  point4 = convert_to_vga(-2.75, 0, 2.05);   // home

  fillQuad(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y,
           point3.vga_x, point3.vga_y, point4.vga_x, point4.vga_y,
           DARK_GREEN);

  // fill3DCircle(0, 0, 5, 2.5, ORANGE);

  // field
  point1 = convert_to_vga(27.43, 0, 27.43);  // first
  point2 = convert_to_vga(-27.43, 0, 27.43); // third
  point3 = convert_to_vga(0, 0, 38.79);      // second
  point4 = convert_to_vga(1.25, 0, 1.05);
  vgaPoint point5 = convert_to_vga(-1.25, 0, 1.05);

  drawLine(point1.vga_x, point1.vga_y, point4.vga_x, point4.vga_y, WHITE); // home to first
  drawLine(point2.vga_x, point2.vga_y, point5.vga_x, point5.vga_y, WHITE); // home to third
  drawLine(point2.vga_x, point2.vga_y, point3.vga_x, point3.vga_y, WHITE); // second to third
  drawLine(point1.vga_x, point1.vga_y, point3.vga_x, point3.vga_y, WHITE); // second to first

  // pitchers box
  point1 = convert_to_vga(-0.3048, 0, 19.395);
  point2 = convert_to_vga(0.3048, 0, 19.395);
  point3 = convert_to_vga(-0.3048, 0, 19.395);
  point4 = convert_to_vga(0.3048, 0, 19.395);
  point5 = convert_to_vga(0, 0, 19.395);
  drawLine(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y, WHITE);
  drawLine(point3.vga_x, point3.vga_y, point4.vga_x, point4.vga_y, WHITE);
  drawLine(point1.vga_x, point1.vga_y, point3.vga_x, point3.vga_y, WHITE);
  drawLine(point2.vga_x, point2.vga_y, point4.vga_x, point4.vga_y, WHITE);
  // fill3DCircle(point5.vga_x, point5.vga_y, point5.vga_z, 4, BLACK);

  // batter's box
  point1 = convert_to_vga(min_width_batters_box, min_height_batters_box, 0);
  point2 = convert_to_vga(min_width_batters_box, max_height_batters_box, 0);
  point3 = convert_to_vga(max_width_batters_box, min_height_batters_box, 0);
  point4 = convert_to_vga(max_width_batters_box, max_height_batters_box, 0);
  drawLine(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y, RED);
  drawLine(point4.vga_x, point4.vga_y, point2.vga_x, point2.vga_y, RED);
  drawLine(point4.vga_x, point4.vga_y, point3.vga_x, point3.vga_y, RED);
  drawLine(point1.vga_x, point1.vga_y, point3.vga_x, point3.vga_y, RED);

  // Where the batter stands (around homeplate)
  point1 = convert_to_vga(-0.75, 0.0, 0.0);
  point2 = convert_to_vga(-1.75, 0.0, 0.0);
  point3 = convert_to_vga(-0.75, 0.0, 1.05);
  point4 = convert_to_vga(-1.75, 0.0, 1.05);

  // drawLine(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y, WHITE);
  drawLine(point1.vga_x, point1.vga_y, point3.vga_x, point3.vga_y, WHITE);
  drawLine(point4.vga_x, point4.vga_y, point3.vga_x, point3.vga_y, WHITE);
  drawLine(point4.vga_x, point4.vga_y, point2.vga_x, point2.vga_y, WHITE);

  point1 = convert_to_vga(0.75, 0.0, 0.0);
  point2 = convert_to_vga(1.75, 0.0, 0.0);
  point3 = convert_to_vga(0.75, 0.0, 1.05);
  point4 = convert_to_vga(1.75, 0.0, 1.05);

  // drawLine(point1.vga_x, point1.vga_y, point2.vga_x, point2.vga_y, WHITE);
  drawLine(point1.vga_x, point1.vga_y, point3.vga_x, point3.vga_y, WHITE);
  drawLine(point4.vga_x, point4.vga_y, point3.vga_x, point3.vga_y, WHITE);
  drawLine(point4.vga_x, point4.vga_y, point2.vga_x, point2.vga_y, WHITE);

  // draw 2nd base
  point1 = convert_to_vga(0.0, 0.0, 36.8);
  point2 = convert_to_vga(0, 0.0, 37.18);
  point3 = convert_to_vga(-0.1905, 0.0, 36.99);
  point4 = convert_to_vga(0.1905, 0.0, 36.99);
  drawLine(point1.vga_x, point1.vga_y, point3.vga_x, point3.vga_y, WHITE);
  drawLine(point1.vga_x, point1.vga_y, point4.vga_x, point4.vga_y, WHITE);
  drawLine(point2.vga_x, point2.vga_y, point3.vga_x, point3.vga_y, WHITE);
  drawLine(point2.vga_x, point2.vga_y, point4.vga_x, point4.vga_y, WHITE);

  // Power bar
  drawRect(20, VGAHEIGHT - 30, 150, 10, WHITE);
  setTextSize(1);
  setTextColor(WHITE);
  setCursor(20, VGAHEIGHT - 40);
  writeString("Power: ");
  // fill3DCircle(0, 0, 5, 2.5, ORANGE);

  // Score Board
  fillRect(VGAWIDTH - 170, VGAHEIGHT - 50, 150, 30, WHITE);
  setTextSize(1);
  setTextColor(BLACK);
  setCursor(VGAWIDTH - 165, VGAHEIGHT - 45);
  writeString("Balls");
  setCursor(VGAWIDTH - 115, VGAHEIGHT - 45);
  writeString("Strikes");
  setCursor(VGAWIDTH - 60, VGAHEIGHT - 45);
  writeString("Runs");

  // Update Scoreboard
  setTextSize(1);
  char str[20];
  // draw strikes
  fillRect(VGAWIDTH - 115, VGAHEIGHT - 35, 10, 10, WHITE);
  sprintf(str, "%d", num_strikes);
  char *ptr = str;
  setCursor(VGAWIDTH - 115, VGAHEIGHT - 35);
  setTextColor2(BLACK, WHITE);
  writeString(ptr);
  // draw player points
  fillRect(VGAWIDTH - 65, VGAHEIGHT - 35, 10, 10, WHITE);
  sprintf(str, "%d", player_points);
  setCursor(VGAWIDTH - 65, VGAHEIGHT - 35);
  setTextColor2(BLACK, WHITE);
  writeString(ptr);
  // draw balls
  fillRect(VGAWIDTH - 165, VGAHEIGHT - 35, 10, 10, WHITE); // remove last num balls
  sprintf(str, "%d", num_balls);
  setCursor(VGAWIDTH - 165, VGAHEIGHT - 35);
  setTextColor2(BLACK, WHITE);
  writeString(ptr);

  setTextSize(1);
  setCursor(VGAWIDTH - 150, 20);
  setTextColor(WHITE);
  writeString("Ball Distance: ");
  // fill field
  setCursor(70, 20);
  setTextColor(WHITE);
  writeString("OUTS: ");
  sprintf(str, "%d", num_outs);
  setCursor(100, 20);
  setTextColor2(WHITE, BLACK);
  writeString(ptr);
}

void drawField()
{
  int middle_screen_x = VGAWIDTH / 2;

  // DRAW BATTER BOXES
  drawLine(middle_screen_x + VGAWIDTH * 2.5 / 38, VGAHEIGHT / 21 * (21 - 4), middle_screen_x + VGAWIDTH * 4 / 38, VGAHEIGHT, WHITE);
  drawLine(middle_screen_x - VGAWIDTH * 2.5 / 38, VGAHEIGHT / 21 * (21 - 4), middle_screen_x - VGAWIDTH * 4 / 38, VGAHEIGHT, WHITE);
  drawLine(VGAWIDTH * 9 / 38, VGAHEIGHT / 21 * (21 - 4), middle_screen_x - VGAWIDTH * 2.5 / 38, VGAHEIGHT / 21 * (21 - 4), WHITE);
  drawLine(VGAWIDTH - VGAWIDTH * 9 / 38, VGAHEIGHT / 21 * (21 - 4), middle_screen_x + VGAWIDTH * 2.5 / 38, VGAHEIGHT / 21 * (21 - 4), WHITE);
  drawLine(VGAWIDTH * 9 / 38, VGAHEIGHT / 21 * (21 - 4), VGAWIDTH * 2.5 / 38, VGAHEIGHT, WHITE);
  drawLine(VGAWIDTH - VGAWIDTH * 9 / 38, VGAHEIGHT / 21 * (21 - 4), VGAWIDTH - VGAWIDTH * 2.5 / 38, VGAHEIGHT, WHITE);

  // DRAW LINES THAT GO TO 1st and 3rd
  drawLine(middle_screen_x + VGAWIDTH * 7 / 38, VGAHEIGHT / 21 * (21 - 4), VGAWIDTH, VGAHEIGHT / 21 * (21 - 7.5), WHITE);
  drawLine(middle_screen_x - VGAWIDTH * 7 / 38, VGAHEIGHT / 21 * (21 - 4), 0, VGAHEIGHT / 21 * (21 - 7.5), WHITE);

  // HOME PLATE
  drawLine(middle_screen_x + VGAWIDTH * 2 / 38, VGAHEIGHT / 21 * (21 - 2.75), middle_screen_x - VGAWIDTH * 2 / 38, VGAHEIGHT / 21 * (21 - 2.75), WHITE); // top of plate
  drawLine(middle_screen_x + VGAWIDTH * 2 / 38, VGAHEIGHT / 21 * (21 - 2.75), middle_screen_x + VGAWIDTH * 2.2 / 38, VGAHEIGHT / 21 * (21 - 2), WHITE);  // sides of plate
  drawLine(middle_screen_x - VGAWIDTH * 2 / 38, VGAHEIGHT / 21 * (21 - 2.75), middle_screen_x - VGAWIDTH * 2.2 / 38, VGAHEIGHT / 21 * (21 - 2), WHITE);
  drawLine(middle_screen_x + VGAWIDTH * 2.2 / 38, VGAHEIGHT / 21 * (21 - 2), middle_screen_x, VGAHEIGHT / 21 * (21 - 1.25), WHITE); // top of pentagon
  drawLine(middle_screen_x - VGAWIDTH * 2.2 / 38, VGAHEIGHT / 21 * (21 - 2), middle_screen_x, VGAHEIGHT / 21 * (21 - 1.25), WHITE); // top of pentagon

  // DRAW HORIZON OF FIELD:
  drawLine(0, VGAHEIGHT / 21 * (11.25), middle_screen_x, VGAHEIGHT / 21 * (11), BLUE);
  drawLine(middle_screen_x, VGAHEIGHT / 21 * (11), VGAWIDTH, VGAHEIGHT / 21 * (11.25), BLUE);

  // DRAW DIRT BETWEEN 1st and 2nd and dirt between 2nd and third
  // Bottom Lines
  drawLine(0, VGAHEIGHT / 21 * (12), middle_screen_x, VGAHEIGHT / 21 * (11.5), DARK_GREEN);
  drawLine(middle_screen_x, VGAHEIGHT / 21 * (11.5), VGAWIDTH, VGAHEIGHT / 21 * (12), DARK_GREEN);
  // top Lines
  drawLine(0, VGAHEIGHT / 21 * (11.5), middle_screen_x, VGAHEIGHT / 21 * (11.6), DARK_GREEN);
  drawLine(middle_screen_x, VGAHEIGHT / 21 * (11.6), VGAWIDTH, VGAHEIGHT / 21 * (11.5), DARK_GREEN);

  // DRAW SIDES OF FRONT OF FIELD
  drawLine(middle_screen_x + VGAWIDTH / 38 * (9), VGAHEIGHT / 21 * (14.5), VGAWIDTH, VGAHEIGHT / 21 * (12.85), DARK_GREEN);
  drawLine(middle_screen_x - VGAWIDTH / 38 * (9), VGAHEIGHT / 21 * (14.5), 0, VGAHEIGHT / 21 * (12.85), DARK_GREEN);
  // need to draw the parabolic front part of the baseball field (For now just a straight line)
  drawLine(middle_screen_x - VGAWIDTH / 38 * (9), VGAHEIGHT / 21 * (14.5), middle_screen_x + VGAWIDTH / 38 * (9), VGAHEIGHT / 21 * (14.5), DARK_GREEN);

  // DRAW PITCHERS MOUND
  drawLine(middle_screen_x + VGAWIDTH / 38 * (3.5), VGAHEIGHT / 21 * (12.1), middle_screen_x + VGAWIDTH / 38 * (1.75), VGAHEIGHT / 21 * (12.5), ORANGE);  // side of bottom
  drawLine(middle_screen_x - VGAWIDTH / 38 * (3.5), VGAHEIGHT / 21 * (12.1), middle_screen_x - VGAWIDTH / 38 * (1.75), VGAHEIGHT / 21 * (12.5), ORANGE);  // side of bottom
  drawLine(middle_screen_x + VGAWIDTH / 38 * (1.75), VGAHEIGHT / 21 * (12.5), middle_screen_x - VGAWIDTH / 38 * (1.75), VGAHEIGHT / 21 * (12.5), ORANGE); // bottom
  drawLine(middle_screen_x + VGAWIDTH / 38 * (3.5), VGAHEIGHT / 21 * (12.1), middle_screen_x + VGAWIDTH / 38 * (1), VGAHEIGHT / 21 * (11.9), ORANGE);     // side of top
  drawLine(middle_screen_x - VGAWIDTH / 38 * (3.5), VGAHEIGHT / 21 * (12.1), middle_screen_x - VGAWIDTH / 38 * (1), VGAHEIGHT / 21 * (11.9), ORANGE);     // side of top
  drawLine(middle_screen_x + VGAWIDTH / 38 * (1), VGAHEIGHT / 21 * (11.9), middle_screen_x - VGAWIDTH / 38 * (1), VGAHEIGHT / 21 * (11.9), ORANGE);       // top

  // draw horizon of fence
  drawLine(0, VGAHEIGHT / 21 * (10), VGAWIDTH, VGAHEIGHT / 21 * (10), WHITE);

  // draw rectangular score box (Not sure if this will look good, tried to mimic the pic)
  fillRect(VGAWIDTH - VGAWIDTH / 38 * (11), VGAHEIGHT - VGAHEIGHT / 21 * 4, VGAWIDTH / 38 * (10), VGAHEIGHT / 21 * 3, DARK_BLUE);
  fillRect(VGAWIDTH - VGAWIDTH / 38 * (11), VGAHEIGHT - VGAHEIGHT / 21 * 4, VGAWIDTH / 38 * (10), VGAHEIGHT / 21, WHITE);
  fillRect(VGAWIDTH - VGAWIDTH / 38 * (11), VGAHEIGHT - VGAHEIGHT / 21 * 3, VGAWIDTH / 38 * (10), VGAHEIGHT / 21 * (0.2), RED);
  // drawText() need to initialize team names in here
}

void update_velocity(fix15 *vx, fix15 *vy, fix15 *d_vx, fix15 *d_vy)
{
  *vx = *vx + *d_vx;
  *vy = *vy + *d_vy;
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  // stores user input
  static int user_input;
  // wait for 0.1 sec
  PT_YIELD_usec(1000000);
  // announce the threader version
  sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
  // non-blocking write
  serial_write;
  while (1)
  {
    // print prompt
    sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
    // non-blocking write
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;
    // convert input string to number
    sscanf(pt_serial_in_buffer, "%d", &user_input);
    // update boid color
    if ((user_input > 0) && (user_input < 16))
    {
      ball_color = (char)user_input;
    }
  } // END WHILE(1)
  PT_END(pt);
} // timer thread

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// PITCHING
#define mound_to_plate 19.395

#define min_horizontal_pitch_amplitude -5
#define max_pitch_amplitude 5 // a

#define max_pitch_starting_height 1.98 // g
#define min_pitch_starting_height 1.83

#define max_amplitude_position 0.5 // n
#define min_amplitude_position 0.2

#define min_ball_v 13.4112 // 30mph
#define max_ball_v 17.8816 // 40mph

static float y_a; // amplitude
static float x_a;

static float y_g; // starting height
static float x_g;

static float y_h; // y coordinate at batters box
static float x_h;

static float y_n; // position of max_amplitude
static float x_n;

static float ball_vx;
static float ball_vy;
static float ball_vz; // ball_velocity

static float ball_x;
static float ball_y;
static float ball_z;

void init_pitch()
{
  // init_y
  y_a = ((float)rand() / RAND_MAX) * (float)max_pitch_amplitude;
  y_g = ((float)rand() / RAND_MAX) * (float)(max_pitch_starting_height - min_pitch_starting_height) + min_pitch_starting_height;
  // y_h = (float)(max_height_batters_box - min_height_batters_box); //
  y_h = ((float)rand() / RAND_MAX) * (float)(max_height_batters_box - min_height_batters_box) + min_height_batters_box;
  y_n = ((float)rand() / RAND_MAX) * (float)(max_amplitude_position - min_amplitude_position) + min_amplitude_position;

  x_a = (((float)rand() / (RAND_MAX)) * 2 - 1) * (float)max_pitch_amplitude;
  x_g = 0;
  x_h = (((float)rand() / (RAND_MAX)) * 2 - 1) * (float)(max_width_batters_box); // right now all pitches are within strike zone
  x_n = ((float)rand() / RAND_MAX) * (float)(max_amplitude_position - min_amplitude_position) + min_amplitude_position;

  ball_vz = ((float)rand() / RAND_MAX) * (float)(max_ball_v - min_ball_v) + min_ball_v;

  ball_x = x_g;
  ball_y = y_g;
  ball_z = mound_to_plate;
}

float model_pitch(float a, float g, float d, float h, float n, float time)
{
  float x = time;
  float xd_over_d_pow_n = powf(x / d, n);
  float term1 = -a / (d * d) * (d * xd_over_d_pow_n) * ((d * xd_over_d_pow_n) - d);
  float term2 = (h * x) / d;
  float term3 = (g * d - g * x) / d;
  return term1 + term2 + term3;
}

// curr time / time to plate * mound to plate
void update_pitch(float time)
{

  ball_z = ball_vz * time;
  ball_x = model_pitch(x_a, x_g, mound_to_plate, x_h, x_n, ball_z);
  ball_y = model_pitch(y_a, y_g, mound_to_plate, y_h, y_n, ball_z);
  ball_z = mound_to_plate - ball_vz * time;
  // updatePower(hold_time/time);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// PITCHING

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Batting

#define bat_swing_to_horizontal 0.0
#define bat_trajectory_angle 15 // degrees
#define bat_miss_threshold 0.08
#define bat_perfect_hit_threshold 0.01 // if delta t ball bat is within this threshold than perfect hit
#define maximum_ball_angle 20          // 90
#define bat_mass 1.02                  // kg

#define bat_speed 32.1869   // m/s
#define ball_mass 0.145     // kg
#define ball_radius 0.03683 // m

float radians_to_degrees(float radians)
{
  return radians * (180.0 / M_PI);
}

float get_ball_angle_from_bat_and_ball_position(float bat_position, float ball_position)
{
  // This function should recieve the y coordinate of the ball and bat when they collide
  float delta_y = ball_position - bat_position;

  float exit_angle = radians_to_degrees(asinf(delta_y / (bat_radius + ball_radius))); // need inv sine function

  return exit_angle;
}

float get_delta_t_ball()
{

  double target_distance = mound_to_plate;
  double t_lower = 0.0;
  double t_upper = 10.0; // Initial guess; increase if necessary
  double tolerance = 1e-6;
  double z0 = mound_to_plate;

  // Expand t_upper until we exceed the desired distance
  while (fabs(ball_vz - z0) < target_distance) // fabs is floating point absolute value
  {
    t_upper *= 2.0;
    if (t_upper > 1e6)
    {
      printf("Target distance not reached within reasonable time.\n");
      return -1.0;
    }
  }

  // Bisection method
  while ((t_upper - t_lower) > tolerance)
  {
    double t_mid = 0.5 * (t_lower + t_upper);
    double dist = fabs(ball_vz - z0);

    if (dist < target_distance)
      t_lower = t_mid;
    else
      t_upper = t_mid;
  }

  return 0.5 * (t_lower + t_upper);
}

#include <math.h>

#define PI 3.14159265f

// Convert degrees to radians
float deg_to_rad(float degrees)
{
  return degrees * (PI / 180.0f);
}

void spherical_to_cartesian_custom(float r, float theta_deg, float phi_deg, float *x, float *y, float *z)
{
  float theta = deg_to_rad(theta_deg);
  float phi = deg_to_rad(phi_deg);

  *x = r * sinf(phi) * sinf(theta); // Theta affects X
  *y = r * sinf(phi) * cosf(theta); // Theta affects Y
  *z = r * cosf(phi);               // Phi starts from +Z
}

#define SCALE_FACTOR 3
void get_collision(float bat_power, float from_pitch_to_swing)
{

  // bat_power is a scale factor dependent on timing
  // This function should be called the second the player initiates their swing

  // float delta_t_ball = (ballz - camera_to_plate) / ballvz; // ballvz is modeled by a function, not a constant. Need to use that function to get delta_t_ball

  // float delta_t_ball = get_delta_t_ball();           // gives time from pitcher release to plate
  printf("\nFrom_pitch_to_swing: %f\n", from_pitch_to_swing);

  float delta_t_ball = mound_to_plate / ball_vz;            // above line commmented out. Ball velocity now constant
  delta_t_ball = delta_t_ball - from_pitch_to_swing + 0.03; // subtract time from pitch to swing
  // float delta_t_bat = bat_swing_to_horizontal;       // assumed constant

  printf("delta_t_ball: %f\n", delta_t_ball);
  // float delta_bat_to_ball = delta_t_bat - delta_t_ball; // difference in time it takes for bat to reach horizontal and ball to reach horizontal

  // printf("delta_bat_to_ball: %f\n", delta_bat_to_ball);

  // debugging
  // delta_bat_to_ball = 0.0;
  bool miss = delta_t_ball < -bat_miss_threshold || delta_t_ball > bat_miss_threshold;

  float ball_y_position = model_pitch(y_a, y_g, mound_to_plate, y_h, y_n, mound_to_plate);

  // bat_position = ball_y_position; //debugging for y
  bool position_miss = !(ball_y_position + ball_radius > bat_position - bat_radius && ball_y_position - ball_radius < bat_position + bat_radius);

  if (!miss && !position_miss)
  { //&& function to determine overlap of bat and ball

    // get angles
    float ball_angle_up = 30 + (get_ball_angle_from_bat_and_ball_position(bat_position, ball_y_position)); // need to deterimen y positions when ball and bat overlap
    float ball_angle_left_right = (delta_t_ball / (bat_miss_threshold)) * maximum_ball_angle;              // difference in time / (some scale factor) * maximum possible angle
    // if (delta_bat_to_ball < 0){
    //   ball_angle_left_right = - ball_angle_left_right;
    // }

    // get velocities
    // float v_ball_f = (bat_mass * vbat * 0.05 + ball_mass * vball_i) / ball_mass;

    float v_ball_f = (bat_mass * bat_speed * 0.05 * bat_power * SCALE_FACTOR + ball_mass * ball_vz) / ball_mass;
    if (ball_angle_up > 45)
    {
      v_ball_f = v_ball_f * (ball_angle_up / 90.0); // TODO: MAKE BETTER IF NECESSARY
    }
    // printf("ball_y_position: %f\n", ball_y_position);
    // printf("bat_position: %f\n", bat_position);
    // printf("exit_angle: %f\n", ball_angle_up);
    spherical_to_cartesian_custom(v_ball_f, ball_angle_left_right, ball_angle_up, &ball_vx, &ball_vy, &ball_vz);

    // update run thing
    player_points += 1;
    setTextSize(1);
    char str[20];
    num_balls = 0;
    num_strikes = 0;
    fillRect(VGAWIDTH - 65, VGAHEIGHT - 35, 10, 10, WHITE);
    // update num points
    sprintf(str, "%d", player_points);
    char *ptr = str;
    setCursor(VGAWIDTH - 65, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
  else // update strike thing
  {
    ball_vx = 0;
    ball_vy = 0;
    ball_vz = 0;
    num_strikes += 1;
    setTextSize(1);
    char str[20];
    fillRect(VGAWIDTH - 115, VGAHEIGHT - 35, 10, 10, WHITE);
    sprintf(str, "%d", num_strikes);
    char *ptr = str;
    setCursor(VGAWIDTH - 115, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
}

void model_random_flight_path()
{
  float ball_angle_up = (((float)rand() / (RAND_MAX))) * 30;
  float ball_angle_left_right = (((float)rand() / (RAND_MAX)) * 2 - 1) * maximum_ball_angle;       // difference in time / (some scale factor) * maximum possible angle
  float v_ball_f = (bat_mass * bat_speed * 0.05 * SCALE_FACTOR + ball_mass * ball_vz / ball_mass); // ball_vz_function(delta_t_ball + from_pitch_to_swing)) / ball_mass;

  spherical_to_cartesian_custom(v_ball_f, ball_angle_left_right, ball_angle_up, &ball_vx, &ball_vy, &ball_vz);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Batting

float gravity(float t, float y0, float v0y)
{
  return y0 + v0y * t - 0.5 * 9.81 * t * t;
}

static float hit_y_position;
static int iteration = 0;
void model_flight(float time)
{
  ball_x = ball_x + ball_vx * time;
  ball_y = gravity(time * iteration, hit_y_position, ball_vy);
  ball_z = ball_z + ball_vz * time;
}

float dt = 1 / 30.3;

static bool ball_has_reached_plate = false; // TODO: terrible naming conventions. Should be called pitch reached plate or something
static bool ball_flight = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Scoreboard

void updateBallsorStrike()
{
  setTextSize(1);
  outside_batters_box = (ball_x - ball_radius < min_width_batters_box) || (ball_x + ball_radius > max_width_batters_box) || (ball_y + ball_radius > max_height_batters_box) || (ball_y - ball_radius < min_height_batters_box);
  char str[20];
  if (outside_batters_box && num_balls == 3 && !bat_swung)
  { // BALL + GIVE A POINT CAUSE ALLOWED TO WALK
    player_points += 1;
    num_balls = 0;
    num_strikes = 0;
    fillRect(VGAWIDTH - 65, VGAHEIGHT - 35, 10, 10, WHITE);  // remove last num points
    fillRect(VGAWIDTH - 165, VGAHEIGHT - 35, 10, 10, WHITE); // remove last num balls

    // update num balls
    sprintf(str, "%d", num_balls);
    char *ptr = str;
    setCursor(VGAWIDTH - 165, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);

    // update num points
    sprintf(str, "%d", player_points);
    ptr = str;
    setCursor(VGAWIDTH - 65, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
  else if (outside_batters_box && !bat_swung)
  { // BALL
    // update num balls
    num_balls += 1;
    fillRect(VGAWIDTH - 165, VGAHEIGHT - 35, 10, 10, WHITE);
    sprintf(str, "%d", num_balls);
    char *ptr = str;
    setCursor(VGAWIDTH - 165, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
  else if ((!outside_batters_box) || (outside_batters_box && bat_swung))
  { // STRIKE
    // update number of strikes
    num_strikes += 1;
    fillRect(VGAWIDTH - 115, VGAHEIGHT - 35, 10, 10, WHITE);
    sprintf(str, "%d", num_strikes);
    char *ptr = str;
    setCursor(VGAWIDTH - 115, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
}

void updateScore()
{
  setTextSize(1);
  bool didnt_swing = (hold_time == 0.0); // corresponds to having no power
  outside_batters_box = (ball_x - ball_radius < min_width_batters_box) || (ball_x + ball_radius > max_width_batters_box) || (ball_y + ball_radius > max_height_batters_box) || (ball_y - ball_radius < min_height_batters_box);
  char str[20];
  if (outside_batters_box && num_balls == 3 && didnt_swing)
  { // BALL + GIVE A POINT CAUSE ALLOWED TO WALK
    player_points += 1;
    num_balls = 0;
    fillRect(VGAWIDTH - 65, VGAHEIGHT - 35, 10, 10, WHITE);  // remove last num points
    fillRect(VGAWIDTH - 165, VGAHEIGHT - 35, 10, 10, WHITE); // remove last num balls

    // update num balls
    sprintf(str, "%d", num_balls);
    char *ptr = str;
    setCursor(VGAWIDTH - 165, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);

    // update num points
    sprintf(str, "%d", player_points);
    ptr = str;
    setCursor(VGAWIDTH - 65, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
  else if (outside_batters_box && didnt_swing)
  { // BALL
    // update num balls
    num_balls += 1;
    fillRect(VGAWIDTH - 165, VGAHEIGHT - 35, 10, 10, WHITE);
    sprintf(str, "%d", num_balls);
    char *ptr = str;
    setCursor(VGAWIDTH - 165, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
  else if ((!outside_batters_box) || (outside_batters_box && !didnt_swing))
  { // STRIKE
    // update number of strikes
    num_strikes += 1;
    fillRect(VGAWIDTH - 115, VGAHEIGHT - 35, 10, 10, WHITE);
    sprintf(str, "%d", num_strikes);
    char *ptr = str;
    setCursor(VGAWIDTH - 115, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }
  else
  { // batter hit the ball
    player_points += 1;
    fillRect(VGAWIDTH - 65, VGAHEIGHT - 35, 10, 10, WHITE);
    // update num points
    sprintf(str, "%d", player_points);
    char *ptr = str;
    setCursor(VGAWIDTH - 65, VGAHEIGHT - 35);
    setTextColor2(BLACK, WHITE);
    writeString(ptr);
  }

  // FIGURE OUT IF THE GAME IS OVER:
  if (player_points == 3)
  {
    setTextSize(6);
    sprintf(str, "YOU WIN! GAME OVER");
    char *ptr = str;
    setCursor(70, VGAHEIGHT / 2 - 40);
    setTextColor2(RED, WHITE);
    writeStringBold(ptr);
  }
  if (num_outs == 3)
  {
    setTextSize(5);
    sprintf(str, "GAME OVER, YOU LOSE");
    char *ptr = str;
    setCursor(0, VGAHEIGHT / 2 - 40); // might need to adjust
    setTextColor2(RED, WHITE);
    writeStringBold(ptr);
  }
  if (num_strikes == 3)
  {
    setTextSize(5);
    sprintf(str, "STRIKE OUT");
    char *ptr = str;
    setCursor(100, VGAHEIGHT / 2 - 40); // might need to adjust
    setTextColor2(RED, BLACK);
    writeStringBold(ptr);
    num_outs+=1;

    setTextSize(1);
    char str[20];
    num_balls = 0;
    num_strikes = 0;
    fillRect(150, 20, 10, 10, BLACK);
    // update num points
    sprintf(str, "%d", num_outs);
    setCursor(100, 20);
    setTextColor2(WHITE, BLACK);
    writeString(ptr);
  }
}

void drawOne()
{
  int middle_screen_x = VGAWIDTH / 2;
  drawLine(middle_screen_x + 0.5 * VGAWIDTH / 38, VGAHEIGHT - 12.25 * VGAHEIGHT / 21, middle_screen_x + 0.5 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, RED); // right side of stem
  drawLine(middle_screen_x - 0.5 * VGAWIDTH / 38, VGAHEIGHT - 14 * VGAHEIGHT / 21, middle_screen_x - 0.5 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, RED);    // left side of stem

  drawLine(middle_screen_x + 4 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, middle_screen_x + 0.5 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, RED); // right top of base
  drawLine(middle_screen_x - 4 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, middle_screen_x - 0.5 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, RED); // left top of base

  drawLine(middle_screen_x - 4 * VGAWIDTH / 38, VGAHEIGHT - 6 * VGAHEIGHT / 21, middle_screen_x - 4 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, RED); // left of base
  drawLine(middle_screen_x + 4 * VGAWIDTH / 38, VGAHEIGHT - 6 * VGAHEIGHT / 21, middle_screen_x + 4 * VGAWIDTH / 38, VGAHEIGHT - 7 * VGAHEIGHT / 21, RED); // right of base
  drawLine(middle_screen_x - 4 * VGAWIDTH / 38, VGAHEIGHT - 6 * VGAHEIGHT / 21, middle_screen_x + 4 * VGAWIDTH / 38, VGAHEIGHT - 6 * VGAHEIGHT / 21, RED); // bottom of base
}

void drawTwo()
{
  int middle_screen_x = VGAWIDTH / 2;
  drawCircle(middle_screen_x, VGAHEIGHT - 12 * VGAHEIGHT / 21, VGAWIDTH / 38 * (1.75), RED); // inner circle for bottom of 3
  drawCircle(middle_screen_x, VGAHEIGHT - 12 * VGAHEIGHT / 21, VGAWIDTH / 38 * (2.75), RED); // outter circle for bottom of 3
  fillRect(middle_screen_x - 3 * VGAWIDTH / 38, VGAHEIGHT - 12 * VGAHEIGHT / 21, 3 * VGAWIDTH / 38, 4 * VGAHEIGHT / 21, BLACK);
  fillRect(middle_screen_x, VGAHEIGHT - 11 * VGAHEIGHT / 21, 3 * VGAWIDTH / 38, 4 * VGAHEIGHT / 21, BLACK);
  drawLine(middle_screen_x + 2.5 * VGAWIDTH / 38, VGAHEIGHT - 10.75 * VGAHEIGHT / 21, middle_screen_x - 1 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, RED);  // right side diagonal line
  drawLine(middle_screen_x + 1.5 * VGAWIDTH / 38, VGAHEIGHT - 10.75 * VGAHEIGHT / 21, middle_screen_x - 2 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, RED);  // left side diagonal line
  drawLine(middle_screen_x - 2 * VGAWIDTH / 38, VGAHEIGHT - 5.75 * VGAHEIGHT / 21, middle_screen_x - 2 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, RED);     // left of base
  drawLine(middle_screen_x + 2.5 * VGAWIDTH / 38, VGAHEIGHT - 5.75 * VGAHEIGHT / 21, middle_screen_x + 2.5 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, RED); // right of base
  drawLine(middle_screen_x - 2 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, middle_screen_x + 2.5 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, RED);   // bottom of base
  drawLine(middle_screen_x + 2.5 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, middle_screen_x - 1 * VGAWIDTH / 38, VGAHEIGHT - 6.75 * VGAHEIGHT / 21, RED);   // top of base
}

void drawThree()
{
  int middle_screen_x = VGAWIDTH / 2;
  drawCircle(middle_screen_x, VGAHEIGHT - 8 * VGAHEIGHT / 21, VGAWIDTH / 38 * (1.75), RED); // inner circle for bottom of 3
  drawCircle(middle_screen_x, VGAHEIGHT - 8 * VGAHEIGHT / 21, VGAWIDTH / 38 * (2.75), RED); // outter circle for bottom of 3
  fillRect(middle_screen_x - 3 * VGAWIDTH / 38, VGAHEIGHT - 12 * VGAHEIGHT / 21, 2 * VGAWIDTH / 38, 4 * VGAHEIGHT / 21, BLACK);
  fillRect(middle_screen_x - 1 * VGAWIDTH / 38, VGAHEIGHT - 12 * VGAHEIGHT / 21, 2 * VGAWIDTH / 38, 3 * VGAHEIGHT / 21, BLACK);
  drawLine(middle_screen_x + .25 * VGAWIDTH / 38, VGAHEIGHT - 10.75 * VGAHEIGHT / 21, middle_screen_x + 2.75 * VGAWIDTH / 38, VGAHEIGHT - 12 * VGAHEIGHT / 21, RED);
  drawLine(middle_screen_x + 2.75 * VGAWIDTH / 38, VGAHEIGHT - 14 * VGAHEIGHT / 21, middle_screen_x + 2.75 * VGAWIDTH / 38, VGAHEIGHT - 12 * VGAHEIGHT / 21, RED);
  drawLine(middle_screen_x - 2.75 * VGAWIDTH / 38, VGAHEIGHT - 14 * VGAHEIGHT / 21, middle_screen_x + 2.75 * VGAWIDTH / 38, VGAHEIGHT - 12 * VGAHEIGHT / 21, RED);
  drawLine(middle_screen_x - 2.75 * VGAWIDTH / 38, VGAHEIGHT - 13 * VGAHEIGHT / 21, middle_screen_x + 2.75 * VGAWIDTH / 38, VGAHEIGHT - 12 * VGAHEIGHT / 21, RED);
  drawLine(middle_screen_x - 2.75 * VGAWIDTH / 38, VGAHEIGHT - 13 * VGAHEIGHT / 21, middle_screen_x + 1 * VGAWIDTH / 38, VGAHEIGHT - 13 * VGAHEIGHT / 21, RED);
  drawLine(middle_screen_x - 1 * VGAWIDTH / 38, VGAHEIGHT - 11 * VGAHEIGHT / 21, middle_screen_x + 1 * VGAWIDTH / 38, VGAHEIGHT - 13 * VGAHEIGHT / 21, RED);
  drawLine(middle_screen_x - 1 * VGAWIDTH / 38, VGAHEIGHT - 11 * VGAHEIGHT / 21, middle_screen_x - 1 * VGAWIDTH / 38, VGAHEIGHT - 9 * VGAHEIGHT / 21, RED);
}

void drawCountdown()
{
  fillRect(0, 0, VGAWIDTH, VGAHEIGHT, BLACK);
  drawField2();
  start_countdown = (float)time_us_64();
  drawThree();
  while (((float)time_us_64() - start_countdown) < 1000000)
  {
  };
  fillRect(0, 0, VGAWIDTH, VGAHEIGHT, BLACK);
  drawField2();
  start_countdown = (float)time_us_64();
  drawTwo();
  while (((float)time_us_64() - start_countdown) < 1000000)
  {
  };
  fillRect(0, 0, VGAWIDTH, VGAHEIGHT, BLACK);
  drawField2();
  start_countdown = (float)time_us_64();
  drawOne();
  while (((float)time_us_64() - start_countdown) < 1000000)
  {
  };
  fillRect(0, 0, VGAWIDTH, VGAHEIGHT, BLACK);
  drawField2();
}

void drawStartScreen()
{
  char str[6];
  drawField2();
  setTextSize(4);
  sprintf(str, "START");
  char *ptr = str;
  setCursor(VGAWIDTH/2 - 65, VGAHEIGHT / 2 - 40); // might need to adjust
  setTextColor2(RED, BLACK);
  writeStringBold(ptr);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Scoreboard

/////////////////////////////////////////////////////////////////////////////
// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  vgaPoint point;
  uint32_t ball_begin_time;
  uint32_t curr_time;
  // drawField2();

  // drawCountdown();
  drawStartScreen();
  // init_pitch();
  // pitch_release = (float)time_us_64();
  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();

    // update strikes and balls game logic

    // draw the boundaries
    // drawArena();
    // drawField2();
    // sweepBall();
    // updateTime(time_us_32());

    // sprintf(pt_serial_out_buffer, "ball_x: %f\n" ,ball_x);
    // serial_write;
    // sprintf(pt_serial_out_buffer, "ball_y: %f\n" ,ball_y);
    // serial_write;
    // sprintf(pt_serial_out_buffer, "ball_z: %f\n" ,ball_z);
    // serial_write;

    // sprintf(pt_serial_out_buffer, "ball_vx: %f\n" ,ball_vx);
    // serial_write;
    // sprintf(pt_serial_out_buffer, "ball_vy: %f\n" ,ball_vy);
    // serial_write;
    // sprintf(pt_serial_out_buffer, "ball_vz: %f\n" ,ball_vz);
    // serial_write;

    // sprintf(pt_serial_out_buffer, "hold_time: %f\n", hold_time);
    // serial_write;
    // sprintf(pt_serial_out_buffer, "hold_time: %f\n", hold_time);
    // serial_write;

    if (PLAYING)
    {
      // PITCHING
      drawBat();
      if (ball_z > 0 && !ball_has_reached_plate && !ball_flight)
      {
        update_pitch(iteration * dt);

        point = convert_to_vga(ball_x, ball_y, ball_z);
        drawCircle(point.vga_x, point.vga_y, point.vga_z * BALLRADIUS * 2, RED);
        iteration = iteration + 1;


        if (holding_power_button) // if holding button, update bar
        {
          hold_time = (float)time_us_64() - start_button_press;

          float intermediate_value = hold_time; // intermediate value holds power

          if (intermediate_value > .5 * 1000000)
          {
            intermediate_value = .5 * 1000000 - (hold_time - .5 * 1000000);
            if (intermediate_value < 0.1 * 1000000)
            {
              intermediate_value = 0.1 * 1000000;
            }
          }

          bat_power = intermediate_value / (.5 * 1000000);
          updatePower(bat_power);
        }
      }
      else if (!ball_has_reached_plate && !ball_flight) //draw ball at strike zone in magenta
      {
        /*just model pitches
        iteration = 0;
        init_pitch();
        fillRect(0, 0, VGAWIDTH, VGAHEIGHT, BLACK);
        drawField2();
        */

        vgaPoint new_point = convert_to_vga(model_pitch(x_a, x_g, mound_to_plate, x_h, x_n, mound_to_plate), model_pitch(y_a, y_g, mound_to_plate, y_h, y_n, mound_to_plate), 0);
        drawCircle(new_point.vga_x, new_point.vga_y, new_point.vga_z * BALLRADIUS * 2, MAGENTA);

        ball_has_reached_plate = true;
        // updateScore();
      }

      if (ball_has_reached_plate && bat_swung)
      {
        // in real operation, ball_has_reached_plate is set by the button interrupts and whether the ball conncets is determined here

        // model_random_flight_path();

        get_collision(bat_power, (release_button_press - pitch_release) / 1000000.0);

        hit_y_position = ball_y; // we need to record final position of ball when it gets hit by the bat
        ball_has_reached_plate = false;
        ball_flight = true;
        iteration = 1; // 0 breaks it
      }
      else if (ball_has_reached_plate && !bat_swung)
      {
        // check to see if strike or ball
        updateBallsorStrike();
      }

      if (ball_flight)
      {
        model_flight(dt); // lowk just dt?
        point = convert_to_vga(ball_x, ball_y, ball_z);
        drawCircle(point.vga_x, point.vga_y, point.vga_z * BALLRADIUS * 2, WHITE);
        iteration = iteration + 1;
      }

      if (ball_flight && ball_y < 0 || ball_has_reached_plate && !bat_swung)
      {
        ball_has_reached_plate = false;
        ball_flight = false;
        bat_swung = false;

        iteration = 0;
        // drawCountdown();
        init_pitch();
        pitch_release = (float)time_us_64();
        fillRect(0, 0, VGAWIDTH, VGAHEIGHT, BLACK);
        drawField2();
        already_hit = false;
        hold_time = 0.0;
      }

      // report ball z position
      setTextSize(1);
      char str[6];
      fillRect(VGAWIDTH - 60, 20, 10, 10, BLACK);
      sprintf(str, "%f", ball_z);
      char *ptr = str;
      setCursor(VGAWIDTH - 60, 20);
      setTextColor2(WHITE, BLACK);
      writeString(ptr);
      // fillRect(VGAWIDTH - 40, 20, 60, 10, BLACK);

      // FIGURE OUT IF THE GAME IS OVER:
      // if (player_points == 3)
      // {
      //   setTextSize(6);
      //   sprintf(str, "YOU WIN! GAME OVER");
      //   char *ptr = str;
      //   setCursor(70, VGAHEIGHT / 2 - 40);
      //   setTextColor2(RED, WHITE);
      //   writeStringBold(ptr);
      // }

  if (num_outs == 3)
  {
    start_countdown= (float)time_us_64();
    setTextSize(4);
    sprintf(str, "GAME OVER, YOU LOSE");
    char *ptr = str;
    setCursor(40, VGAHEIGHT / 2 - 40); // might need to adjust
    setTextColor2(RED, WHITE);
    writeStringBold(ptr);
    while (((float)time_us_64() - start_countdown) < 2000000)
    {
    }; // count 2 seconds


    PLAYING = false;
    drawStartScreen();
  }
  if (num_strikes == 3)
  { 
    char *ptr = str;
    if (num_outs<2){
    setTextSize(5);
    sprintf(str, "STRIKE OUT");
    char *ptr = str;
    setCursor(150, VGAHEIGHT / 2 - 40); // might need to adjust
    setTextColor2(RED, BLACK);
    writeStringBold(ptr);
    }
    num_outs+=1;


    setTextSize(1);
  
    num_balls = 0;
    num_strikes = 0;
    fillRect(100, 20, 10, 10, BLACK);
    // update num points
    sprintf(str, "%d", num_outs);
    setCursor(100, 20);
    setTextColor2(WHITE, BLACK);
    writeString(ptr);

    start_countdown= (float)time_us_64();
    while (((float)time_us_64() - start_countdown) < 1000000)
    {
    }; // count 1 seconds
    drawField2();
  }

      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time);

      if (spare_time < 0)
      {
        gpio_put(LED_PIN, 1);
      }
      else
      {
        gpio_put(LED_PIN, 0);
      }

      // yield for necessary amount of time
      PT_YIELD_usec(spare_time);
      // NEVER exit while
    }
    else
    {
      PT_YIELD_usec(FRAME_RATE);
    }

  } // END WHILE(1
  PT_END(pt);
} // animation thread

void on_falling_edge(uint gpio, uint32_t events)
{
  // sprintf(pt_serial_out_buffer, "Rising edge detected on GPIO %d\n", gpio);
  // serial_write;
  if (PLAYING)
  {
    if (!already_hit)
    {
      holding_power_button = true;
      start_button_press = (float)time_us_64();
    }
    already_hit = true;
  }
  else
  {
    PLAYING = true;
    drawField2();
    num_outs=0;
    num_strikes = 0;
    player_points = 0;
    num_balls = 0;
    init_pitch();
  }
}

void on_rising_edge(uint gpio, uint32_t events)
{
  // sprintf(pt_serial_out_buffer, "Falling edge detected on GPIO %d\n", gpio);
  // serial_write;
  // if (!already_hit)
  // {
  //     hold_time = ((float) time_us_64()) - start_button_press;
  // }
  if (PLAYING)
  {
    holding_power_button = false;
    release_button_press = (float)time_us_64();
    bat_swung = true;
  }
}

void gpio_callback(uint gpio, uint32_t events)
{
  if (events & GPIO_IRQ_EDGE_RISE) // replace with first enter boolean
  {
    on_rising_edge(gpio, events);
  }
  else if (events & GPIO_IRQ_EDGE_FALL) // replace with second enter boolean
  {
    on_falling_edge(gpio, events);
  }
  else
  {
    hold_time = 69420.0;
  }
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
  //_sys_clock_khz(250000, true); // set to 250Ghz to "overclock" system
  // initialize stio
  stdio_init_all();
  adc_init();
  adc_gpio_init(26); // GPIO 26 = poteniometer
  adc_select_input(0);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 0);

  gpio_init(BUTTONPIN);
  gpio_set_dir(BUTTONPIN, GPIO_IN);

  gpio_init(POWER_BUTTON);
  gpio_set_dir(POWER_BUTTON, GPIO_IN);
  gpio_pull_up(POWER_BUTTON);                                                                                      // or gpio_pull_up(PIN); depending on your use
  gpio_set_irq_enabled_with_callback(POWER_BUTTON, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback); // check gpio any

  init_audio();

  // ========================================
  // === DMA Setup
  // ========================================
  // Initialize SPI channel (channel, baud rate set to 20MHz)
  // spi_init(SPI_PORT, 20000000);

  // setup rosc for random number generation
  rosc_setup();

  // initialize VGA
  initVGA();

  // commented out multicore functionality
  //  start core 1
  //  multicore_reset_core1();
  //  multicore_launch_core1(&core1_main);

  // add threads
  // pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start;
}
