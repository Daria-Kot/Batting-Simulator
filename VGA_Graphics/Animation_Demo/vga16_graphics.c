#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// Our assembled programs:
// Each gets the name <pio_filename.pio.h>
#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"
// Header file
#include "vga16_graphics.h"
// Font file
#include "glcdfont.c"
#include "font_rom_brl4.h"

// VGA timing constants
#define H_ACTIVE 655   // (active + frontporch - 1) - one cycle delay for mov
#define V_ACTIVE 479   // (active - 1)
#define RGB_ACTIVE 319 // (horizontal active)/2 - 1
// #define RGB_ACTIVE 639 // change to this if 1 pixel/byte

// Length of the pixel array, and number of DMA transfers
#define TXCOUNT 153600 // Total pixels/2 (since we have 2 pixels per byte)

// Pixel color array that is DMA's to the PIO machines and
// a pointer to the ADDRESS of this color array.
// Note that this array is automatically initialized to all 0's (black)
unsigned char vga_data_array[TXCOUNT];
char *address_pointer = &vga_data_array[0];

// Bit masks for drawPixel routine
#define TOPMASK 0b00001111
#define BOTTOMMASK 0b11110000

// For drawLine
#define swap(a, b) \
  {                \
    short t = a;   \
    a = b;         \
    b = t;         \
  }

// For writing text
#define tabspace 4 // number of spaces for a tab

// For accessing the font library
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

// For drawing characters
unsigned short cursor_y, cursor_x, textsize;
char textcolor, textbgcolor, wrap;

// Screen width/height
#define _width 640
#define _height 480


#define camera_height 1.5
#define camera_to_plate 5

typedef struct
{
  int vga_x;
  int vga_y;
  float vga_z;
} vgaPoint;
#define scale_factor 0.5
vgaPoint convert_to_vga_graphics_file(float x, float y, float z)
{
  int center_x = 320;
  // int center_y = 320  - 150; //not upside down
  int center_y = 225; // 480*1/3;
  float f = 500;      // Focal length in pixels, found with 150 degree FOV was 400, 160 400 works

  z = z + camera_to_plate; // added

  vgaPoint point = {0, 0, 0.0};

  // Protect against divide-by-zero

  point.vga_x = center_x + (int)((x * f) / (1 + z));                                    // maybe 1+z
  point.vga_y = center_y + (int)(((y - camera_height) * f) / ((1 + z) * scale_factor)); // added + camera height
  // point.vga_y = point.vga_y - (point.vga_y - 240);

  point.vga_z = f / z;
  return point;
}



void initVGA()
{
  // Choose which PIO instance to use (there are two instances, each with 4 state machines)
  PIO pio = pio0;

  // Our assembled program needs to be loaded into this PIO's instruction
  // memory. This SDK function will find a location (offset) in the
  // instruction memory where there is enough space for our program. We need
  // to remember these locations!
  //
  // We only have 32 instructions to spend! If the PIO programs contain more than
  // 32 instructions, then an error message will get thrown at these lines of code.
  //
  // The program name comes from the .program part of the pio file
  // and is of the form <program name_program>
  uint hsync_offset = pio_add_program(pio, &hsync_program);
  uint vsync_offset = pio_add_program(pio, &vsync_program);
  uint rgb_offset = pio_add_program(pio, &rgb_program);

  // Manually select a few state machines from pio instance pio0.
  uint hsync_sm = 0;
  uint vsync_sm = 1;
  uint rgb_sm = 2;

  // Call the initialization functions that are defined within each PIO file.
  // Why not create these programs here? By putting the initialization function in
  // the pio file, then all information about how to use/setup that state machine
  // is consolidated in one place. Here in the C, we then just import and use it.
  hsync_program_init(pio, hsync_sm, hsync_offset, HSYNC);
  vsync_program_init(pio, vsync_sm, vsync_offset, VSYNC);
  rgb_program_init(pio, rgb_sm, rgb_offset, LO_GRN);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // ============================== PIO DMA Channels =================================================
  /////////////////////////////////////////////////////////////////////////////////////////////////////

  // DMA channels - 0 sends color data, 1 reconfigures and restarts 0
  int rgb_chan_0 = dma_claim_unused_channel(true);
  int rgb_chan_1 = dma_claim_unused_channel(true);

  // Channel Zero (sends color data to PIO VGA machine)
  dma_channel_config c0 = dma_channel_get_default_config(rgb_chan_0); // default configs
  channel_config_set_transfer_data_size(&c0, DMA_SIZE_8);             // 8-bit txfers
  channel_config_set_read_increment(&c0, true);                       // yes read incrementing
  channel_config_set_write_increment(&c0, false);                     // no write incrementing
  channel_config_set_dreq(&c0, DREQ_PIO0_TX2);                        // DREQ_PIO0_TX2 pacing (FIFO)
  channel_config_set_chain_to(&c0, rgb_chan_1);                       // chain to other channel

  dma_channel_configure(
      rgb_chan_0,        // Channel to be configured
      &c0,               // The configuration we just created
      &pio->txf[rgb_sm], // write address (RGB PIO TX FIFO)
      &vga_data_array,   // The initial read address (pixel color array)
      TXCOUNT,           // Number of transfers; in this case each is 1 byte.
      false              // Don't start immediately.
  );

  // Channel One (reconfigures the first channel)
  dma_channel_config c1 = dma_channel_get_default_config(rgb_chan_1); // default configs
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);            // 32-bit txfers
  channel_config_set_read_increment(&c1, false);                      // no read incrementing
  channel_config_set_write_increment(&c1, false);                     // no write incrementing
  channel_config_set_chain_to(&c1, rgb_chan_0);                       // chain to other channel

  dma_channel_configure(
      rgb_chan_1,                        // Channel to be configured
      &c1,                               // The configuration we just created
      &dma_hw->ch[rgb_chan_0].read_addr, // Write address (channel 0 read address)
      &address_pointer,                  // Read address (POINTER TO AN ADDRESS)
      1,                                 // Number of transfers, in this case each is 4 byte
      false                              // Don't start immediately.
  );

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////

  // Initialize PIO state machine counters. This passes the information to the state machines
  // that they retrieve in the first 'pull' instructions, before the .wrap_target directive
  // in the assembly. Each uses these values to initialize some counting registers.
  pio_sm_put_blocking(pio, hsync_sm, H_ACTIVE);
  pio_sm_put_blocking(pio, vsync_sm, V_ACTIVE);
  pio_sm_put_blocking(pio, rgb_sm, RGB_ACTIVE);

  // Start the two pio machine IN SYNC
  // Note that the RGB state machine is running at full speed,
  // so synchronization doesn't matter for that one. But, we'll
  // start them all simultaneously anyway.
  pio_enable_sm_mask_in_sync(pio, ((1u << hsync_sm) | (1u << vsync_sm) | (1u << rgb_sm)));

  // Start DMA channel 0. Once started, the contents of the pixel color array
  // will be continously DMA's to the PIO machines that are driving the screen.
  // To change the contents of the screen, we need only change the contents
  // of that array.
  dma_start_channel_mask((1u << rgb_chan_0));
}

// A function for drawing a pixel with a specified color.
// Note that because information is passed to the PIO state machines through
// a DMA channel, we only need to modify the contents of the array and the
// pixels will be automatically updated on the screen.
void drawPixel(short x, short y, char color)
{
  // Range checks (640x480 display)
  if (x > 639)
    x = 639;
  if (x < 0)
    x = 0;
  if (y < 0)
    y = 0;
  if (y > 479)
    y = 479;
  // if((x > 639) | (x < 0) | (y > 479) | (y < 0) ) return;

  // Which pixel is it?
  int pixel = ((640 * y) + x);

  // Is this pixel stored in the first 4 bits
  // of the vga data array index, or the second
  // 4 bits? Check, then mask.
  if (pixel & 1)
  {
    vga_data_array[pixel >> 1] = (vga_data_array[pixel >> 1] & TOPMASK) | (color << 4);
  }
  else
  {
    vga_data_array[pixel >> 1] = (vga_data_array[pixel >> 1] & BOTTOMMASK) | (color);
  }
}

void drawVLine(short x, short y, short h, char color)
{
  for (short i = y; i < (y + h); i++)
  {
    drawPixel(x, i, color);
  }
}

void drawHLine(short x, short y, short w, char color)
{
  for (short i = x; i < (x + w); i++)
  {
    drawPixel(i, y, color);
  }
}

// Bresenham's algorithm - thx wikipedia and thx Bruce!
void drawLine(short x0, short y0, short x1, short y1, char color)
{
  /* Draw a straight line from (x0,y0) to (x1,y1) with given color
   * Parameters:
   *      x0: x-coordinate of starting point of line. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y0: y-coordinate of starting point of line. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      x1: x-coordinate of ending point of line. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y1: y-coordinate of ending point of line. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      color: 3-bit color value for line
   */
  short steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1)
  {
    swap(x0, x1);
    swap(y0, y1);
  }

  short dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  short err = dx / 2;
  short ystep;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  for (; x0 <= x1; x0++)
  {
    if (steep)
    {
      drawPixel(y0, x0, color);
    }
    else
    {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0)
    {
      y0 += ystep;
      err += dx;
    }
  }
}

// Draw a rectangle
void drawRect(short x, short y, short w, short h, char color)
{
  /* Draw a rectangle outline with top left vertex (x,y), width w
   * and height h at given color
   * Parameters:
   *      x:  x-coordinate of top-left vertex. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y:  y-coordinate of top-left vertex. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      w:  width of the rectangle
   *      h:  height of the rectangle
   *      color:  16-bit color of the rectangle outline
   * Returns: Nothing
   */
  drawHLine(x, y, w, color);
  drawHLine(x, y + h - 1, w, color);
  drawVLine(x, y, h, color);
  drawVLine(x + w - 1, y, h, color);
}

void drawCircle(short x0, short y0, short r, char color)
{
  /* Draw a circle outline with center (x0,y0) and radius r, with given color
   * Parameters:
   *      x0: x-coordinate of center of circle. The top-left of the screen
   *          has x-coordinate 0 and increases to the right
   *      y0: y-coordinate of center of circle. The top-left of the screen
   *          has y-coordinate 0 and increases to the bottom
   *      r:  radius of circle
   *      color: 16-bit color value for the circle. Note that the circle
   *          isn't filled. So, this is the color of the outline of the circle
   * Returns: Nothing
   */
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  drawPixel(x0, y0 + r, color);
  drawPixel(x0, y0 - r, color);
  drawPixel(x0 + r, y0, color);
  drawPixel(x0 - r, y0, color);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void drawCircleHelper(short x0, short y0, short r, unsigned char cornername, char color)
{
  // Helper function for drawing circles and circular objects
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (cornername & 0x4)
    {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2)
    {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8)
    {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1)
    {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void fillCircle(short x0, short y0, short r, char color)
{
  /* Draw a filled circle with center (x0,y0) and radius r, with given color
   * Parameters:
   *      x0: x-coordinate of center of circle. The top-left of the screen
   *          has x-coordinate 0 and increases to the right
   *      y0: y-coordinate of center of circle. The top-left of the screen
   *          has y-coordinate 0 and increases to the bottom
   *      r:  radius of circle
   *      color: 16-bit color value for the circle
   * Returns: Nothing
   */
  drawVLine(x0, y0 - r, 2 * r + 1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}

void fillCircleHelper(short x0, short y0, short r, unsigned char cornername, short delta, char color)
{
  // Helper function for drawing filled circles
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    if (cornername & 0x1)
    {
      drawVLine(x0 + x, y0 - y, 2 * y + 1 + delta, color);
      drawVLine(x0 + y, y0 - x, 2 * x + 1 + delta, color);
    }
    if (cornername & 0x2)
    {
      drawVLine(x0 - x, y0 - y, 2 * y + 1 + delta, color);
      drawVLine(x0 - y, y0 - x, 2 * x + 1 + delta, color);
    }
  }
}

void fillEllipseHelper(short x0, short y0, short rx, short ry, char color)
{
  for (short y = -ry; y <= ry; y++)
  {
    for (short x = -rx; x <= rx; x++)
    {
      if ((x * x) * (ry * ry) + (y * y) * (rx * rx) <= (rx * rx) * (ry * ry))
      {
        drawPixel(x0 + x, y0 + y, color);
      }
    }
  }
}

// Draw a 3D-looking filled circle (really an ellipse) given real-world meters
void fill3DCircle(float x_m, float y_m, float z_m, float radius_m, char color)
{
  // Project 3D position into 2D screen space
  vgaPoint pt = convert_to_vga_graphics_file(x_m, y_m, z_m);

  if (z_m <= 0)
    return; // Object is behind camera; don't draw

  // Compute scaling factors based on depth
  float scale_x = pt.vga_z;
  float scale_y = pt.vga_z * 0.6f; // Vertical compression for depth illusion

  // Convert radius from meters to screen pixels
  short ellipse_rx = (short)(radius_m * scale_x); // horizontal radius
  short ellipse_ry = (short)(radius_m * scale_y); // vertical radius

  // Draw the ellipse centered at (screen_x, screen_y)
  fillEllipseHelper((short)(pt.vga_x),
                    (short)(pt.vga_y),
                    ellipse_rx,
                    ellipse_ry,
                    color);
}

// Draw a rounded rectangle
void drawRoundRect(short x, short y, short w, short h, short r, char color)
{
  /* Draw a rounded rectangle outline with top left vertex (x,y), width w,
   * height h and radius of curvature r at given color
   * Parameters:
   *      x:  x-coordinate of top-left vertex. The x-coordinate of
   *          the top-left of the screen is 0. It increases to the right.
   *      y:  y-coordinate of top-left vertex. The y-coordinate of
   *          the top-left of the screen is 0. It increases to the bottom.
   *      w:  width of the rectangle
   *      h:  height of the rectangle
   *      color:  16-bit color of the rectangle outline
   * Returns: Nothing
   */
  // smarter version
  drawHLine(x + r, y, w - 2 * r, color);         // Top
  drawHLine(x + r, y + h - 1, w - 2 * r, color); // Bottom
  drawVLine(x, y + r, h - 2 * r, color);         // Left
  drawVLine(x + w - 1, y + r, h - 2 * r, color); // Right
  // draw four corners
  drawCircleHelper(x + r, y + r, r, 1, color);
  drawCircleHelper(x + w - r - 1, y + r, r, 2, color);
  drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
  drawCircleHelper(x + r, y + h - r - 1, r, 8, color);
}

// Fill a rounded rectangle
void fillRoundRect(short x, short y, short w, short h, short r, char color)
{
  // smarter version
  fillRect(x + r, y, w - 2 * r, h, color);

  // draw four corners
  fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
  fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}

// fill a rectangle
void fillRect(short x, short y, short w, short h, char color)
{
  /* Draw a filled rectangle with starting top-left vertex (x,y),
   *  width w and height h with given color
   * Parameters:
   *      x:  x-coordinate of top-left vertex; top left of screen is x=0
   *              and x increases to the right
   *      y:  y-coordinate of top-left vertex; top left of screen is y=0
   *              and y increases to the bottom
   *      w:  width of rectangle
   *      h:  height of rectangle
   *      color:  3-bit color value
   * Returns:     Nothing
   */

  // rudimentary clipping (drawChar w/big text requires this)
  // if((x >= _width) || (y >= _height)) return;
  // if((x + w - 1) >= _width)  w = _width  - x;
  // if((y + h - 1) >= _height) h = _height - y;

  // tft_setAddrWindow(x, y, x+w-1, y+h-1);

  for (int i = x; i < (x + w); i++)
  {
    for (int j = y; j < (y + h); j++)
    {
      drawPixel(i, j, color);
    }
  }
}

void fill3DRect(float x_m, float y_m, float z_m, float width_m, float height_m, char color)
{
  if (z_m <= 0)
    return; // behind camera

  // Parameters for slight vertical offset
  float height_offset_m = height_m / 2.0f; // half the real-world height

  // Top of the rectangle is slightly farther away
  float z_top = z_m + height_offset_m;
  float z_bottom = z_m - height_offset_m;

  // Protect against bad geometry
  if (z_top <= 0.1f)
    z_top = 0.1f;
  if (z_bottom <= 0.1f)
    z_bottom = 0.1f;

  // Project center of the rectangle
  vgaPoint center_pt = convert_to_vga_graphics_file(x_m, y_m, z_m);

  short center_x = (short)(center_pt.vga_x);
  short center_y = (short)(center_pt.vga_y);

  // Scaling factors
  float scale_center = center_pt.vga_z;
  float scale_top = 85.0f / z_top;
  float scale_bottom = 85.0f / z_bottom;

  // Widths based on distance
  short half_top_width = (short)((width_m * scale_top) / 2.0f);
  short half_bottom_width = (short)((width_m * scale_bottom) / 2.0f);

  // Heights scale with center scale
  short half_height = (short)((height_m * scale_center * 0.6f) / 2.0f); // 0.6 squish factor for vertical

  // Now draw from top to bottom, line by line
  for (short dy = -half_height; dy <= half_height; dy++)
  {
    // Interpolate width between top and bottom
    float t = (float)(dy + half_height) / (2.0f * half_height); // t = 0 (top) to 1 (bottom)
    short current_half_width = (short)((1.0f - t) * half_top_width + t * half_bottom_width);

    drawHLine(center_x - current_half_width,
              center_y + dy,
              2 * current_half_width,
              color);
  }
}

// Draw a character
void drawChar(short x, short y, unsigned char c, char color, char bg, unsigned char size)
{
  char i, j;
  if ((x >= _width) ||            // Clip right
      (y >= _height) ||           // Clip bottom
      ((x + 6 * size - 1) < 0) || // Clip left
      ((y + 8 * size - 1) < 0))   // Clip top
    return;

  for (i = 0; i < 6; i++)
  {
    unsigned char line;
    if (i == 5)
      line = 0x0;
    else
      line = pgm_read_byte(font + (c * 5) + i);
    for (j = 0; j < 8; j++)
    {
      if (line & 0x1)
      {
        if (size == 1) // default size
          drawPixel(x + i, y + j, color);
        else
        { // big size
          fillRect(x + (i * size), y + (j * size), size, size, color);
        }
      }
      else if (bg != color)
      {
        if (size == 1) // default size
          drawPixel(x + i, y + j, bg);
        else
        { // big size
          fillRect(x + i * size, y + j * size, size, size, bg);
        }
      }
      line >>= 1;
    }
  }
}

inline void setCursor(short x, short y)
{
  /* Set cursor for text to be printed
   * Parameters:
   *      x = x-coordinate of top-left of text starting
   *      y = y-coordinate of top-left of text starting
   * Returns: Nothing
   */
  cursor_x = x;
  cursor_y = y;
}

inline void setTextSize(unsigned char s)
{
  /*Set size of text to be displayed
   * Parameters:
   *      s = text size (1 being smallest)
   * Returns: nothing
   */
  textsize = (s > 0) ? s : 1;
}

inline void setTextColor(char c)
{
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

inline void setTextColor2(char c, char b)
{
  /* Set color of text to be displayed
   * Parameters:
   *      c = 16-bit color of text
   *      b = 16-bit color of text background
   */
  textcolor = c;
  textbgcolor = b;
}

inline void setTextWrap(char w)
{
  wrap = w;
}

void tft_write(unsigned char c)
{
  if (c == '\n')
  {
    cursor_y += textsize * 8;
    cursor_x = 0;
  }
  else if (c == '\r')
  {
    // skip em
  }
  else if (c == '\t')
  {
    int new_x = cursor_x + tabspace;
    if (new_x < _width)
    {
      cursor_x = new_x;
    }
  }
  else
  {
    drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
    cursor_x += textsize * 6;
    if (wrap && (cursor_x > (_width - textsize * 6)))
    {
      cursor_y += textsize * 8;
      cursor_x = 0;
    }
  }
}

inline void writeString(char *str)
{
  /* Print text onto screen
   * Call tft_setCursor(), tft_setTextColor(), tft_setTextSize()
   *  as necessary before printing
   */
  while (*str)
  {
    tft_write(*str++);
  }
}

//=================================================
// added 10/16/2023 brl4
inline void setTextColorBig(char color, char background)
{
  /* Set color of text to be displayed
   * Parameters:
   *      color = 16-bit color of text
   *      b = 16-bit color of text background
   *      background ==-1 means trasnparten background
   */
  textcolor = color;
  textbgcolor = background;
}
//=================================================
// added 10/11/2023 brl4
// Draw a character
void drawCharBig(short x, short y, unsigned char c, char color, char bg)
{
  char i, j;
  unsigned char line;
  for (i = 0; i < 15; i++)
  {
    line = pgm_read_byte(bigFont + ((int)c * 16) + i);
    for (j = 0; j < 8; j++)
    {
      if (line & 0x80)
      {
        drawPixel(x + j, y + i, color);
      }
      else if (bg != color)
      {
        drawPixel(x + j, y + i, bg);
      }
      line <<= 1;
    }
  }
}

inline void writeStringBig(char *str)
{
  /* Print text onto screen
   * Call tft_setCursor(), tft_setTextColorBig()
   *  as necessary before printing
   */
  while (*str)
  {
    char c = *str++;
    drawCharBig(cursor_x, cursor_y, c, textcolor, textbgcolor);
    cursor_x += 8;
  }
}

inline void writeStringBold(char *str)
{
  /* Print text onto screen
   * Call tft_setCursor(), tft_setTextColorBig()
   *  as necessary before printing
   */
  /* Print text onto screen
   * Call tft_setCursor(), tft_setTextColor(), tft_setTextSize()
   *  as necessary before printing
   */
  char temp_bg;
  temp_bg = textbgcolor;
  while (*str)
  {
    char c = *str++;
    drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
    drawChar(cursor_x + 1, cursor_y, c, textcolor, textcolor, textsize);
    cursor_x += 7 * textsize;
  }
  textbgcolor = temp_bg;
}


// Fill a convex quadrilateral given four (x,y) pairs
void fillQuad(short x0, short y0, short x1, short y1,
            short x2, short y2, short x3, short y3,
            char color) {
  // Find top-most and bottom-most y coordinates
  short ymin = y0, ymax = y0;
  if (y1 < ymin) ymin = y1;
  if (y2 < ymin) ymin = y2;
  if (y3 < ymin) ymin = y3;

  if (y1 > ymax) ymax = y1;
  if (y2 > ymax) ymax = y2;
  if (y3 > ymax) ymax = y3;

  // For each scanline (horizontal line)
  for(short y = ymin; y <= ymax; y++) {
      short x_intercepts[4];
      int count = 0;

      // Check each edge for intersection with scanline
      short edges[4][4] = {
          {x0, y0, x1, y1},
          {x1, y1, x2, y2},
          {x2, y2, x3, y3},
          {x3, y3, x0, y0}
      };

      for (int i = 0; i < 4; i++) {
          short ex0 = edges[i][0];
          short ey0 = edges[i][1];
          short ex1 = edges[i][2];
          short ey1 = edges[i][3];

          // If the scanline crosses this edge
          if ((ey0 <= y && ey1 > y) || (ey1 <= y && ey0 > y)) {
              float slope = (float)(ex1 - ex0) / (ey1 - ey0);
              short x = (short)(ex0 + (y - ey0) * slope);
              x_intercepts[count++] = x;
          }
      }

      // If two intercepts found, draw horizontal line
      if (count == 2) {
          if (x_intercepts[0] > x_intercepts[1]) {
              swap(x_intercepts[0], x_intercepts[1]);
          }
          drawHLine(x_intercepts[0], y, x_intercepts[1] - x_intercepts[0], color);
      }
  }
}