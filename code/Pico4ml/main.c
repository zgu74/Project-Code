#include <stdio.h>
#include "pico/stdlib.h"
#include <tusb.h>
#include "pico/multicore.h"
#include "arducam/arducam.h"
#include "lib/st7735.h"
#include "lib/fonts.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"



#define FLAG_VALUE 123


#include "hardware/vreg.h"

/////////////////////////////////////////////////////////////////////
////////////////////////        VGA        //////////////////////////
// VGA timing constants
#define H_ACTIVE   655    // (active + frontporch - 1) - one cycle delay for mov
#define V_ACTIVE   479    // (active - 1)
#define RGB_ACTIVE 319    // (horizontal active)/2 - 1
// #define RGB_ACTIVE 639 // change to this if 1 pixel/byte

// Length of the pixel array, and number of DMA transfers
#define TXCOUNT 153600 // Total pixels/2 (since we have 2 pixels per byte)

// Pixel color array that is DMA's to the PIO machines and
// a pointer to the ADDRESS of this color array.
// Note that this array is automatically initialized to all 0's (black)
unsigned char vga_data_array[TXCOUNT];
char * address_pointer = &vga_data_array[0] ;

// Give the I/O pins that we're using some names that make sense
#define HSYNC     21
#define VSYNC     17
#define RED_PIN   18
#define GREEN_PIN 19
#define BLUE_PIN  20

// We can only produce 8 colors, so let's give them readable names
#define BLACK   0
#define RED     6
#define GREEN   3
#define YELLOW  4
#define BLUE    1
#define MAGENTA 5
#define CYAN    2
#define WHITE   7


// A function for drawing a pixel with a specified color.
// Note that because information is passed to the PIO state machines through
// a DMA channel, we only need to modify the contents of the array and the
// pixels will be automatically updated on the screen.
void drawPixel(int x, int y, char color) {
    // Range checks
    if (x > 639) x = 639 ;
    if (x < 0) x = 0 ;
    if (y < 0) y = 0 ;
    if (y > 479) y = 479 ;

    // Which pixel is it?
    int pixel = ((640 * y) + x) ;

    // Is this pixel stored in the first 3 bits
    // of the vga data array index, or the second
    // 3 bits? Check, then mask.
    if (pixel & 1) {
        vga_data_array[pixel>>1] |= (color << 3) ;
    }
    else {
        vga_data_array[pixel>>1] |= (color) ;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// Stuff for Mandelbrot ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////        VGA        //////////////////////////
/////////////////////////////////////////////////////////////////////

int main() {
  int loops=20;
  int count=0;
  int i=0;
  int j=0;
  int readio=0;
  stdio_init_all();

  uint8_t image_buf[324*324];
  //uint8_t displayBuf[80*160*2];
  //uint8_t header[2] = {0x55,0xAA};

  // while (!tud_cdc_connected()) ;

  // printf("tud_cdc_connected(%d)\n", tud_cdc_connected()?1:0);
  gpio_set_function(28,5);
  gpio_init(28);
  gpio_set_dir(28, GPIO_IN);


  vreg_set_voltage(VREG_VOLTAGE_1_30);
  sleep_ms(1000);
  set_sys_clock_khz(250000, true);

 	gpio_init(PIN_LED);
	gpio_set_dir(PIN_LED, GPIO_OUT);

	//ST7735_Init();
	//ST7735_DrawImage(0, 0, 80, 160, arducam_logo);

	struct arducam_config config;
	config.sccb = i2c0;
	config.sccb_mode = I2C_MODE_16_8;
	config.sensor_address = 0x24;
	config.pin_sioc = PIN_CAM_SIOC;
	config.pin_siod = PIN_CAM_SIOD;
	config.pin_resetb = PIN_CAM_RESETB;
	config.pin_xclk = PIN_CAM_XCLK;
	config.pin_vsync = PIN_CAM_VSYNC;
	config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;

	config.pio = pio1;
	config.pio_sm = 0;

	config.dma_channel = 0;
	config.image_buf = image_buf;
	config.image_buf_size = sizeof(image_buf);

	arducam_init(&config);

  
   // while(gpio_get(1)==0);

	



  while (1)
    {
	  gpio_put(PIN_LED, !gpio_get(PIN_LED));
	  arducam_capture_frame(&config);

	  uint16_t index = 0;
	  // for (int y = 0; y < 160; y++) {
	  //   for (int x = 0; x < 80; x++) {
    //           uint8_t c = image_buf[(2+320-2*y)*324+(2+40+2*x)];
		// 	  //printf("x,y:%d,%d   c:%d\n",x,y,c);
		// 	  //if(c<=50) c=0; 
    //           uint16_t imageRGB   = ST7735_COLOR565(c, c, c);
    //           displayBuf[index++] = (uint8_t)(imageRGB >> 8) & 0xFF;
    //           displayBuf[index++] = (uint8_t)(imageRGB)&0xFF;
    //         }
	  // }
	  // ST7735_DrawImage(0, 0, 80, 160, displayBuf);
    gpio_put(28,0);
    readio=gpio_get(28);
	  //count++;
	  if(readio==1)
	  {
		/////////////////////////////////////////////////////////////////////
	////////////////////////        VGA        //////////////////////////
  // for(i=0;i<160*8*2;i++)
  //   {
  //     printf("i:%d",displayBuf[i]);
  //   }
  //gpio_put(PIN_LED, !gpio_get(PIN_LED));
	set_sys_clock_khz(125000, true);
  pio_clear_instruction_memory(pio1);
    PIO pio = pio0;
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
    rgb_program_init(pio, rgb_sm, rgb_offset, RED_PIN);


    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // ===========================-== DMA Data Channels =================================================
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // DMA channels - 0 sends color data, 1 reconfigures and restarts 0
    int rgb_chan_0 = 0;
    int rgb_chan_1 = 1;

    // Channel Zero (sends color data to PIO VGA machine)
    dma_channel_config c0 = dma_channel_get_default_config(rgb_chan_0);  // default configs
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_8);              // 8-bit txfers
    channel_config_set_read_increment(&c0, true);                        // yes read incrementing
    channel_config_set_write_increment(&c0, false);                      // no write incrementing
    channel_config_set_dreq(&c0, DREQ_PIO0_TX2) ;                        // DREQ_PIO0_TX2 pacing (FIFO)
    channel_config_set_chain_to(&c0, rgb_chan_1);                        // chain to other channel

    dma_channel_configure(
        rgb_chan_0,                 // Channel to be configured
        &c0,                        // The configuration we just created
        &pio->txf[rgb_sm],          // write address (RGB PIO TX FIFO)
        &vga_data_array,            // The initial read address (pixel color array)
        TXCOUNT,                    // Number of transfers; in this case each is 1 byte.
        false                       // Don't start immediately.
    );

    // Channel One (reconfigures the first channel)
    dma_channel_config c1 = dma_channel_get_default_config(rgb_chan_1);   // default configs
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);              // 32-bit txfers
    channel_config_set_read_increment(&c1, false);                        // no read incrementing
    channel_config_set_write_increment(&c1, false);                       // no write incrementing
    channel_config_set_chain_to(&c1, rgb_chan_0);                         // chain to other channel

    dma_channel_configure(
        rgb_chan_1,                         // Channel to be configured
        &c1,                                // The configuration we just created
        &dma_hw->ch[rgb_chan_0].read_addr,  // Write address (channel 0 read address)
        &address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers, in this case each is 4 byte
        false                               // Don't start immediately.
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
    dma_start_channel_mask((1u << rgb_chan_0)) ;

  //  uint8_t image_buf_2[324*324];
  //  index=0;
  //  for (int y = 0; y < 320; y++) {
	//     for (int x = 0; x < 160; x++) {
  //             image_buf_2[index++] = image_buf[(2+320-y)*324+(2+40+x)];
  //     }
  //  }

  //  for (int x=0; x<320*160;x++)
  //  {
  //   image_buf_2[x]=image_buf_2[x]/32;
  //  }

  index=0;
  int pixel;
	while(1)
	{
		for (i=0; i<640; i++) {
            
            for (j=0; j<480; j++) {

                                // Draw the pixel
                // if (i <= 160) drawPixel(i, j, BLACK) ;
                // else if (i>160&&i<=320) drawPixel(i, j, YELLOW) ;
                // else if (i>320&&i<=480) drawPixel(i, j, YELLOW) ;
                // else if(i>480) drawPixel(i, j, RED);
              //  if(j>=160) drawPixel(i, j, BLACK) ;
              //  else if(i>=320) drawPixel(i, j, BLACK) ;
              //  else if(i<320&&j<160) drawPixel(i, j, image_buf_2[i*320+j]) ;
              // drawPixel(i, j, image_buf_2[i*640+j]) ;
              pixel=image_buf[(2+320-i/2)*324+(2+40+j/3)];
               if(pixel<=31)pixel=0;
               else if(pixel>=32&&pixel<96)pixel=2*pixel-32;
               else pixel=0.6*pixel+102;
              pixel=pixel/32;               
              drawPixel(i,j,pixel); 

            }
        }
	}
	  }

	}
}
