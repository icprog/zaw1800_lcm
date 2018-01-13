#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif

#include "lcm_drv.h"
#include "lcm_adc_define.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH (480)
#define FRAME_HEIGHT (854)
#define ILI9806E_LCM_ID (0x0604)

// Uncomment to use LCM_1, LCM_2 is used by default
// #define LCM_1 1

#define REGFLAG_DELAY 0xFE

#ifdef LCM_1
#define REGFLAG_END_OF_TABLE 0xFFF // END OF REGISTERS MARKER
#else                              // LCM_2
#define REGFLAG_END_OF_TABLE 0xFF  // END OF REGISTERS MARKER
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

static unsigned int lcm_esd_test = FALSE; // only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                  (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                         (lcm_util.udelay(n))
#define MDELAY(n)                                         (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)  lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)     lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                    lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                     lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)             lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table
{
  unsigned char cmd;
  unsigned char count;
  unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
#if 0 // def LCM_1 // LCM_1
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x01}},
  {0x08, 1, {0x18}},
  {0x21, 1, {0x01}},
  {0x30, 1, {0x01}},
  {0x31, 1, {0x00}},
  {0x40, 1, {0x16}},
  {0x41, 1, {0x55}},
  {0x42, 1, {0x03}},
  {0x43, 1, {0x8C}},
  {0x44, 1, {0x86}},
  {0x46, 1, {0x55}},
  {0x47, 1, {0x33}},
  {0x50, 1, {0x70}},
  {0x51, 1, {0x70}},
  {0x52, 1, {0x00}},
  {0x53, 1, {0x2D}},
  {0x60, 1, {0x07}},
  {0x61, 1, {0x00}},
  {0x62, 1, {0x07}},
  {0x63, 1, {0x02}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x01}},
  {0xA0, 1, {0x00}},
  {0xA1, 1, {0x0D}},
  {0xA2, 1, {0x1F}},
  {0xA3, 1, {0x10}},
  {0xA4, 1, {0x09}},
  {0xA5, 1, {0x16}},
  {0xA6, 1, {0x17}},
  {0xA7, 1, {0x06}},
  {0xA8, 1, {0x04}},
  {0xA9, 1, {0x09}},
  {0xAA, 1, {0x0A}},
  {0xAB, 1, {0x06}},
  {0xAC, 1, {0x0F}},
  {0xAD, 1, {0x2A}},
  {0xAE, 1, {0x2B}},
  {0xAF, 1, {0x00}},
  {0xC0, 1, {0x00}},
  {0xC1, 1, {0x1D}},
  {0xC2, 1, {0x1C}},
  {0xC3, 1, {0x10}},
  {0xC4, 1, {0x0A}},
  {0xC5, 1, {0x18}},
  {0xC6, 1, {0x17}},
  {0xC7, 1, {0x06}},
  {0xC8, 1, {0x04}},
  {0xC9, 1, {0x08}},
  {0xCA, 1, {0x03}},
  {0xCB, 1, {0x01}},
  {0xCC, 1, {0x08}},
  {0xCD, 1, {0x26}},
  {0xCE, 1, {0x1D}},
  {0xCF, 1, {0x00}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x06}},
  {0x00, 1, {0x21}},
  {0x01, 1, {0x06}},
  {0x02, 1, {0xA0}},
  {0x03, 1, {0x02}},
  {0x04, 1, {0x01}},
  {0x05, 1, {0x01}},
  {0x06, 1, {0x80}},
  {0x07, 1, {0x04}},
  {0x08, 1, {0x00}},
  {0x09, 1, {0x80}},
  {0x0A, 1, {0x00}},
  {0x0B, 1, {0x00}},
  {0x0C, 1, {0x2C}},
  {0x0D, 1, {0x2C}},
  {0x0E, 1, {0x1C}},
  {0x0F, 1, {0x00}},
  {0x10, 1, {0xFF}},
  {0x11, 1, {0xF0}},
  {0x12, 1, {0x00}},
  {0x13, 1, {0xC0}},
  {0x14, 1, {0x00}},
  {0x15, 1, {0xC0}},
  {0x16, 1, {0x08}},
  {0x17, 1, {0x00}},
  {0x18, 1, {0x00}},
  {0x19, 1, {0x00}},
  {0x1A, 1, {0x00}},
  {0x1B, 1, {0x00}},
  {0x1C, 1, {0x00}},
  {0x1D, 1, {0x00}},
  {0x20, 1, {0x01}},
  {0x21, 1, {0x23}},
  {0x22, 1, {0x45}},
  {0x23, 1, {0x67}},
  {0x24, 1, {0x01}},
  {0x25, 1, {0x23}},
  {0x26, 1, {0x45}},
  {0x27, 1, {0x67}},
  {0x30, 1, {0x12}},
  {0x31, 1, {0x22}},
  {0x32, 1, {0x22}},
  {0x33, 1, {0x22}},
  {0x34, 1, {0x87}},
  {0x35, 1, {0x96}},
  {0x36, 1, {0xAA}},
  {0x37, 1, {0xDB}},
  {0x38, 1, {0xCC}},
  {0x39, 1, {0xBD}},
  {0x3A, 1, {0x78}},
  {0x3B, 1, {0x69}},
  {0x3C, 1, {0x22}},
  {0x3D, 1, {0x22}},
  {0x3E, 1, {0x22}},
  {0x3F, 1, {0x22}},
  {0x40, 1, {0x22}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x07}},
  {0x18, 1, {0x1D}},
  {0x02, 1, {0x77}},
  {0x06, 1, {0x13}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x00}},
  {0x3A, 1, {0x77}},
  {0x11, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},
  {0x29, 1, {0x00}},

  {REGFLAG_END_OF_TABLE, 0x00, {}}
#else // LCM_2
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x01}},
  {0x08, 1, {0x18}},
  {0x21, 1, {0x03}},
  {0x30, 1, {0x01}},
  {0x31, 1, {0x00}},
  {0x40, 1, {0x10}},
  {0x41, 1, {0x66}},
  {0x42, 1, {0x02}},
  {0x43, 1, {0x8B}},
  {0x44, 1, {0x86}},
  {0x46, 1, {0x66}},
  {0x50, 1, {0x65}},
  {0x51, 1, {0x63}},
  {0x52, 1, {0x00}},
  {0x53, 1, {0x32}},
  {0x60, 1, {0x07}},
  {0x61, 1, {0x04}},
  {0x62, 1, {0x06}},
  {0x63, 1, {0x02}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x01}},
  {0xA0, 1, {0x08}},
  {0xA1, 1, {0x0B}},
  {0xA2, 1, {0x17}},
  {0xA3, 1, {0x10}},
  {0xA4, 1, {0x09}},
  {0xA5, 1, {0x17}},
  {0xA6, 1, {0x07}},
  {0xA7, 1, {0x07}},
  {0xA8, 1, {0x04}},
  {0xA9, 1, {0x09}},
  {0xAA, 1, {0x07}},
  {0xAB, 1, {0x05}},
  {0xAC, 1, {0x0C}},
  {0xAD, 1, {0x2A}},
  {0xAE, 1, {0x24}},
  {0xAF, 1, {0x08}},
  {0xC0, 1, {0x00}},
  {0xC1, 1, {0x0B}},
  {0xC2, 1, {0x17}},
  {0xC3, 1, {0x10}},
  {0xC4, 1, {0x09}},
  {0xC5, 1, {0x17}},
  {0xC6, 1, {0x09}},
  {0xC7, 1, {0x07}},
  {0xC8, 1, {0x04}},
  {0xC9, 1, {0x09}},
  {0xCA, 1, {0x06}},
  {0xCB, 1, {0x05}},
  {0xCC, 1, {0x0C}},
  {0xCD, 1, {0x2A}},
  {0xCE, 1, {0x24}},
  {0xCF, 1, {0x00}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x06}},
  {0x00, 1, {0x21}},
  {0x01, 1, {0x06}},
  {0x02, 1, {0xA0}},
  {0x03, 1, {0x02}},
  {0x04, 1, {0x01}},
  {0x05, 1, {0x01}},
  {0x06, 1, {0x80}},
  {0x07, 1, {0x04}},
  {0x08, 1, {0x00}},
  {0x09, 1, {0x80}},
  {0x0A, 1, {0x00}},
  {0x0B, 1, {0x00}},
  {0x0C, 1, {0x2C}},
  {0x0D, 1, {0x2C}},
  {0x0E, 1, {0x1C}},
  {0x0F, 1, {0x00}},
  {0x10, 1, {0xFF}},
  {0x11, 1, {0xF0}},
  {0x12, 1, {0x00}},
  {0x13, 1, {0xC0}},
  {0x14, 1, {0x00}},
  {0x15, 1, {0xC0}},
  {0x16, 1, {0x08}},
  {0x17, 1, {0x00}},
  {0x18, 1, {0x00}},
  {0x19, 1, {0x00}},
  {0x1A, 1, {0x00}},
  {0x1B, 1, {0x00}},
  {0x1C, 1, {0x00}},
  {0x1D, 1, {0x00}},
  {0x20, 1, {0x01}},
  {0x21, 1, {0x23}},
  {0x22, 1, {0x45}},
  {0x23, 1, {0x67}},
  {0x24, 1, {0x01}},
  {0x25, 1, {0x23}},
  {0x26, 1, {0x45}},
  {0x27, 1, {0x67}},
  {0x30, 1, {0x12}},
  {0x31, 1, {0x22}},
  {0x32, 1, {0x22}},
  {0x33, 1, {0x22}},
  {0x34, 1, {0x87}},
  {0x35, 1, {0x96}},
  {0x36, 1, {0xAA}},
  {0x37, 1, {0xDB}},
  {0x38, 1, {0xCC}},
  {0x39, 1, {0xBD}},
  {0x3A, 1, {0x78}},
  {0x3B, 1, {0x69}},
  {0x3C, 1, {0x22}},
  {0x3D, 1, {0x22}},
  {0x3E, 1, {0x22}},
  {0x3F, 1, {0x22}},
  {0x40, 1, {0x22}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x07}},
  {0x18, 1, {0x1D}},
  {0x02, 1, {0x77}},
  {0xE1, 1, {0x79}},
  {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x00}},
  {0x3A, 1, {0x77}},
  {0x11, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},
  {0x29, 1, {0x00}},
  {REGFLAG_DELAY, 50, {}},

  {REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};

static struct LCM_setting_table lcm_sleep_out_setting[] =
{
  // Sleep Out
  {0x11, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},

  // Display ON
  {0x29, 1, {0x00}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{
  // {0xFF, 5, {0xFF, 0x98, 0x06, 0x04, 0x00}},

  // Display off sequence
  {0x28, 1, {0x00}},
  {REGFLAG_DELAY, 50, {}},

  // Sleep Mode On
  {0x10, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},

  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
  unsigned int i;

  for (i = 0; i < count; i++)
  {
    unsigned cmd;
    cmd = table[i].cmd;

    switch (cmd)
    {
    case REGFLAG_DELAY:
      MDELAY(table[i].count);
      break;

    case REGFLAG_END_OF_TABLE:
      break;

    default:
      dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
      MDELAY(10);
    }
  }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
  memset(params, 0, sizeof(LCM_PARAMS));

  params->type = LCM_TYPE_DSI;
  params->width = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  params->dsi.mode = SYNC_PULSE_VDO_MODE;
  params->dsi.LANE_NUM = LCM_TWO_LANE;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
#if 0
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;

  params->dsi.vertical_sync_active = 2;
  params->dsi.vertical_backporch = 20;
  params->dsi.vertical_frontporch = 10;
  params->dsi.vertical_active_line = FRAME_HEIGHT;

  params->dsi.horizontal_sync_active = 10;
  params->dsi.horizontal_backporch = 50;
  params->dsi.horizontal_frontporch = 100;
  params->dsi.horizontal_blanking_pixel = 120;
  params->dsi.horizontal_active_pixel = FRAME_WIDTH;

  params->dsi.word_count = 1440;

  params->dsi.intermediat_buffer_num = 2;

  params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
#endif
  params->dsi.pll_select=1;

  params->dsi.pll_div1 = 1;
  params->dsi.pll_div2 = 1;
  params->dsi.fbk_sel = 1;
  params->dsi.fbk_div = 30;
#if 0
  params->dsi.noncont_clock = 1;
  params->dsi.noncont_clock_period = 2;
#endif

  // params->dsi.PLL_CLOCK = 208;
}

static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(15);
  SET_RESET_PIN(0);
  MDELAY(25);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

extern int disp_bls_set_backlight(unsigned int level);
static void lcm_suspend(void)
{
  unsigned int data_array[16];
#if defined(BUILD_LK)
  printf("%s\n", __func__);
#elif defined(BUILD_UBOOT)
  printf("%s\n", __func__);
#else
  printk(" 9806e lcm_suspend");
#endif

  disp_bls_set_backlight(0);

  data_array[0] = 0x00280500;
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(20);

  data_array[0] = 0x00100500;
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(20);

  SET_RESET_PIN(0);
  // data_array[0] = 0x014F1500;
  // dsi_set_cmdq(data_array, 1, 1);
  // MDELAY(40);
}

static void lcm_resume(void)
{
  SET_RESET_PIN(1);
  MDELAY(5);
  SET_RESET_PIN(0);
  MDELAY(15);
  SET_RESET_PIN(1);
  MDELAY(60);
  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_setbacklight(unsigned int level)
{
  unsigned int data_array[16];

#if defined(BUILD_LK)
  printf("%s, %d\n", __func__, level);
#else
  printk("lcm_setbacklight = %d\n", level);
#endif

  if (level > 255)
    level = 255;

  data_array[0] = 0x00023902;
  data_array[1] = (0x51 | (level << 8));
  dsi_set_cmdq(data_array, 2, 1);
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
  if (lcm_esd_test)
  {
    lcm_esd_test = FALSE;
    return TRUE;
  }

  /// please notice: the max return packet size is 1
  /// if you want to change it, you can refer to the following marked code
  /// but read_reg currently only support read no more than 4 bytes....
  /// if you need to read more, please let BinHan knows.
  /*
      unsigned int data_array[16];
      unsigned int max_return_size = 1;

      data_array[0]= 0x00003700 | (max_return_size << 16);

      dsi_set_cmdq(&data_array, 1, 1);
  */

  if (read_reg(0xB6) == 0x42)
  {
    return FALSE;
  }
  else
  {
    return TRUE;
  }
#endif
}

static unsigned int lcm_esd_recover(void)
{
  lcm_init();

  return TRUE;
}

#if 0 // TODO shuld be use read ic id method
static unsigned int lcm_compare_id(void)
{
  int array[4];
  char buffer[5];
  char id_high=0;
  char id_midd=0;
  char id_low=0;
  int id=0;

  //Do reset here
  SET_RESET_PIN(1);
  SET_RESET_PIN(0);
  MDELAY(25);
  SET_RESET_PIN(1);
  MDELAY(120);
  array[0]=0x00063902;
  array[1]=0x0698ffff;
  array[2]=0x00000104;
  dsi_set_cmdq(array, 3, 1);
  MDELAY(10);

  array[0]=0x00023700;//0x00023700;
  dsi_set_cmdq(array, 1, 1);
  //read_reg_v2(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.

  read_reg_v2(0x00, buffer,1);
  id_high = buffer[0]; ///////////////////////0x98

  array[0]=0x00043700;//0x00023700;
  dsi_set_cmdq(array, 1, 1);
  read_reg_v2(0x01, buffer,1);
  id_midd = buffer[0]; ///////////////////////0x06

  array[0]=0x00043700;//0x00023700;
  dsi_set_cmdq(array, 1, 1);
  read_reg_v2(0x02, buffer,1);
  id_low = buffer[0]; ////////////////////////0x04

  id = (id_midd << 8) | id_low;

#if defined(BUILD_UBOOT) || defined(BUILD_LK)
  printf("ILI9806: %s, id = 0x%x 0x%x 0x%x 0x%x 0x%x\n", __func__, id_high,id_midd,id_low,buffer[4]);//buffer[0],buffer[1],buffer[2],buffer[3],buffer[4] );
#else
  printk("ILI9806: %s, id = 0x%08x\n", __func__, id);
#endif

  return (ILI9806E_LCM_ID == id)?1:0;
}
#else
static unsigned int lcm_compare_id(void)
{
#if 0
  int data[4];
  int adcVol=0;
  int res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data , 0 );
    adcVol=data[0]*1000+data[1]*10;

#ifdef BUILD_LK
      printf("LINE=%d %s, res=%d,adcVol = %d \n", __LINE__,__func__,res,adcVol);
#else
      printk("LINE=%d %s, res=%d,adcVol = %d \n", __LINE__,__func__,res,adcVol);
#endif

  if(adcVol > MIN_VOLTAGE_TXD && adcVol < MAX_VOLTAGE_TXD)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
#endif

  // Force ID for now
  return 1;
}
#endif

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_dsi_vdo_fwvga_txd_zaw1800_lcm_drv =
{
  .name = "ili9806e_dsi_vdo_fwvga_txd_zaw1800",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params = lcm_get_params,
  .init = lcm_init,
  .suspend = lcm_suspend,
  .resume = lcm_resume,
  //.set_backlight  = lcm_setbacklight,
  //.set_pwm        = lcm_setpwm,
  //.get_pwm        = lcm_getpwm,
  .compare_id = lcm_compare_id,
  //.esd_check      = lcm_esd_check,
  //.esd_recover    = lcm_esd_recover,
};
