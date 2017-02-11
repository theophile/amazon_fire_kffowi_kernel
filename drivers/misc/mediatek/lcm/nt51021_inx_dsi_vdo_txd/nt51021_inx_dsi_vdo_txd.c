#ifndef BUILD_LK
	#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#else
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
	#include <mach/upmu_common.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  						(600)
#define FRAME_HEIGHT 						(1024)

#define REGFLAG_DELAY             				0XFE
#define REGFLAG_END_OF_TABLE      				0xFF

#define GPIO_LCD_PWR_EN                                         (GPIO44 | 0x80000000)
#define GPIO_LCD_3V3_EN                                         (GPIO83 | 0x80000000)
#define GPIO_LCD_RST						(GPIO89 | 0x80000000)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)      					(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)	        			(lcm_util.set_gpio_out((n), (v)))
#define UDELAY(n) 						(lcm_util.udelay(n))
#define MDELAY(n) 						(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE					0

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
/**********GIP setting**********/
	{0x8F,  1 , {0xA5}},
	{REGFLAG_DELAY, 20, {}},
	{0x83,  1 , {0x00}},
	{0x84,  1 , {0x00}},

	{0x85,  1 , {0x04}},
	{0x86,  1 , {0x08}},
	{0x8C,  1 , {0x8E}},

	{0xC5,  1 , {0x2B}},
	{0xC7,  1 , {0x2B}},
	{0xFD,  1 , {0x5B}},
	{0xFA,  1 , {0x14}},

/**********MIPI setting**********/
	{0x83,  1 , {0xAA}},
	{0x84,  1 , {0x11}},
	{0xA9,  1 , {0x4B}},
	{0x8F,  1 , {0x00}},

	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
																						static struct LCM_setting_table lcm_suspend_setting[] =
{
	{0x28,  0 , {}},
	{REGFLAG_DELAY, 10, {}},
	{0x10,  0 , {}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for(i = 0; i < count; i++) {
	unsigned cmd;
	cmd = table[i].cmd;
	switch (cmd) {
	case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
	case REGFLAG_END_OF_TABLE :
                break;
	default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;	//SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;//SYNC_EVENT_VDO_MODE;

	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      	= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;

	params->dsi.vertical_sync_active			= 2;
	params->dsi.vertical_backporch				= 24;
	params->dsi.vertical_frontporch				= 35;
	params->dsi.vertical_active_line			= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active			= 2;
	params->dsi.horizontal_backporch			= 59;
	params->dsi.horizontal_frontporch			= 80;
	params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

	// Bit rate calculation
	//1 Every lane speed
	params->dsi.PLL_CLOCK = 156;                       //468;//495;//440;
	// continuous clock
	params->dsi.cont_clock = 1;
	params->dsi.ssc_disable = 1;
}

static void lcd_power_en(unsigned char enabled)
{
#ifndef BUILD_LK
	printk("[NT51021][K] %s : %s\n", __func__, enabled ? "on" : "off");
#else
	printf("[NT51021][LK] %s : %s\n", __func__, enabled ? "on" : "off");
#endif

	if (enabled)
	{
		mt_set_gpio_mode(GPIO_LCD_PWR_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_PWR_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_mode(GPIO_LCD_PWR_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_PWR_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
	}
}

static void lcm_init(void)
{
#ifdef BUILD_LK
	printf("[NT51021][LK] %s\n", __func__);
	upmu_set_rg_vgp1_vosel(0x7);
	upmu_set_rg_vgp1_en(0x1);
#else
	printk("[NT51021][K] %s\n", __func__);
	hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_3300,"lcm");
#endif
	MDELAY(20);

	mt_set_gpio_mode(GPIO_LCD_RST, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST , GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(20);

	lcd_power_en(1);
	MDELAY(10);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef BUILD_LK
	printf("[NT51021]lcm_init func:LK akx123 nt51021 4th txd lcm init ok!\n");
#else
	printk("[NT51021]lcm_init func:Kernel akx123 nt51021 4th txd lcm init ok!\n");
#endif
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend_power(void)
{
#ifndef BUILD_LK
	printk("[NT51021][K] %s\n", __func__);
#else
	printf("[NT51021][LK] %s\n", __func__);
#endif
	lcd_power_en(0);
	MDELAY(20);

	mt_set_gpio_mode(GPIO_LCD_RST, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(10);

#ifdef BUILD_LK
	upmu_set_rg_vgp1_en(0x0);
#else
	hwPowerDown(MT6323_POWER_LDO_VGP1,"lcm");
#endif
	MDELAY(10);
	printk("[NT51021]lcm_suspend func:Kernel akx123 nt51021 4th txd lcm suspend ok!\n");
}

static void lcm_resume(void)
{
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	printk("[NT51021]lcm_init func:Kernel akx123 nt51021 4th txd lcm resume ok!\n");
}

static void lcm_resume_power(void)
{
#ifdef BUILD_LK
	printf("[NT51021][LK] %s\n", __func__);
	upmu_set_rg_vgp1_vosel(0x7);
	upmu_set_rg_vgp1_en(0x1);
#else
	printk("[NT51021][K] %s\n", __func__);
	hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_3300,"lcm");
#endif
	MDELAY(20);

	mt_set_gpio_mode(GPIO_LCD_RST, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST , GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(20);

	lcd_power_en(1);
	MDELAY(10);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

LCM_DRIVER nt51021_inx_dsi_vdo_txd_lcm_drv =
{
    	.name			= "nt51021_inx_dsi_vdo_txd_4th",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           	= lcm_init,
	.suspend        	= lcm_suspend,
	.suspend_power  	= lcm_suspend_power,
	.resume         	= lcm_resume,
	.resume_power		= lcm_resume_power,
	.compare_id             = lcm_compare_id,
};

