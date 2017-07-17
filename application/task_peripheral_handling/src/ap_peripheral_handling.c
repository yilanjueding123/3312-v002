#include "ap_peripheral_handling.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"
//#include "drv_l1_system.h"
#include "driver_l1.h"
#include "drv_l1_cdsp.h"

#define LED_STATUS_FLASH	1
#define LED_STATUS_BLINK	2

#define CRAZY_KEY_TEST		0		// Send key events faster than human finger can do
#define LED_OFF             0
#define LED_ON              1

//#define current_state       State_video
static INT8U led_status;	//0: nothing  1: flash	2: blink
static INT8U led_cnt;

static INT8U g_led_count;
static INT8U g_led_flicker_state;

static INT8U    led_red_flag;
static INT8U    led_green_flag;
static INT8U    led_main_flag;

static INT8U 	g_led_r_state;
static INT8U	g_led_g_state;
static INT8U    power_keyup = 1;

static INT8U KEY_DOWN_VALUE = 0;
static INT32U pressed_key = 0;
extern INT8S video_record_sts;
extern volatile  INT8U video_down_flag;
extern volatile  INT8U pic_down_flag;
INT8U  jh_batt_level = 3;

#if TV_DET_ENABLE
INT8U tv_plug_in_flag;
INT8U tv_debounce_cnt = 0;
#endif
static	INT8U tv = !TV_DET_ACTIVE;
static INT8U backlight_tmr = 0;

#if C_SCREEN_SAVER == CUSTOM_ON
INT8U auto_off_force_disable = 0;
void ap_peripheral_auto_off_force_disable_set(INT8U);
#endif
extern INT32U gStop_Sensor;

static INT8U led_flash_timerid;
static INT16U config_cnt;

//----------------------------
typedef struct
{
    INT8U byRealVal ;
    INT8U byCalVal;
} AD_MAP_t;


//----------------------------
extern void avi_adc_gsensor_data_register(void **msgq_id, INT32U *msg_id);

INT8U gsensor_lock_flag = 0;
INT8U gsensor_data[2][32] = {0};
static void   *gsensor_msgQId0 = 0;
static INT32U gsensor_msgId0 = 0;

static INT16U adc_battery_value_new, adc_battery_value_old;
static INT32U battery_stable_cnt = 0;

#define C_BATTERY_STABLE_THRESHOLD		4  // Defines threshold number that AD value is deemed stable

#if C_BATTERY_DETECT == CUSTOM_ON
static INT16U low_voltage_cnt;
static INT32U battery_value_sum = 0;
static INT8U bat_ck_cnt = 0;
#endif



#if USE_ADKEY_NO
static INT8U ad_detect_timerid;
static INT16U ad_value;
static KEYSTATUS ad_key_map[USE_ADKEY_NO + 1];

//#if KEY_TYPE == KEY_TYPE0
static INT8U  ad_line_select = 0;				//AD sample control, select line0 or line1 channel.
static INT16U adc_key_release_value_old, adc_key_release_value_new, adc_key_release_value_stable;
static INT32U key_release_stable_cnt = 0;
//#endif

#define C_RESISTOR_ACCURACY				5//josephhsieh@140418 3			// 2% accuracy
#define C_KEY_PRESS_WATERSHED			600//josephhsieh@140418 175
#define C_KEY_STABLE_THRESHOLD			4//josephhsieh@140418 3			// Defines threshold number that AD value of key is deemed stable
#define C_KEY_FAST_JUDGE_THRESHOLD 		40			// Defines threshold number that key is should be judge before it is release. 0=Disable
#define C_KEY_RELEASE_STABLE_THRESHOLD	4  // Defines threshold number that AD value is deemed stable

INT16U adc_key_value;

//static INT8U  ad_value_cnt ;
INT32U key_pressed_cnt;
INT8U fast_key_exec_flag;
INT8U normal_key_exec_flag;
INT8U long_key_exec_flag;
#endif

static INT32U key_active_cnt;
static INT8U lcd_bl_sts;
static INT8U power_off_timerid;
static INT8U usbd_detect_io_timerid;
static KEYSTATUS key_map[USE_IOKEY_NO];
static INT8U key_detect_timerid;
static INT16U adp_out_cnt;
static INT16U usbd_cnt;
#if USB_PHY_SUSPEND == 1
static INT16U phy_cnt = 0;
#endif
static INT16U adp_cnt;
INT8U  adp_status;
static INT8U battery_low_flag = 0;
INT8U  usbd_exit;
INT8U s_usbd_pin;
extern INT8U MODE_KEY_flag;
//	prototypes
void ap_peripheral_key_init(void);
void ap_peripheral_rec_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_function_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_next_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_prev_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_ok_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_sos_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_usbd_plug_out_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_pw_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_menu_key_exe(INT16U *tick_cnt_ptr);


/////////////////////////////////////////////////
void ap_peripheral_capture_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_video_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_brose_exe(INT16U *tick_cnt_ptr);

/////////////////////////////////////////////////
#if KEY_FUNTION_TYPE == SAMPLE2
void ap_peripheral_capture_key_exe(INT16U *tick_cnt_ptr);
#endif
void ap_peripheral_null_key_exe(INT16U *tick_cnt_ptr);
#if USE_ADKEY_NO
void ap_peripheral_ad_detect_init(INT8U adc_channel, void (*bat_detect_isr)(INT16U data));
void ap_peripheral_ad_check_isr(INT16U value);
#endif
extern INT8U usb_state_get(void);
extern void usb_state_set(INT8U flag);
static void init_usbstate(void)
{

    static INT8U usb_dete_cnt = 0;
    static INT8U err_cnt = 0;

    while(++err_cnt < 100)
    {
        if(sys_pwr_key1_read())
            usb_dete_cnt++;
        else
        {
            usb_dete_cnt = 0;
            break;
        }
        if(usb_dete_cnt > 3)
            break;
        OSTimeDly(2);
    }
    if(usb_dete_cnt > 3)
        usb_state_set(3);
    err_cnt = 0;


}

void ap_peripheral_init(void)
{
#if TV_DET_ENABLE
    INT32U i;
#endif

    power_off_timerid = usbd_detect_io_timerid = led_flash_timerid = 0xFF;
    key_detect_timerid = 0xFF;

    //LED IO init
    gpio_init_io(LED_BLUE, GPIO_OUTPUT);
    gpio_set_port_attribute(LED_BLUE, ATTRIBUTE_HIGH);
    gpio_write_io(LED_BLUE, DATA_HIGH);

    gpio_init_io(LED_RED, GPIO_OUTPUT);
    gpio_set_port_attribute(LED_RED, ATTRIBUTE_HIGH);
    gpio_write_io(LED_RED, DATA_LOW);

    gpio_init_io(LED_MAIN, GPIO_OUTPUT);
    gpio_set_port_attribute(LED_MAIN, ATTRIBUTE_HIGH);
    gpio_write_io(LED_MAIN, DATA_HIGH);

    gpio_init_io(LED_MAIN1, GPIO_OUTPUT);
    gpio_set_port_attribute(LED_MAIN1, ATTRIBUTE_HIGH);
    gpio_write_io(LED_MAIN1, DATA_HIGH);

    gpio_init_io(BATT_CHARGE, GPIO_INPUT);
    gpio_set_port_attribute(BATT_CHARGE, ATTRIBUTE_LOW);
    gpio_write_io(BATT_CHARGE, 1);	//pull high
    init_usbstate();

    led_status = 0;
    led_cnt = 32;
    led_red_flag  = 0;
    led_green_flag = 0;
    led_main_flag = 0;

    gpio_init_io(AV_IN_DET, GPIO_INPUT);
    gpio_set_port_attribute(AV_IN_DET, ATTRIBUTE_LOW);
    gpio_write_io(AV_IN_DET, !TV_DET_ACTIVE);	//pull high or low

#if TV_DET_ENABLE
    tv_plug_in_flag = 0;
    for(i = 0; i < 5; i++)
    {
        if(gpio_read_io(AV_IN_DET) == !TV_DET_ACTIVE)
        {
            break;
        }
        OSTimeDly(1);
    }

    if(i == 5)
    {
        tv = TV_DET_ACTIVE;
        tv_plug_in_flag = 1;
    }
#endif

    gpio_init_io(HDMI_IN_DET, GPIO_INPUT);
    gpio_set_port_attribute(HDMI_IN_DET, ATTRIBUTE_LOW);
    gpio_write_io(HDMI_IN_DET, 0);	//pull low

    gpio_init_io(HDMI_EN, GPIO_OUTPUT);
    gpio_set_port_attribute(HDMI_EN, ATTRIBUTE_HIGH);
    gpio_write_io(HDMI_EN, 1);

#if TV_DET_ENABLE
    if(tv_plug_in_flag)
    {
        gpio_write_io(SPEAKER_EN, 0); //mute local speaker
    }
    else
#endif
    {
        gpio_write_io(SPEAKER_EN, 1); //enable local speaker
    }

    ap_peripheral_key_init();

#if USE_ADKEY_NO
    ad_detect_timerid = 0xFF;
    ap_peripheral_ad_detect_init(AD_KEY_DETECT_PIN, ap_peripheral_ad_check_isr);
#else
    adc_init();
#endif

    config_cnt = 0;
    MODE_KEY_flag = 2;
    gsensor_lock_flag = 0;
}

#if 0 //#ifdef PWM_CTR_LED 
void ap_peripheral_PWM_OFF(void)
{
    INT8U byPole = 0;
    INT16U wPeriod = 0;
    INT16U wPreload = 0;
    INT8U byEnable = 0;
    ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable) ;
    //	ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable);
    DBG_PRINT("PWM0/1 off!\r\n");

}


void ap_peripheral_PWM_LED_high(void)
{
#if 0
    INT8U byPole = 1;
    INT16U wPeriod = 0x6000;
    INT16U wPreload = 0x5fff;
    INT8U byEnable = 0;
    ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable) ;
    //	ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable);
    DBG_PRINT("PWM0/1 OUT PUT HIGH 750ms, low 68us\r\n");
#endif

}

void ap_peripheral_PWM_LED_low(void)
{
#if 0
    INT8U byPole = 1;
    INT16U wPeriod = 0x6000;
    INT16U wPreload = 0x1;
    INT8U byEnable = TRUE;
    ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable) ;
    //    ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable) ;
    //    DBG_PRINT("PWM0/1 OUT PUT LOW 750ms, high 68us\r\n");
#endif
}

#endif

void ap_peripheral_led_set(LED_MODE_ENUM type)
{

    g_led_g_state = 0;
    g_led_r_state = 0;

    switch((INT32U)type)
    {
    case LED_INIT:
        led_green_on();
        //led_red_off();
        DBG_PRINT("LED_TYPE=LED_INIT\r\n");
        break;
    case LED_WAITING_RECORD:
        led_green_on();
        //	led_red_off();
        DBG_PRINT("LED_TYPE=LED_WAITING_RECORD\r\n");
        break;
    case LED_RECORD:
        //	led_green_on();
        g_led_g_state = 1;

        DBG_PRINT("LED_TYPE=LED_RECORD\r\n");
        break;
    case LED_WAITING_CAPTURE:
        led_green_on();
        //led_red_off();

        DBG_PRINT("LED_TYPE=LED_WAITING_CAPTURE\r\n");
        break;
    case LED_CAPTURE:
        //	led_red_off();

        DBG_PRINT("LED_TYPE=LED_CAPTURE\r\n");
        break;
    case LED_USB_CONNECT:
        //led_green_on();
        //led_red_on();
        DBG_PRINT("LED_TYPE=LED_USB_CONNECT\r\n");
    case LED_USB_DISCONNECT:
        //led_red_off();

        DBG_PRINT("LED_TYPE=LED_USB_DISCONNECT\r\n");
        break;
        break;

    }

#if 0
#ifdef PWM_CTR_LED
    INT8U byPole;
    INT16U wPeriod = 0;
    INT16U wPreload = 0;
    INT8U byEnable;
    if(type) 		//high
    {
        ap_peripheral_PWM_LED_high();
    }
    else 			//low
    {
        ap_peripheral_PWM_LED_low();
    }

#else
    gpio_write_io(LED, type);
    led_status = 0;
    led_cnt = 0;
#endif
#endif
}

void ap_peripheral_led_flash_set(void)
{
#if 0
#ifdef PWM_CTR_LED
    ap_peripheral_PWM_LED_high();
    led_status = LED_STATUS_FLASH;
    led_cnt = 0;

#else
    gpio_write_io(LED, DATA_HIGH);
    led_status = LED_STATUS_FLASH;
    led_cnt = 0;
#endif
#endif
}
void ap_peripheral_led_blink_set(void)
{
#if 0
#ifdef PWM_CTR_LED
    INT8U byPole = 1;
    INT16U wPeriod = 0x6000 ;
    INT16U wPreload = 0x2fff ;
    INT8U byEnable = TRUE;
    ext_rtc_pwm0_enable(byPole, wPeriod, wPreload, byEnable ) ;
    //    ext_rtc_pwm1_enable(byPole, wPeriod, wPreload, byEnable) ;
    DBG_PRINT("PWM0/1 blink on  750ms,380ms \r\n");

#else
    gpio_write_io(LED, DATA_HIGH);
    led_status = LED_STATUS_BLINK;
    led_cnt = 0;
#endif
#endif
}
void led_red_on(void)
{
    if(led_red_flag != LED_ON)
    {
        gpio_write_io(LED_RED, DATA_HIGH);
        led_red_flag = LED_ON;
    }
}

void led_green_on(void)
{
    if(led_green_flag != LED_ON)
    {
        gpio_write_io(LED_BLUE, DATA_HIGH);
        led_green_flag = LED_ON;
    }
}
void led_main_on(void)
{

    if(led_main_flag != LED_ON)
    {
        gpio_write_io(LED_MAIN, DATA_LOW);
        gpio_write_io(LED_MAIN1, DATA_LOW);
        led_main_flag = LED_ON;
    }
}


void led_main_off(void)
{
    if(led_main_flag != LED_OFF)
    {
        gpio_write_io(LED_MAIN, DATA_HIGH);
        gpio_write_io(LED_MAIN1, DATA_HIGH);
        led_main_flag = LED_OFF;
    }
}
void led_green_off(void)
{
    if(led_green_flag != LED_OFF)
    {
        gpio_write_io(LED_BLUE, DATA_LOW);
        led_green_flag = LED_OFF;

    }
}

void led_red_off(void)
{
    if(led_red_flag != LED_OFF)
    {
        gpio_write_io(LED_RED, DATA_LOW);
        led_red_flag = LED_OFF;
    }
}
void led_all_off(void)
{
    //if(led_green_flag != LED_OFF)
    //   {
    //	   gpio_write_io(LED_BLUE, DATA_LOW);
    //   led_green_flag=LED_OFF;
    //  }
    //  if(led_red_flag != LED_OFF)
    //  {
    //   gpio_write_io(LED_RED, DATA_LOW);
    //   led_red_flag=LED_OFF;
    //  }
    led_main_off();
    led_green_off();
    led_red_off();

}
extern INT8U video_stop_flag;
//extern INT8U usb_state_get(void);

void LED_service(void)
{
    static INT8U flag = 0;
    g_led_count++;
    if(g_led_count > 31)
    {
        g_led_count = 0;
    }
    if(g_led_g_state)
    {
        if(g_led_count / 16 == 1)
            led_green_on();
        else
            led_green_off();
    }

    if(g_led_r_state)
    {
        if(g_led_count / 16 == 1)
            led_red_on();
        else
            led_red_off();
    }

    if(usb_state_get())
        led_red_on();
    else
        led_red_off();
}


#if C_MOTION_DETECTION == CUSTOM_ON

void ap_peripheral_motion_detect_judge(void)
{
    INT32U result;

    result = hwCdsp_MD_get_result();
    //DBG_PRINT("MD_result = 0x%x\r\n",result);
    if(result > 0x40)
    {
        msgQSend(ApQ, MSG_APQ_MOTION_DETECT_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
    }
}

void ap_peripheral_motion_detect_start(void)
{
    motion_detect_status_set(MOTION_DETECT_STATUS_START);
}

void ap_peripheral_motion_detect_stop(void)
{
    motion_detect_status_set(MOTION_DETECT_STATUS_STOP);
}
#endif

#if USE_ADKEY_NO
void ap_peripheral_ad_detect_init(INT8U adc_channel, void (*ad_detect_isr)(INT16U data))
{

    battery_value_sum = 0;
    bat_ck_cnt = 0;

    adc_init();
    adc_vref_enable_set(TRUE);
    adc_conv_time_sel(4);
    adc_manual_ch_set(adc_channel);
    adc_manual_callback_set(ad_detect_isr);
    if (ad_detect_timerid == 0xFF)
    {
        ad_detect_timerid = AD_DETECT_TIMER_ID;
        sys_set_timer((void *)msgQSend, (void *) PeripheralTaskQ, MSG_PERIPHERAL_TASK_AD_DETECT_CHECK, ad_detect_timerid, PERI_TIME_INTERVAL_AD_DETECT);
    }
}

void ap_peripheral_ad_check_isr(INT16U value)
{
    ad_value = value;
}

INT16U adc_key_release_calibration(INT16U value)
{
    return value;
}

void ap_peripheral_clr_screen_saver_timer(void)
{
    key_active_cnt = 0;
}

#if  0//(KEY_TYPE == KEY_TYPE1)||(KEY_TYPE==KEY_TYPE2)||(KEY_TYPE==KEY_TYPE3)||(KEY_TYPE==KEY_TYPE4)||(KEY_TYPE==KEY_TYPE5)

/*
       1/10 的分壓如下
	4.1v => 2460
	3.9v => 2364
	3.8v => 2319
	3.7v =>
	3.6v => 2200
	3.5v =>
	3.4v => 2079
	外部線路是 1/2 分壓
*/

enum
{
    BATTERY_CNT = 8,
    BATTERY_Lv3 = 2460 * BATTERY_CNT,
    BATTERY_Lv2 = 2319 * BATTERY_CNT,
    BATTERY_Lv1 = 2200 * BATTERY_CNT
};

enum
{
    DEBUNCE_TIME = 3,
    LONG_KEY_TIME = 35
};

enum
{
    C_KEY_NULL = 0,
    C_KEY_KEY1 = 1,
    C_KEY_KEY2 = 2,
    C_KEY_KEY3 = 3,
    C_KEY_KEY4 = 4,
    C_KEY_KEY5 = 5,
    C_KEY_KEY6 = 6,
    C_KEY_HOLD = 0x80
};

static short Median_Filter(short adc_value)
{
    static short D1 = 0;
    static short D2 = 0;
    short max, mid, min;

    if (D2 > D1)
    {
        max = D2;
        min = D1;
    }
    else
    {
        max = D1;
        min = D2;
    }

    if (adc_value > max)
    {
        mid = max;
    }
    else
    {
        mid = adc_value;
    }

    if (mid < min)
    {
        mid = min;
    }

    D2 = D1;
    D1 = adc_value;
    return mid;
}
#if  (KEY_TYPE == KEY_TYPE1)
static char AD_Key_Select(short adc_value)
{
    char key;

    adc_value = Median_Filter(adc_value);
    //  DBG_PRINT("Filter_key = %d \r\n",adc_value);
    /*  if (adc_value>800)
      {
         key = C_KEY_KEY6;
      }
      else*/
    if (adc_value > 2000)
    {
        //
        key = C_KEY_KEY5;
    }
    else if (adc_value > 1496)
    {
        //
        key = C_KEY_KEY4;
    }
    else if (adc_value > 974)
    {
        //
        key = C_KEY_KEY3;
    }
    else if (adc_value > 487)
    {
        //
        key = C_KEY_KEY2;
    }
    else if (adc_value > 129)
    {
        //
        key = C_KEY_KEY1;
    }
    else
    {
        key = C_KEY_NULL;
    }

    return key;
}
#elif  (KEY_TYPE == KEY_TYPE2)
static char AD_Key_Select(short adc_value)
{
    char key;

    adc_value = Median_Filter(adc_value);
    // DBG_PRINT("Filter_key = %d \r\n",adc_value);
    if (adc_value > 1650)
    {
        key = C_KEY_KEY6;
    }
    else if (adc_value > 1150)
    {
        //
        key = C_KEY_KEY5;
    }
    else if (adc_value > 960)
    {
        //
        key = C_KEY_KEY4;
    }
    else if (adc_value > 560)
    {
        //
        key = C_KEY_KEY3;
    }
    else if (adc_value > 420)
    {
        //
        key = C_KEY_KEY2;
    }
    else if (adc_value > 200)
    {
        //
        key = C_KEY_KEY1;
    }
    else
    {
        key = C_KEY_NULL;
    }
    return key;
}
#elif  (KEY_TYPE == KEY_TYPE4)
static char AD_Key_Select(short adc_value)
{
    char key;

    adc_value = Median_Filter(adc_value);
    // DBG_PRINT("Filter_key = %d \r\n",adc_value);
    if (adc_value > 1850) //2200
    {
        key = C_KEY_KEY6;
    }
    else if (adc_value > 1600) //1900
    {
        //
        key = C_KEY_KEY5;
    }
    else if (adc_value > 1100) //1400
    {
        //
        key = C_KEY_KEY4;
    }
    else if (adc_value > 900) //1000
    {
        //
        key = C_KEY_KEY3;
    }
    else if (adc_value > 420) //500
    {
        //
        key = C_KEY_KEY2;
    }
    else if (adc_value > 200)
    {
        //
        key = C_KEY_KEY1;
    }
    else
    {
        key = C_KEY_NULL;
    }

    return key;
}
#elif  (KEY_TYPE == KEY_TYPE5)
static char AD_Key_Select(short adc_value)
{
    char key;

    adc_value = Median_Filter(adc_value);
    // DBG_PRINT("Filter_key = %d \r\n",adc_value);
    if (adc_value > 1850) //2200
    {
        key = C_KEY_KEY6;
    }
    else if (adc_value > 1600) //1900
    {
        //
        key = C_KEY_KEY5;
    }
    else if (adc_value > 1100) //1400
    {
        //
        key = C_KEY_KEY4;
    }
    else if (adc_value > 900) //1000
    {
        //
        key = C_KEY_KEY3;
    }
    else if (adc_value > 420) //500
    {
        //
        key = C_KEY_KEY2;
    }
    else if (adc_value > 200)
    {
        //
        key = C_KEY_KEY1;
    }
    else
    {
        key = C_KEY_NULL;
    }

    return key;
}
#else
static char AD_Key_Select(short adc_value)
{
    char key;

    adc_value = Median_Filter(adc_value);
    // DBG_PRINT("Filter_key = %d \r\n",adc_value);
    if (adc_value > 0x893)
    {
        key = C_KEY_KEY5;
    }
    else if (adc_value > 0x655)
    {
        //
        key = C_KEY_KEY4;
    }
    else if (adc_value > 0x371)
    {
        //
        key = C_KEY_KEY3;
    }
    else if (adc_value > 0x1A5)
    {
        //
        key = C_KEY_KEY2;
    }
    else if (adc_value > 0xD2)
    {
        //
        key = C_KEY_KEY1;
    }
    else
    {
        //
        key = C_KEY_NULL;
    }

    return key;
}
#endif

static char AD_Key_Get(short adc_value)
{
    static char gc_KeyValue = C_KEY_NULL;
    static char gc_Key_Pressed = C_KEY_NULL;
    static char gc_Key_PrevPressed = C_KEY_NULL;
    static char gc_KeyFinalValue = C_KEY_NULL;
    static char gb_KeyStart = 0;
    static short gw_KeyTimeDebunce = DEBUNCE_TIME;
    static short gw_LongKeyCount = 0;
    static short gw_LongKeyCnt = LONG_KEY_TIME;
    char ret = C_KEY_NULL;

    ////////////////////////////////////////////////////////////
    //          Key Selection Algorithm (ML decision)
    ///////////////////////////////////////////////////////////

    gc_Key_Pressed = AD_Key_Select(adc_value);

    ////////////////////////////////////////////////////////////
    //          Key Detection Algorithm
    ///////////////////////////////////////////////////////////

    // key estimation
    if((gc_KeyFinalValue < gc_Key_Pressed) && (gc_Key_Pressed != C_KEY_NULL))
    {
        gc_KeyFinalValue = gc_Key_Pressed;
    }

    // debunce or discard current key
    if(gb_KeyStart && (gc_Key_PrevPressed != C_KEY_NULL))
    {
        if(gw_KeyTimeDebunce)
            gw_KeyTimeDebunce--;
        if(gw_KeyTimeDebunce && (gc_Key_PrevPressed != gc_Key_Pressed))
        {
            // 在 Debunce 的過程中，上次不等於這次，一切都白費了
            gc_Key_PrevPressed = C_KEY_NULL;
            gb_KeyStart = 0;
        }
    }

    ///////// key detect start  /////////
    if ((gc_Key_PrevPressed == C_KEY_NULL) && (gc_Key_Pressed != C_KEY_NULL))		// 新按鍵
    {
        // 1. 完全沒有按鍵、剛按鍵
        // 2. Debunce 過程中放棄前一鍵，重新計數這一鍵
        gc_Key_PrevPressed = gc_Key_Pressed;
        gw_KeyTimeDebunce = DEBUNCE_TIME;
        gc_KeyFinalValue = C_KEY_NULL;
        gb_KeyStart = 1;
        gw_LongKeyCount = 0;
    }

    ///////// key detect end  /////////
    if (gw_KeyTimeDebunce == 0)
    {
        if(gc_Key_PrevPressed  != gc_KeyFinalValue)
        {
            // 平穩段與預測值不同,丟掉Key
            gc_Key_PrevPressed = gc_Key_Pressed;
            gw_KeyTimeDebunce = DEBUNCE_TIME;
            gc_KeyFinalValue = C_KEY_NULL;
            gw_LongKeyCount = 0;
        }
        else if ((gc_Key_Pressed == C_KEY_NULL) && (gc_Key_PrevPressed != C_KEY_NULL) )
        {
            gc_Key_PrevPressed = gc_Key_Pressed;
            gw_KeyTimeDebunce = DEBUNCE_TIME;
            gc_KeyFinalValue = C_KEY_NULL;
            gw_LongKeyCount = 0;
        }
        else if ( (gc_Key_Pressed == gc_Key_PrevPressed) && (gw_LongKeyCount < gw_LongKeyCnt) ) // ?僇墿偌
        {
            gw_LongKeyCount++;
            gc_KeyValue = gc_KeyFinalValue;
            if(gw_LongKeyCount >= gw_LongKeyCnt)   // Detect Long Key
            {
                ad_key_map[gc_KeyValue].key_cnt = 100;	// 長按（已經執行過了）
                ad_key_map[gc_KeyValue].key_function(&(ad_key_map[gc_KeyValue].key_cnt)); // 呼叫按鍵功能

                gc_KeyValue = gc_KeyFinalValue | C_KEY_HOLD;	// Long Key pressed
            }
        }
        //If user hold long key contiueously, it will do nothing.}
    }

    // release key
    if ((gc_KeyValue != C_KEY_NULL) && (gc_Key_Pressed == C_KEY_NULL))
    {
        ret = gc_KeyValue;
        gw_LongKeyCount = 0;
        gw_KeyTimeDebunce = DEBUNCE_TIME;
        gc_KeyValue = C_KEY_NULL;
    }

    return ret;
}

#define BATTERY_PEROID 0xF
static unsigned int battery_period_cnt = 0;
void ap_peripheral_ad_key_judge(void)
{
    char key;

    if ((battery_period_cnt & BATTERY_PEROID) == 0)
    {
        unsigned int diff;
        //DBG_PRINT("bat=0x%x\r\n",ad_value>>4);
        // Battery Detect
        adc_battery_value_old = adc_battery_value_new;
        adc_battery_value_new = ad_value >> 4;

        if (adc_battery_value_new >= adc_battery_value_old)
        {
            diff = adc_battery_value_new - adc_battery_value_old;
        }
        else
        {
            diff = adc_battery_value_old - adc_battery_value_new;
        }
        if (!diff || (100 * diff <= C_RESISTOR_ACCURACY * adc_battery_value_old))
        {
            if (battery_stable_cnt < C_BATTERY_STABLE_THRESHOLD) battery_stable_cnt++;
        }
        else
        {
            battery_stable_cnt = 1;

            bat_ck_cnt = 0;
            battery_value_sum = 0;
        }

        if (battery_stable_cnt >= C_BATTERY_STABLE_THRESHOLD)
        {

#if C_BATTERY_DETECT == CUSTOM_ON
            ap_peripheral_battery_check_calculate();
#endif

        }
    }
    else
    {
        // AD-Key  Detect
        //DBG_PRINT("adc=0x%x\r\n",ad_value>>4);
        key = AD_Key_Get(ad_value >> 4);
        if (key != C_KEY_NULL)
        {

#if C_SCREEN_SAVER == CUSTOM_ON
#if TV_DET_ENABLE
            if (!tv_plug_in_flag)
#endif
            {
                key_active_cnt = 0;
                ap_peripheral_lcd_backlight_set(BL_ON);
            }
#endif

            //DBG_PRINT("key = 0x%x\r\n",(unsigned char)key);
            switch(key)
            {
            case 1:
                DBG_PRINT("PREV\r\n");
                break;
            case 2:
                DBG_PRINT("NEXT\r\n");
                break;
            case 3:
                DBG_PRINT("FUNC\r\n");
                break;
            case 4:
                DBG_PRINT("MENU\r\n");
                break;
            case 5:
                DBG_PRINT("OK\r\n");
                break;
            default:
                DBG_PRINT("Wrong Key\r\n");
            }

            if (key & C_KEY_HOLD)
            {
                //key &= (~C_KEY_HOLD);
                //ad_key_map[key].key_cnt = 100;	// 長按（已經執行過了）
                //ad_key_map[key].key_function(&(ad_key_map[key].key_cnt)); // 呼叫按鍵功能
                __asm {NOP};
            }
            else
            {
                ad_key_map[key].key_cnt = 3;		// 短按
                ad_key_map[key].key_function(&(ad_key_map[key].key_cnt)); // 呼叫按鍵功能
            }
        }
    }

    //DBG_PRINT("cnt=0x%x\r\n",battery_period_cnt);
    battery_period_cnt++;
    // next SAE ADC manual mode
    if ((battery_period_cnt & BATTERY_PEROID) == 0)
    {
        adc_manual_ch_set(AD_BAT_DETECT_PIN);
    }
    else
    {
        adc_manual_ch_set(AD_KEY_DETECT_PIN);
    }
    adc_manual_sample_start();
}

#else

/*
	0.41v => 495
	0.39v =>
	0.38v => 460
	0.37v =>
	0.36v => 440
	0.35v =>
	0.34v =>
*/

enum
{
    BATTERY_CNT = 16,
    BATTERY_Lv3 = 2530 * BATTERY_CNT, //4v
    BATTERY_Lv2 = 2380 * BATTERY_CNT, //3.8v
    BATTERY_Lv1 = 2224 * BATTERY_CNT, //3.55v
    BATTERY_Lv0 = 2175 * BATTERY_CNT
};

static INT32U adc_key_factor_table[USE_ADKEY_NO] =  	// x1000
{
    // 6 AD-keys
    //680K, 300K, 150K, 68K, 39K, 22K
    ///*1969, 2933, 4182, 5924, */7104, 8102
    (900), (600), (380), (190), (80)
};
static INT32U ad_time_stamp;

INT32U adc_key_judge(INT32U adc_value)
{
    INT32U candidate_key;
    INT32S i;
    for (i = USE_ADKEY_NO - 1; i >= 0; i--)
    {
        if(adc_value <= adc_key_factor_table[i])
        {
            candidate_key = i;
            break;
        }
    }
    // DBG_PRINT("key=%d\r\n",candidate_key);
#if C_SCREEN_SAVER == CUSTOM_ON
    key_active_cnt = 0;
    ap_peripheral_lcd_backlight_set(BL_ON);
#endif
    return candidate_key;
}


void ap_peripheral_ad_key_judge(void)
{
    INT32U  t;
    INT32U diff;
    INT16U adc_current_value;
    static INT8U cnt = 0;
    TIME_T tm;
    static INT8U now_sec;

    t = OSTimeGet();
    if ((t - ad_time_stamp) < 2)
    {
        return;
    }
    ad_time_stamp = t;

    ad_line_select++;
    if (ad_line_select & 0x01)
    {
        adc_manual_ch_set(AD_KEY_DETECT_PIN);
    }
    else
    {
        adc_manual_ch_set(AD_BAT_DETECT_PIN);
        ad_line_select = 0;
    }

    if(ad_line_select == 1)
    {
#if C_BATTERY_DETECT == CUSTOM_ON
        ap_peripheral_battery_check_calculate();
#endif
    }
    else
    {
        adc_current_value = (ad_value >> 4);
        // DBG_PRINT("key_value=%d\r\n",adc_current_value);
        if(adc_current_value < 10)
        {
            key_pressed_cnt = 0;
            if(KEY_DOWN_VALUE)
            {
                if( ad_key_map[pressed_key].long_flag)
            	{
	                ad_key_map[pressed_key].long_flag = 0;
            	}
                else if(ad_key_map[pressed_key].key_cnt >= 2)
                {
                    if(power_keyup == 0)
                	{
	                    ad_key_map[pressed_key].key_function(&(ad_key_map[pressed_key].key_cnt));
                	}
                }

                ad_key_map[pressed_key].key_cnt = 0;
                KEY_DOWN_VALUE = 0;
            }
        }
        else if(adc_current_value <= 900)
        {
            set_pa_cnt(128);
            //DBG_PRINT("key_value=%d\r\n",adc_current_value);
            if(adc_current_value >= adc_key_value){
                diff = adc_current_value - adc_key_value;
            }
            else{
                diff = adc_key_value - adc_current_value;
            }
            if(diff < 20 )
            {
                if(key_pressed_cnt < 65535)
            	{
	                key_pressed_cnt ++;
            	}
            }
            else{
                key_pressed_cnt = 0;
            }
            if(key_pressed_cnt >= 2)			//eliminate shake handle.
            {
                if(!KEY_DOWN_VALUE)				
                {
                    pressed_key = adc_key_judge(adc_key_value);
                    KEY_DOWN_VALUE = 1;
                }
                ad_key_map[pressed_key].key_cnt = key_pressed_cnt;
                if(!(ad_key_map[pressed_key].long_flag))
                {
                    if(ad_key_map[pressed_key].key_cnt >= 32)
                    {
                        ad_key_map[pressed_key].long_flag = 1;
                        if(power_keyup == 0){
                            ad_key_map[pressed_key].key_function(&(ad_key_map[pressed_key].key_cnt));
                        }
                    }
                }
            }
            adc_key_value = adc_current_value;
        }
    }
    adc_manual_sample_start();
}
#endif // AD-Key
#endif

#if C_BATTERY_DETECT == CUSTOM_ON

INT32U previous_direction = 0;
extern void ap_state_handling_led_off(void);
extern INT8U display_str_battery_low;


#define BATTERY_GAP 10*BATTERY_CNT
static INT8U ap_peripheral_smith_trigger_battery_level(INT32U direction)
{
    static INT8U bat_lvl_cal_bak = (INT8U)BATTERY_Lv3;
    INT8U bat_lvl_cal;
    //////////////////fankun
    //	bat_lvl_cal=3;
    //	return;

    // DBG_PRINT("(%d)\r\n", battery_value_sum);
    if (battery_value_sum >= BATTERY_Lv3)
    {
        bat_lvl_cal = 3;
    }
    else if ((battery_value_sum < BATTERY_Lv3) && (battery_value_sum >= BATTERY_Lv2))
    {
        bat_lvl_cal = 2;
    }
    else if ((battery_value_sum < BATTERY_Lv2) && (battery_value_sum >= BATTERY_Lv1))
    {
        bat_lvl_cal = 1;
    }
    else if (battery_value_sum < BATTERY_Lv1)
    {
        bat_lvl_cal = 0;
    }


    if  ( (direction == 0) && (bat_lvl_cal > bat_lvl_cal_bak) )
    {
        if (battery_value_sum >= BATTERY_Lv3 + BATTERY_GAP)
        {
            bat_lvl_cal = 3;
        }
        else if ((battery_value_sum < BATTERY_Lv3 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 + BATTERY_GAP))
        {
            bat_lvl_cal = 2;
        }
        else if ((battery_value_sum < BATTERY_Lv2 + BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 + BATTERY_GAP))
        {
            bat_lvl_cal = 1;
        }
        else if (battery_value_sum < BATTERY_Lv1 + BATTERY_GAP)
        {
            bat_lvl_cal = 0;
        }
    }


    if  ( (direction == 1) && (bat_lvl_cal < bat_lvl_cal_bak) )
    {
        if (battery_value_sum >= BATTERY_Lv3 - BATTERY_GAP)
        {
            bat_lvl_cal = 3;
        }
        else if ((battery_value_sum < BATTERY_Lv3 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv2 - BATTERY_GAP))
        {
            bat_lvl_cal = 2;
        }
        else if ((battery_value_sum < BATTERY_Lv2 - BATTERY_GAP) && (battery_value_sum >= BATTERY_Lv1 - BATTERY_GAP))
        {
            bat_lvl_cal = 1;
        }
        else if (battery_value_sum < BATTERY_Lv1 - BATTERY_GAP)
        {
            bat_lvl_cal = 0;
        }
    }


    bat_lvl_cal_bak = bat_lvl_cal;
    jh_batt_level = bat_lvl_cal;
    return bat_lvl_cal;

}


void ap_peripheral_battery_check_calculate(void)
{
    INT8U bat_lvl_cal;
    INT32U direction = 0;

    if (adp_status == 0)
    {
        //unkown state
        return;
    }
    else if (adp_status == 1)
    {
        //adaptor in state
        direction = 1; //low voltage to high voltage
        if(previous_direction != direction)
        {
            msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_SHOW, NULL, NULL, MSG_PRI_NORMAL);
        }
        previous_direction = direction;
    }
    else
    {
        //adaptor out state
        direction = 0; //high voltage to low voltage
        if(previous_direction != direction)
        {
            msgQSend(ApQ, MSG_APQ_BATTERY_CHARGED_CLEAR, NULL, NULL, MSG_PRI_NORMAL);
        }
        previous_direction = direction;
    }

    battery_value_sum += (ad_value >> 4);
    //DBG_PRINT("%d, ",(ad_value>>4));
    bat_ck_cnt++;
    if (bat_ck_cnt >= BATTERY_CNT)
    {

        bat_lvl_cal = ap_peripheral_smith_trigger_battery_level(direction);
        DBG_PRINT("%d,", bat_lvl_cal);

        if(!battery_low_flag)
        {
            msgQSend(ApQ, MSG_APQ_BATTERY_LVL_SHOW, &bat_lvl_cal, sizeof(INT8U), MSG_PRI_NORMAL);
        }

        if (bat_lvl_cal == 0 && direction == 0)
        {

            low_voltage_cnt++;
            if (low_voltage_cnt > 3)
            {
                low_voltage_cnt = 0;
                if(battery_value_sum > (BATTERY_Lv0 - BATTERY_GAP))
                {
                    if ((!pic_down_flag) && (!video_down_flag))
                    {
                        if (video_record_sts & 0x02)
                        {

                            msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
                        }
                    }
                    bat_ck_cnt = 0;
                    battery_value_sum = 0;
                    return;
                }

                ap_state_handling_led_off();
#if C_BATTERY_LOW_POWER_OFF == CUSTOM_ON
                if(!battery_low_flag)
                {
                    battery_low_flag = 1;
                    {
                        INT8U type;
                        msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
                        type = FALSE;
                        msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_FREESIZE_CHECK_SWITCH, &type, sizeof(INT8U), MSG_PRI_URGENT);
                        type = BETTERY_LOW_STATUS_KEY;
                        msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
                        msgQSend(ApQ, MSG_APQ_BATTERY_LOW_SHOW, NULL, sizeof(INT8U), MSG_PRI_NORMAL);
                    }
                }
#endif
                //OSTimeDly(100);
                //display_str_battery_low = 1;
                //msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            }
        }
        else
        {
            if(battery_low_flag)
            {
                INT8U type;
                battery_low_flag = 0;
                msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_START, NULL, NULL, MSG_PRI_NORMAL);
                type = GENERAL_KEY;
                msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
            }
            low_voltage_cnt = 0;
        }

        bat_ck_cnt = 0;
        battery_value_sum = 0;
    }
}

#endif




#if C_SCREEN_SAVER == CUSTOM_ON

void ap_peripheral_auto_off_force_disable_set(INT8U auto_off_disable)
{
    auto_off_force_disable = auto_off_disable;
}

void ap_peripheral_lcd_backlight_set(INT8U type)
{
    if (type == BL_ON)
    {
        if (lcd_bl_sts)
        {
            lcd_bl_sts = 0;
            DBG_PRINT("LCD ON\r\n");
            tft_backlight_en_set(TRUE);
        }
    }
    else
    {
        if (!lcd_bl_sts)
        {
            lcd_bl_sts = 1;
            DBG_PRINT("LCD OFF\r\n");
            tft_backlight_en_set(FALSE);
        }
    }
}
#endif

void ap_peripheral_night_mode_set(INT8U type)
{
    if(type)
    {
        gpio_write_io(IR_CTRL, 1);
    }
    else
    {
        gpio_write_io(IR_CTRL, 0);
    }
}

void ap_peripheral_key_init(void)
{
#if 1
    INT32U i;

    gp_memset((INT8S *) &key_map, NULL, sizeof(KEYSTATUS));
    ap_peripheral_key_register(GENERAL_KEY);
#if 0
    for (i = 0 ; i < USE_IOKEY_NO ; i++)
    {
        if (key_map[i].key_io)
        {
            key_map[i].key_cnt = 0;
            key_map[i].long_flag = 0;
            gpio_init_io(key_map[i].key_io, GPIO_INPUT);
            gpio_set_port_attribute(key_map[i].key_io, ATTRIBUTE_LOW);
            gpio_write_io(key_map[i].key_io, KEY_ACTIVE ^ 1);
            DBG_PRINT("INIT\r\n");
        }
    }

    for (i = 0 ; i < USE_IOKEY_NO ; i++)
    {
        if (key_map[i].key_io == PWR_KEY0)
        {
            while (sys_pwr_key0_read())
            {
                OSTimeDly(5);
            }
        }
    }
#endif
#if 0
    for (i = 0 ; i < USE_IOKEY_NO ; i++)
    {
        if (key_map[i].key_io == PWR_KEY0)
        {
            while (sys_pwr_key0_read())
            {
                OSTimeDly(5);
            }
        }
    }
#endif
#endif
}

void ap_peripheral_key_register(INT8U type)
{
    INT32U i;

    if (type == GENERAL_KEY)
    {
        DBG_PRINT("GENERAL_KEY_SET\r\n");

        key_map[0].key_io = PW_KEY;
        key_map[0].key_function = (KEYFUNC) ap_peripheral_pw_key_exe;

		//adc detect key handle
        ad_key_map[4].key_io = OK_KEY;
        ad_key_map[4].key_function = (KEYFUNC)ap_peripheral_ok_key_exe; 
        ad_key_map[4].long_flag = 0;
        ad_key_map[4].key_cnt = 0;

        ad_key_map[3].key_io = FUNCTION_KEY1;
        ad_key_map[3].key_function = (KEYFUNC)ap_peripheral_menu_key_exe; 
        ad_key_map[3].long_flag = 0;
        ad_key_map[3].key_cnt = 0;

        ad_key_map[2].key_io = FUNCTION_KEY2 ;
        ad_key_map[2].key_function = (KEYFUNC)ap_peripheral_prev_key_exe; 
        ad_key_map[2].long_flag = 0;
        ad_key_map[2].key_cnt = 0;

        ad_key_map[1].key_io = FUNCTION_KEY3 ;
        ad_key_map[1].key_function = (KEYFUNC)ap_peripheral_next_key_exe; 
        ad_key_map[1].long_flag = 0;
        ad_key_map[1].key_cnt = 0;

        ad_key_map[0].key_io = FUNCTION_KEY4 ;
        ad_key_map[0].key_function = (KEYFUNC)ap_peripheral_function_key_exe; 
        ad_key_map[0].long_flag = 0;
        ad_key_map[0].key_cnt = 0;
    }
    else if (type == USBD_DETECT)
    {
#if USE_IOKEY_NO
        for (i = 0 ; i < USE_IOKEY_NO ; i++)
        {
            key_map[i].key_io = NULL;
        }
#endif
#if USE_ADKEY_NO
        for (i = 0 ; i < USE_ADKEY_NO ; i++)
        {
            ad_key_map[i].key_function = ap_peripheral_null_key_exe;
        }
#endif
    }
    else if (type == DISABLE_KEY)
    {
#if USE_IOKEY_NO
        for (i = 0 ; i < USE_IOKEY_NO ; i++)
        {
            key_map[i].key_io = NULL;
        }
#endif
#if USE_ADKEY_NO
        for (i = 0 ; i < USE_ADKEY_NO ; i++)
        {
            ad_key_map[i].key_function = ap_peripheral_null_key_exe;
        }
#endif
    }
    else if (type == BETTERY_LOW_STATUS_KEY)
    {
        key_map[0].key_io = PW_KEY;
        key_map[0].key_function = (KEYFUNC) ap_peripheral_pw_key_exe;
#if USE_ADKEY_NO
        for (i = 0 ; i < USE_ADKEY_NO ; i++)
        {
            ad_key_map[i].key_function = ap_peripheral_null_key_exe;
        }
#endif
    }
}


extern INT8U ap_state_config_auto_off_get(void);

INT8U long_pw_key_pressed = 0;
#if CRAZY_KEY_TEST == 1
INT8U crazy_key_enable = 0;
INT32U crazy_key_cnt = 0;
#endif
void ap_peripheral_key_judge(void)
{
    INT32U i, key_press = 0;

    INT16U key_down = 0;
    LED_service();

    for (i = 0 ; i < USE_IOKEY_NO ; i++)
    {
        if(key_map[i].key_io)
        {
            if(sys_pwr_key0_read())
            {
                set_pa_cnt(128);
                if(!long_pw_key_pressed)
                {
                    key_map[i].key_cnt += 1;
                    if (key_map[i].key_cnt >= 48)
                    {
                        key_press = 1;
                        key_active_cnt = 0;
                        long_pw_key_pressed = 1;
                        if(power_keyup == 0)
                            key_map[i].key_function(&(key_map[i].key_cnt));
                    }
                }
                else
                {
                    key_map[i].key_cnt = 0;
                }
                if (key_map[i].key_cnt == 65535)
                {
                    key_map[i].key_cnt = 16;
                }
            }
            else
            {

                long_pw_key_pressed = 0;
                if(key_map[i].key_cnt >= 3) //Short_Single_width
                {
                    if(power_keyup == 0)
                    {
                        if(led_main_flag == LED_ON)
                            led_main_off();
                        else
                            led_main_on();
                    }
                    key_active_cnt = 0;
                    key_press = 1;
                }
                key_map[i].key_cnt = 0;

                power_keyup = 0;
                {
#if C_SCREEN_SAVER == CUSTOM_ON
                    INT32U cnt_sec;
                    INT32U screen_auto_off;
                    INT32U cnt_clr_flag = 0;
                    INT32U screen_auto_off_TFT_BL;
					
                    screen_auto_off = ap_state_config_auto_off_get();
					
                    if ((screen_auto_off != 0) && !auto_off_force_disable && !s_usbd_pin && !ap_state_config_md_get())
                    {
                        //don't auto off under following conditions:
                        //1. recording & playing avi files(by auto_off_force_disable)
                        //2. usb connecting
                        //3. motion detect on
                        if(screen_auto_off == 2)  	//3 min
                        {
                            screen_auto_off = 3;
                        }
                        else if (screen_auto_off == 3)     //5min
                        {
                            screen_auto_off = 5;
                        }

                        if (cnt_clr_flag == 0)
                        {
                            key_active_cnt += PERI_TIME_INTERVAL_AD_DETECT;//PERI_TIME_INTERVAL_KEY_DETECT;
                        }
                        cnt_sec = (key_active_cnt >> 7) / USE_IOKEY_NO;
                        if (cnt_sec > screen_auto_off * 60)
                        {
                            key_active_cnt = 0;

                            if(screen_saver_enable)
                            {
                                screen_saver_enable = 0;
                                msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
                            }
							
                            msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
                            DBG_PRINT("NO active Auto Power off\r\n");

                        }
                        cnt_clr_flag++;
                    }

                    screen_auto_off_TFT_BL = ap_state_config_auto_off_TFT_BL_get() ;  // 0=>off, 1=>3min, 2=>5min, 3=>10min
                    if  ( (screen_saver_enable == 0) && (screen_auto_off_TFT_BL != 0) )
                    {
                        if (screen_auto_off_TFT_BL == 1)
                        {
                            screen_auto_off_TFT_BL = 3;  // 3min
                        }
                        else if (screen_auto_off_TFT_BL == 2)
                        {
                            screen_auto_off_TFT_BL = 5; // 5 min
                        }
                        else if (screen_auto_off_TFT_BL == 3)
                        {
                            screen_auto_off_TFT_BL = 10; // 10min
                        }

                        if (cnt_clr_flag == 0)
                        {
                            key_active_cnt += PERI_TIME_INTERVAL_AD_DETECT;//PERI_TIME_INTERVAL_KEY_DETECT;
                        }
                        cnt_sec = (key_active_cnt >> 7) / USE_IOKEY_NO;
                        if (cnt_sec > screen_auto_off_TFT_BL * 60)
                        {
                            key_active_cnt = 0;
                            ap_state_handling_lcd_backlight_switch(0);  // trun off Backligth
                            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SCREEN_SAVER_ENABLE, NULL, NULL, MSG_PRI_NORMAL);
                            DBG_PRINT("Turn off Backlight\r\n");
                        }
                        cnt_clr_flag++;
                    }


                    if(screen_saver_enable && key_press)
                    {
                        screen_saver_enable = 0;
                        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
                    }

                    if (cnt_clr_flag == 0)
                    {
                        key_active_cnt = 0;
                    }
#endif
                }
            }
        }
    }

}

static int ap_peripheral_power_key_read(int pin)
{
    int status;


#if  (KEY_TYPE == KEY_TYPE1)||(KEY_TYPE == KEY_TYPE2)||(KEY_TYPE == KEY_TYPE3)||(KEY_TYPE == KEY_TYPE4)||(KEY_TYPE == KEY_TYPE5)
    status = gpio_read_io(pin);
#else
    switch(pin)
    {
    case PWR_KEY0:
        status = sys_pwr_key0_read();
        break;
    case PWR_KEY1:
        status = sys_pwr_key1_read();
        break;
    }
#endif

    if (status != 0)
        return 1;
    else return 0;
}

void ap_peripheral_adaptor_out_judge(void)
{

#if USB_PHY_SUSPEND == 1
    if (s_usbd_pin == 0)
    {
        if (!ap_peripheral_power_key_read(C_USBDEVICE_PIN))
        {
            if (phy_cnt == PERI_USB_PHY_SUSPEND_TIME)
            {
                // disable USB PHY CLK for saving power
                DBG_PRINT("MSG_CHARGE_PLUG_OUT\r\n");
                OSQPost(USBTaskQ, (void *) MSG_CHARGE_PLUG_OUT);  // 一定要在下一次 MSG_USBD_INITIAL 送到 USB QUEUE 裏
                phy_cnt++;	// 目的是 Turn Off 只做一次
            }
            else if (phy_cnt < PERI_USB_PHY_SUSPEND_TIME)
            {
                phy_cnt++;
            }
        }
        else
        {
            phy_cnt = 0;
        }
    }
    else phy_cnt = 0;
#endif

    adp_out_cnt++;
    switch(adp_status)
    {
    case 0: //unkown state
        if (ap_peripheral_power_key_read(ADP_OUT_PIN))
        {
            adp_cnt++;
            if (adp_cnt > 16)
            {
                adp_out_cnt = 0;
                adp_cnt = 0;
                adp_status = 1;
                OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
#if C_BATTERY_DETECT == CUSTOM_ON && USE_ADKEY_NO
                //battery_lvl = 1;
#endif
            }
        }
        else
        {
            adp_cnt = 0;
        }

        if (adp_out_cnt > 24)
        {
            adp_out_cnt = 0;
            adp_status = 3;
#if C_BATTERY_DETECT == CUSTOM_ON && USE_ADKEY_NO
            //battery_lvl = 2;
            low_voltage_cnt = 0;
#endif
        }
        break;

    case 1: //adaptor in state
        if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
        {
            if (adp_out_cnt > 8)
            {
                adp_status = 2;
                low_voltage_cnt = 0;
                // 若螢幕保護開時，要點亮背光
                if(screen_saver_enable)
                {
                    screen_saver_enable = 0;
                    ap_state_handling_lcd_backlight_switch(1);
                }
            }
        }
        else
        {
            adp_out_cnt = 0;
        }
        break;

    case 2: //adaptor out state
        if (!ap_peripheral_power_key_read(ADP_OUT_PIN))
        {
            if ((adp_out_cnt > PERI_ADP_OUT_PWR_OFF_TIME))
            {
                adp_out_cnt = 48;
                ap_peripheral_pw_key_exe(&adp_out_cnt);
            }
            adp_cnt = 0;
        }
        else
        {
            adp_cnt++;
            if (adp_cnt > 3)
            {
                adp_out_cnt = 0;
                adp_status = 1;
                usbd_exit = 0;
                OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
            }
        }
        break;

    case 3://adaptor initial out state
        if (ap_peripheral_power_key_read(ADP_OUT_PIN))
        {
            if (adp_out_cnt > 3)
            {
                adp_out_cnt = 0;
                adp_status = 1;
                OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
            }
        }
        else
        {
            adp_out_cnt = 0;
        }
        break;
    default:
        break;
    }

    if (s_usbd_pin == 1)
    {
        usbd_cnt++;
        if (!ap_peripheral_power_key_read(C_USBDEVICE_PIN))
        {
            if (usbd_cnt > 3)
            {
                ap_peripheral_usbd_plug_out_exe(&usbd_cnt);
            }
        }
        else
        {
            usbd_cnt = 0;
        }
    }


}


void ap_peripheral_function_key_exe(INT16U *tick_cnt_ptr)
{
    msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_MODE, NULL, NULL, MSG_PRI_NORMAL);
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        DBG_PRINT("function_Key\r\n");
#if 0// KEY_FUNTION_TYPE == SAMPLE2
        if(*tick_cnt_ptr > 24)
        {
            if(MODE_KEY_flag == 2)
            {
                DBG_PRINT("MENU_ACTIVE\r\n");
                msgQSend(ApQ, MSG_APQ_MENU_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);

            }
            else if(MODE_KEY_flag == 1)
            {
                DBG_PRINT("MENU_ACTIVE\r\n");
                msgQSend(ApQ, MSG_APQ_MENU_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            }
            else
            {
                DBG_PRINT("MODE_ACTIVE\r\n");
                msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
            }
        }
        else
        {
            if(MODE_KEY_flag == 2)
            {
                DBG_PRINT("MODE_ACTIVE\r\n");
                DBG_PRINT("*tick_cnt_ptr=%d\r\n", *tick_cnt_ptr);
                msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
            }
            else if(MODE_KEY_flag == 1)
            {
                DBG_PRINT("MENU_ACTIVE\r\n");
                DBG_PRINT("*tick_cnt_ptr=%d\r\n", *tick_cnt_ptr);
                msgQSend(ApQ, MSG_APQ_MENU_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            }
            else
            {
                DBG_PRINT("MODE_ACTIVE\r\n");
                DBG_PRINT("*tick_cnt_ptr=%d\r\n", *tick_cnt_ptr);
                msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
            }
        }
#else
        if(*tick_cnt_ptr > 31)
        {
        }
        else
        {
            if((!video_down_flag) && (!pic_down_flag))
            {
                DBG_PRINT("MODE_ACTIVE,*tick_cnt_ptr=%d\r\n", *tick_cnt_ptr);
                msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);		//mode switch key function.
            }
        }
#endif
    }
    *tick_cnt_ptr = 0;
}

void ap_peripheral_next_key_exe(INT16U *tick_cnt_ptr)
{
    INT8U data = 0;
    //if(get_check_flag(15))
    // 	{
    // 	 set_check_flag(1,1);
    //	 tick_cnt_ptr =0;
    //	 return;
    //	}
    msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_DOWN, NULL, NULL, MSG_PRI_NORMAL);
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        if((!video_down_flag) && (!pic_down_flag))
        {
            if(*tick_cnt_ptr > 31)
            {
                msgQSend(ApQ, MSG_APQ_FORWARD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);
            }
            else
            {
                msgQSend(ApQ, MSG_APQ_NEXT_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
            }
        }
    }

    *tick_cnt_ptr = 0;
}

void ap_peripheral_prev_key_exe(INT16U *tick_cnt_ptr)
{
    INT8U data = 0;

    msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_UP, NULL, NULL, MSG_PRI_NORMAL);
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        if((!video_down_flag) && (!pic_down_flag))
        {
            if(*tick_cnt_ptr > 31)
            {
                msgQSend(ApQ, MSG_APQ_BACKWORD_FAST_PLAY, &data, sizeof(INT8U), MSG_PRI_NORMAL);		//rr function, long prev key. 

            }
            else
            {
                msgQSend(ApQ, MSG_APQ_PREV_KEY_ACTIVE, &data, sizeof(INT8U), MSG_PRI_NORMAL);
            }
        }
    }
    *tick_cnt_ptr = 0;
}

void ap_peripheral_ok_key_exe(INT16U *tick_cnt_ptr)
{
#if 1
    msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_OK, NULL, NULL, MSG_PRI_NORMAL);
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        if((!video_down_flag) && (!pic_down_flag))
        {
            if(*tick_cnt_ptr > 31)
            {
                msgQSend(ApQ, MSG_APQ_FUNCTION_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            }
            else
            {
                DBG_PRINT("ok_key\r\n");
                msgQSend(ApQ, MSG_APQ_FUNCTION_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            }
        }
    }
#endif
    *tick_cnt_ptr = 0;
}
#if KEY_FUNTION_TYPE == SAMPLE2
void ap_peripheral_capture_key_exe(INT16U *tick_cnt_ptr)
{
    msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_OK, NULL, NULL, MSG_PRI_NORMAL);
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        msgQSend(ApQ, MSG_APQ_CAPTURE_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
    }
    *tick_cnt_ptr = 0;
}
#endif

void ap_peripheral_brose_exe(INT16U *tick_cnt_ptr)
{
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {

        if (!s_usbd_pin)
        {
            if(ap_state_handling_storage_id_get() != NO_STORAGE)
            {
                if(*tick_cnt_ptr > 31)
                {

                }
                else
                {
                    msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
                }
            }
        }
    }


}
void ap_peripheral_capture_exe(INT16U *tick_cnt_ptr)
{
    DBG_PRINT("capture_exe\r\n");
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {

        if (!s_usbd_pin)
        {
            if(ap_state_handling_storage_id_get() != NO_STORAGE)
            {
                if(*tick_cnt_ptr > 31)
                {

                    DBG_PRINT("capture_long_key\r\n");
                }
                else
                {
                    DBG_PRINT("capture_key\r\n");
                    msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
                }
            }
        }
    }

}
void ap_peripheral_video_exe(INT16U *tick_cnt_ptr)
{
    DBG_PRINT("video_exe\r\n");
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        if (!s_usbd_pin)
        {
            if(ap_state_handling_storage_id_get() != NO_STORAGE)
            {
                if(*tick_cnt_ptr > 31)
                {
                    DBG_PRINT("video_longkey\r\n");
                }
                else
                {
                    DBG_PRINT("video_key\r\n");
                    msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
                }
            }
        }
    }

}

void ap_peripheral_sos_key_exe(INT16U *tick_cnt_ptr)
{
#if 0
#if CRAZY_KEY_TEST == 1
    if (!crazy_key_enable)
    {
        crazy_key_enable = 1;
    }
    else
    {
        crazy_key_enable = 0;
    }
    *tick_cnt_ptr = 0;
    return;
#endif

    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        if(*tick_cnt_ptr > 24)
        {
        }
        else
        {
            msgQSend(ApQ, MSG_APQ_SOS_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
        }
    }
#endif
    *tick_cnt_ptr = 0;
}

void ap_peripheral_usbd_plug_out_exe(INT16U *tick_cnt_ptr)
{
    msgQSend(ApQ, MSG_APQ_DISCONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
    *tick_cnt_ptr = 0;
}

void ap_peripheral_pw_key_exe(INT16U *tick_cnt_ptr)
{
    //DBG_PRINT("xx=%d\r\n",screen_saver_enable);
    if(screen_saver_enable)
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        if(*tick_cnt_ptr > 47)
        {
            DBG_PRINT("Power key OFF\r\n");

            msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
        }
        else
        {
            //	msgQSend(ApQ, MSG_APQ_PARK_MODE_SET, NULL, NULL, MSG_PRI_NORMAL);
            //			msgQSend(ApQ, MSG_APQ_NIGHT_MODE_KEY, NULL, NULL, MSG_PRI_NORMAL);
        }
    }
    *tick_cnt_ptr = 0;
}

void ap_peripheral_menu_key_exe(INT16U *tick_cnt_ptr)
{
#if KEY_FUNTION_TYPE == C6_KEY
    msgQSend(ApQ, MSG_APQ_AUDIO_EFFECT_MENU, NULL, NULL, MSG_PRI_NORMAL);
    if(screen_saver_enable)			// open screen, screen working mode.
    {
        screen_saver_enable = 0;
        msgQSend(ApQ, MSG_APQ_KEY_WAKE_UP, NULL, NULL, MSG_PRI_NORMAL);
    }
    else
    {
        if((!video_down_flag) && (!pic_down_flag))
        {
            if(*tick_cnt_ptr > 31)
            {
            }
            else
            {

                msgQSend(ApQ, MSG_APQ_MENU_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            }
        }
    }
#endif
    *tick_cnt_ptr = 0;
}
INT32U sd_pos_ck = 3;
//extern INT8S Card_ReadSenddata_main(void);
void ap_peripheral_null_key_exe(INT16U *tick_cnt_ptr)
{
    DBG_PRINT("null_key_exe\r\n");
#if 0
    if(sd_pos_ck == SDC1_IOD10_IOD11_IOD12_IOD13_IOD14_IOD15)
    {
        sd_choice(SDC0_IOB14_IOB15_IOB10_IOB11_IOB12_IOB13);
        sd_pos_ck = SDC0_IOB14_IOB15_IOB10_IOB11_IOB12_IOB13;
    }
    else
    {
        sd_choice(SDC1_IOD10_IOD11_IOD12_IOD13_IOD14_IOD15);
        sd_pos_ck = SDC1_IOD10_IOD11_IOD12_IOD13_IOD14_IOD15;
    }
#endif
    //	Card_trans_loop();

}

void ap_TFT_backlight_tmr_check(void)
{
    if(backlight_tmr)
    {
        backlight_tmr--;
        if((backlight_tmr == 0) && (tv == !TV_DET_ACTIVE))
        {
            //gpio_write_io(TFT_BL, DATA_HIGH);	//turn on LCD backlight
            tft_backlight_en_set(1);
        }
    }
}

//+++ TV_OUT_D1
#if TV_DET_ENABLE
INT8U tv_plug_status_get(void)
{
    return tv_plug_in_flag;
}
#endif
//---

void ap_peripheral_tv_detect(void)
{
#if TV_DET_ENABLE
    INT8U temp;

    temp = gpio_read_io(AV_IN_DET);
    if(temp != tv)
    {
        tv_debounce_cnt++;
        if(tv_debounce_cnt > 4)
        {
            tv_debounce_cnt = 0;
            tv = temp;
            if(tv == !TV_DET_ACTIVE)  	//display use TFT
            {
                //backlight_tmr = PERI_TIME_BACKLIGHT_DELAY;	//delay some time to enable LCD backlight so that no noise shown on LCD
                gpio_write_io(SPEAKER_EN, DATA_HIGH);	//open local speaker

                //+++ TV_OUT_D1
                tv_plug_in_flag = 0;
                msgQSend(ApQ, MSG_APQ_TV_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
                //---

            }
            else     //display use TV
            {
                gpio_write_io(SPEAKER_EN, DATA_LOW);	//mute local speaker
                //gpio_write_io(TFT_BL, DATA_LOW);		//turn off LCD backlight

                //+++ TV_OUT_D1
                tv_plug_in_flag = 1;
                msgQSend(ApQ, MSG_APQ_TV_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
                //---
            }
        }
    }
    else
    {
        tv_debounce_cnt = 0;
    }
#endif

}

void ap_peripheral_gsensor_data_register(void )
{
    avi_adc_gsensor_data_register(&gsensor_msgQId0, (INT32U *)(&gsensor_msgId0));
}

void ap_peripheral_read_gsensor(void)
{
    static INT16U g_idx = 0;
    INT16U temp;

    if(gsensor_lock_flag) return;
    gsensor_lock_flag = 1;

    temp = G_sensor_get_int_active();

    if((temp != 0xff) && (temp & 0x04)) //active int flag
    {
        G_sensor_clear_int_flag();

        if(ap_state_config_G_sensor_get())
        {
            msgQSend(ApQ, MSG_APQ_SOS_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
        }

        DBG_PRINT("gsensor int actived\r\n");
    }

    if (gsensor_msgQId0 != NULL)
    {
        G_Sensor_gps_data_get(gsensor_data[g_idx]);
        OSQPost((OS_EVENT *)gsensor_msgQId0, (void *)(gsensor_msgId0 | g_idx));
        g_idx ^= 0x1;
    }

    gsensor_lock_flag = 0;
    //DBG_PRINT("gsensor chipid = 0x%x\r\n", temp);
}

void ap_peripheral_config_store(void)
{
    if (config_cnt++ == PERI_COFING_STORE_INTERVAL)
    {
        config_cnt = 0;
        msgQSend(ApQ, MSG_APQ_USER_CONFIG_STORE, NULL, NULL, MSG_PRI_NORMAL);
    }
}


void ap_peripheral_hdmi_detect(void)
{
    static BOOLEAN	HDMI_StatusBak = 0;
    static BOOLEAN	HDMI_StateBak = 0;	// HDMI_REMOVE
    static unsigned char HDMI_DetCount = 0;
    BOOLEAN cur_status;

    cur_status = gpio_read_io(HDMI_IN_DET);
    //DBG_PRINT("hdmi:%d",cur_status);
    // debounce
    if (HDMI_StatusBak != cur_status)
    {
        HDMI_DetCount = 0;
    }
    else
    {
        HDMI_DetCount++;
    }

    if (HDMI_DetCount == 0x10)
    {
        if (cur_status != HDMI_StateBak)
        {
            HDMI_DetCount = 0;
            if(cur_status)	// HDM_IN_DET
            {
                msgQSend(ApQ, MSG_APQ_HDMI_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
                gpio_write_io(SPEAKER_EN, DATA_LOW);	//mute local speaker
                DBG_PRINT("HDMI Insert\r\n");	// HDMI Insert
            }
            else
            {
                msgQSend(ApQ, MSG_APQ_HDMI_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
                gpio_write_io(SPEAKER_EN, DATA_HIGH);

                DBG_PRINT("HDMI Remove\r\n");	// HDMI Remove
            }
        }
        HDMI_StateBak = cur_status;
    }
    HDMI_StatusBak = cur_status;

}

#ifdef SDC_DETECT_PIN

void ap_peripheral_SDC_detect_init(void)
{

    gpio_init_io(SDC_DETECT_PIN, GPIO_INPUT);
    gpio_set_port_attribute(SDC_DETECT_PIN, ATTRIBUTE_LOW);
    gpio_write_io(SDC_DETECT_PIN, 1);	//pull high
}

INT32S ap_peripheral_SDC_at_plug_OUT_detect()
{
    INT32S ret;
    BOOLEAN cur_status;
    ap_peripheral_SDC_detect_init();
    cur_status = gpio_read_io(SDC_DETECT_PIN);
    //	DBG_PRINT("SDC_DETECT_PIN_=%d\r\n",cur_status);
    if(cur_status) 		//plug_out
    {
        ret = -1;
    }
    else 			//plug_in
    {
        ret = 0;
    }
    return ret;
}



INT32S ap_peripheral_SDC_at_plug_IN_detect()
{
    INT32S ret;
    BOOLEAN cur_status;
    ap_peripheral_SDC_detect_init();
    cur_status = gpio_read_io(SDC_DETECT_PIN);
    //	DBG_PRINT("SDC_DETECT_PIN=%d\r\n",cur_status);
    if(cur_status) 		//plug_out
    {
        ret = -1;
    }
    else 				//plug_in
    {
        ret = 0;
    }
    return ret;
}

#endif
