#include "state_setting.h"
#include "ap_music.h"
#include "ap_state_config.h"
#include "avi_encoder_app.h"
#include "ap_state_handling.h"
#include "ap_video_record.h"
#include "ap_display.h"

//	prototypes
void state_setting_init(INT32U prev_state, INT32U prev_state1, INT8U *tag);
void state_setting_exit(INT32U prev_state);

extern INT32S video_encode_task_start(void);
extern OS_EVENT *avi_encode_ack_m;
extern void ap_USB_setting_page_draw(INT32U state, INT32U state1, INT8U *tag, INT32U buff_addr);
extern void Card_trans_loop();

INT8U USB_select_entry(void *para, INT32U buff_addr)
{
    EXIT_FLAG_ENUM exit_flag = EXIT_RESUME;
    INT32U msg_id, prev_state, prev_state1;
    INT8U curr_tag, sub_tag;
    INT32U led_type;
    STAudioConfirm *audio_temp;

    curr_tag = 0;
    sub_tag = 0xFF;
    prev_state = prev_state1 = *((INT32U *) para);
    ap_USB_setting_page_draw(prev_state, prev_state1, &curr_tag, buff_addr);
    while (exit_flag == EXIT_RESUME)
    {
        if (msgQReceive(ApQ, &msg_id, (void *) ApQ_para, AP_QUEUE_MSG_MAX_LEN) == STATUS_FAIL)
        {
            continue;
        }
        switch (msg_id)
        {
        case EVENT_APQ_ERR_MSG:
            audio_temp = (STAudioConfirm *)ApQ_para;
            if ((audio_temp->result == AUDIO_ERR_DEC_FINISH) && (audio_temp->source_type == AUDIO_SRC_TYPE_APP_RS))
            {
            }
            else
            {
                audio_confirm_handler((STAudioConfirm *)ApQ_para);
            }
            break;
        case MSG_STORAGE_SERVICE_MOUNT:
            ap_state_handling_storage_id_set(ApQ_para[0]);
            DBG_PRINT("[Setting Mount OK]\r\n");
            break;
        case MSG_STORAGE_SERVICE_NO_STORAGE:
            ap_state_handling_storage_id_set(ApQ_para[0]);
            DBG_PRINT("[Setting Mount FAIL]\r\n");
            break;
        case MSG_APQ_POWER_KEY_ACTIVE:
            ap_state_handling_charge_icon_show(0);
            OSTimeDly(10);

            ap_state_handling_power_off(0);
            break;

        //case MSG_APQ_COPY_CARD:
        //	DBG_PRINT("KK\r\n");
        //			   Card_trans_loop();
        //		   break;

        case MSG_APQ_DISCONNECT_TO_PC:
            led_type = LED_USB_DISCONNECT; // LED_RECORD;
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_START, NULL, NULL, MSG_PRI_NORMAL);
            msgQSend(ApQ, MSG_APQ_DISCONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
            exit_flag = EXIT_BREAK;
            break;

        case MSG_APQ_FUNCTION_KEY_ACTIVE:
            exit_flag = EXIT_BREAK;
            break;
        case MSG_APQ_NEXT_KEY_ACTIVE:
        case MSG_APQ_PREV_KEY_ACTIVE:
            ap_USB_setting_direction_key_active(&curr_tag, &sub_tag, msg_id, prev_state, prev_state1);
            break;

        default:
            ap_state_common_handling(msg_id);
            break;
        }
    }

    if(curr_tag == 0)
    {
        ap_state_config_usb_mode_set(0);
    }
    else
    {
        ap_state_config_usb_mode_set(1);
    }

    if(msg_id == MSG_APQ_FUNCTION_KEY_ACTIVE)
    {
        ap_USB_setting_display_ok();
        OSTimeDly(5);
    }
    return curr_tag;
}


void state_setting_init(INT32U prev_state, INT32U prev_state1, INT8U *tag)
{
    DBG_PRINT("setting state init enter\r\n");

    ap_setting_init(prev_state, prev_state1, tag);
}

void state_setting_entry(void *para)
{
    EXIT_FLAG_ENUM exit_flag = EXIT_RESUME;
    INT32U msg_id, prev_state, prev_state1;
    INT8U curr_tag, sub_tag;
    INT32U led_type;
    STAudioConfirm *audio_temp;

    curr_tag = 0;
    sub_tag = 0xFF;
    prev_state = prev_state1 = *((INT32U *) para);

    state_setting_init(prev_state, prev_state1, &curr_tag);

    //ap_current_state_set(State_menu);
    while (exit_flag == EXIT_RESUME)
    {
        if (msgQReceive(ApQ, &msg_id, (void *) ApQ_para, AP_QUEUE_MSG_MAX_LEN) == STATUS_FAIL)
        {
            continue;
        }

        switch (msg_id)
        {
        case EVENT_APQ_ERR_MSG:
            audio_temp = (STAudioConfirm *)ApQ_para;
            if ((audio_temp->result == AUDIO_ERR_DEC_FINISH) && (audio_temp->source_type == AUDIO_SRC_TYPE_APP_RS))
            {
            }
            else
            {
                audio_confirm_handler((STAudioConfirm *)ApQ_para);
            }
            break;
        case MSG_STORAGE_SERVICE_MOUNT:
            ap_state_handling_storage_id_set(ApQ_para[0]);
            DBG_PRINT("[Setting Mount OK]\r\n");
            break;
        case MSG_STORAGE_SERVICE_NO_STORAGE:
            ap_state_handling_storage_id_set(ApQ_para[0]);
            DBG_PRINT("[Setting Mount FAIL]\r\n");
            break;
        case MSG_APQ_POWER_KEY_ACTIVE:
            video_calculate_left_recording_time_disable();
            ap_setting_exit(prev_state1);
            OSTimeDly(10);

            ap_state_handling_power_off(0);
            break;
        case MSG_APQ_MENU_KEY_ACTIVE:
            exit_flag = ap_setting_menu_key_active(&curr_tag, &sub_tag, &prev_state, &prev_state1);
            if(exit_flag == EXIT_BREAK)
            {
                OSTimeDly(5);
            }
            break;
        case MSG_APQ_MODE:
            //SETTING 按模式键后退回VIDEO
            //		prev_state1=STATE_VIDEO_RECORD;
            exit_flag = ap_setting_mode_key_active(prev_state1, &sub_tag);
            break;
        case MSG_APQ_FUNCTION_KEY_ACTIVE:
            ap_setting_func_key_active(&curr_tag, &sub_tag, prev_state, prev_state1);
            break;
        case MSG_APQ_NEXT_KEY_ACTIVE:
        case MSG_APQ_PREV_KEY_ACTIVE:
            DBG_PRINT("NEXT_PREV_KEY\r\n");
            ap_setting_direction_key_active(&curr_tag, &sub_tag, msg_id, prev_state, prev_state1);
            break;
        case MSG_STORAGE_SERVICE_BROWSE_REPLY:
            ap_setting_reply_action(prev_state, prev_state1, &curr_tag, (STOR_SERV_PLAYINFO *) ApQ_para);
            break;

        case MSG_APQ_CONNECT_TO_PC:
            led_type = LED_USB_CONNECT; // LED_RECORD;
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
            if(ap_display_get_device() != DISP_DEV_TFT) break;
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_STOP, NULL, NULL, MSG_PRI_NORMAL);
            OSTimeDly(3);
        case MSG_APQ_BATTERY_LOW_SHOW:
            //	DBG_PRINT("brose_batt_low\r\n");
            //	prev_state1=STATE_VIDEO_RECORD;
            //	exit_flag = ap_setting_mode_key_active(prev_state1, &sub_tag);
            //	exit_flag = EXIT_BREAK;
            ap_state_handling_clear_all_icon();
            OSTimeDly(5);
            ap_state_handling_str_draw_exit();
            ap_state_handling_str_draw(STR_BATTERY_LOW, WARNING_STR_COLOR);
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_BATTERY_LOW_BLINK_START, NULL, NULL, MSG_PRI_NORMAL);
            break;

        case MSG_STORAGE_SERVICE_FORMAT_REPLY:
            ap_setting_format_reply(&curr_tag, prev_state, prev_state1);
            DBG_PRINT("setting format done!\r\n");
            msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_START, NULL, NULL, MSG_PRI_NORMAL);
            break;
#if C_SCREEN_SAVER == CUSTOM_ON
        case MSG_APQ_KEY_IDLE:
            ap_state_handling_lcd_backlight_switch(0);
            break;
        case MSG_APQ_KEY_WAKE_UP:
            ap_state_handling_lcd_backlight_switch(1);
            break;
#endif
        case MSG_APQ_SELECT_FILE_DEL_REPLY:
            if(ApQ_para[0] == 0x55)
            {
                ap_setting_del_protect_file_show(&curr_tag, prev_state, prev_state1);
                break;
            }
            ap_setting_other_reply();
            OSQPost(StateHandlingQ, (void *) (0xA5A50000 | STATE_BROWSE) );
            OSTimeDly(5);//wait for display queue empty
            exit_flag = EXIT_BREAK;
            break;

        case MSG_APQ_COPY_CARD:
            DBG_PRINT("KK\r\n");
            Card_trans_loop();
            break;

        case MSG_APQ_FILE_DEL_ALL_REPLY:
        case MSG_APQ_FILE_LOCK_ONE_REPLY:
        case MSG_APQ_FILE_LOCK_ALL_REPLY:
        case MSG_APQ_FILE_UNLOCK_ONE_REPLY:
        case MSG_APQ_FILE_UNLOCK_ALL_REPLY:
            ap_setting_other_reply();
            OSQPost(StateHandlingQ, (void *) STATE_BROWSE);
            OSTimeDly(5);//wait for display queue empty
            exit_flag = EXIT_BREAK;
            break;

        case MSG_APQ_USER_CONFIG_STORE:
            ap_state_config_store();
            break;

        //+++ TV_OUT_D1
        case MSG_APQ_TV_PLUG_OUT:
        case MSG_APQ_TV_PLUG_IN:
        case MSG_APQ_HDMI_PLUG_IN:
            OSQPost(StateHandlingQ, (void *) prev_state1);
            exit_flag = EXIT_BREAK;
            break;

        default:
            ap_state_common_handling(msg_id);
            break;
        }
    }

    state_setting_exit(prev_state1);

    if (msg_id == MSG_APQ_MODE)
    {
        //		msgQFlush(ApQ);
        //	    msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
    }
    else if(msg_id == MSG_APQ_CONNECT_TO_PC)
    {
        msgQFlush(ApQ);
        msgQSend(ApQ, MSG_APQ_CONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
    }
    else if(msg_id == MSG_APQ_BATTERY_LOW_SHOW)
    {
        msgQFlush(ApQ);
        msgQSend(ApQ, MSG_APQ_BATTERY_LOW_SHOW, NULL, sizeof(INT8U), MSG_PRI_NORMAL);
    }
    else if((msg_id == MSG_APQ_TV_PLUG_IN) || (msg_id == MSG_APQ_TV_PLUG_OUT) || (msg_id == MSG_APQ_HDMI_PLUG_IN))
    {
        msgQSend(ApQ, msg_id, NULL, NULL, MSG_PRI_NORMAL);
    }
}

void state_setting_exit(INT32U prev_state)
{
    ap_setting_exit(prev_state);
    DBG_PRINT("Exit setting state\r\n");
}
