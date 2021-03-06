#include "state_browse.h"
#include "task_video_decoder.h"
#include "ap_state_handling.h"
#include "ap_music.h"
#include "ap_state_config.h"
#include "ap_display.h"
#include "ap_browse.h"
#include "ap_video_preview.h"
#include "avi_encoder_app.h"
#include "ap_peripheral_handling.h"

//	prototypes
void state_browse_init(INT32U prev_state, INT16U play_index);
void state_browse_exit(INT32U next_state_msg);

extern STOR_SERV_PLAYINFO browse_curr_avi;

static INT8U mode_key_enabled = 0;
static delayed_action_flag = 0;
INT8U brose_flag = 0;
void state_browse_init(INT32U prev_state, INT16U play_index)
{
    DBG_PRINT("Browse state init enter\r\n");

    g_browser_reply_action_flag = 0;
    delayed_action_flag = 0;

    ap_browse_init(prev_state, play_index);
    browser_play_speed = 0;
}
//extern INT32U gStop_Sensor;
extern volatile INT8U  seninit_flag;
void state_browse_entry(void *para, INT16U play_index)
{
    EXIT_FLAG_ENUM exit_flag = EXIT_RESUME;
    INT32U msg_id, prev_state, search_type;
    STAudioConfirm *audio_temp;
    INT8U  func_key_act = 0;
    INT32U led_type;

    DBG_PRINT("state_browse_entry()\r\n");
    brose_flag = 1;

    prev_state = *((INT32U *) para);
    if((prev_state == STATE_SETTING) || (prev_state == STATE_THUMBNAIL))
    {
        msgQAccept(ApQ, &msg_id, (void *) ApQ_para, AP_QUEUE_MSG_MAX_LEN);
        if(msg_id == MSG_APQ_CONNECT_TO_PC)
        {
            msgQSend(ApQ, MSG_APQ_CONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
        }
        else
        {
            OS_Q_DATA data;
            msgQQuery(ApQ, &data);
            if (data.OSNMsgs != 0) // if queue isn't empty.   modify by josephhsieh@20150205
            {
                msgQSend(ApQ, msg_id, (void *) ApQ_para, sizeof(STAudioConfirm), MSG_PRI_NORMAL);	//max size of para is STAudioConfirm
            }
            state_browse_init(prev_state, play_index);
        }
    }
    else
    {
        state_browse_init(prev_state, play_index);
    }
    //ap_current_state_set(State_brose);
    //	ap_state_handling_icon_show_cmd(ICON_LOCKED1, NULL, NULL);
    //ap_pa_ctrl(1);

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
            if( (audio_temp->result == AUDIO_ERR_DEC_FINISH) && (audio_temp->source_type == AUDIO_SRC_TYPE_FS) )
            {
                browser_play_speed = 0;
                ap_state_handling_icon_clear_cmd(ICON_FORWARD, ICON_BACKWARD, ICON_2X);
                ap_state_handling_icon_clear_cmd(ICON_PLAY, ICON_PAUSE, NULL);
                ap_state_handling_icon_clear_cmd(ICON_PLAY1, ICON_PAUSE1, NULL);
                if(g_AVI_resolution == AVI_RES_VGA)
                {
                    ap_state_handling_icon_show_cmd(ICON_PLAY, NULL, NULL);
                }
                else
                {
                    ap_state_handling_icon_show_cmd(ICON_PLAY1, NULL, NULL);
                }
                ap_browse_wav_stop();
                ap_state_audio_play_speed_set(1);
                msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_START, NULL, NULL, MSG_PRI_NORMAL);
            }
            else if ((audio_temp->result == AUDIO_ERR_DEC_FINISH) && (audio_temp->source_type == AUDIO_SRC_TYPE_APP_RS))
            {
                if (func_key_act)
                {
                    if(g_browser_reply_action_flag)
                    {
                        ap_browse_func_key_active();
                    }
                    func_key_act = 0;
                }
            }
            else
            {
                audio_confirm_handler((STAudioConfirm *)ApQ_para);
            }
            break;

        case MSG_STORAGE_SERVICE_MOUNT:
            ap_state_handling_str_draw_exit();
            ap_state_handling_storage_id_set(ApQ_para[0]);
            ap_browse_sts_set(~BROWSE_UNMOUNT);
            g_browser_reply_action_flag = 0;
            search_type = STOR_SERV_SEARCH_INIT;
            msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_BROWSE_REQ, (void *) &search_type, sizeof(INT32U), MSG_PRI_NORMAL);
            DBG_PRINT("[State Browse Mount OK]\r\n");
            break;

        case MSG_STORAGE_SERVICE_NO_STORAGE:
            g_browser_reply_action_flag = 1;
            ap_state_handling_storage_id_set(ApQ_para[0]);
            ap_browse_sts_set(BROWSE_UNMOUNT);
            ap_browse_stop_handle();

            ap_state_handling_icon_clear_cmd(ICON_PAUSE, ICON_PLAY, NULL);
            ap_state_handling_icon_clear_cmd(ICON_PLAY1, ICON_PAUSE1, NULL);
            ap_state_handling_icon_clear_cmd(ICON_PLAYBACK_PLAY, ICON_PLAYBACK_PAUSE, NULL);
            ap_state_handling_icon_clear_cmd(ICON_PLAYBACK, ICON_PLAYBACK_MOVIE, NULL);

            DBG_PRINT("ICON_PLAYBACK_CLEAR4\r\n");
            ap_state_handling_icon_show_cmd(ICON_PLAYBACK, NULL, NULL);

            DBG_PRINT("ICON_PLAYBACK_SHOW5\r\n");
            date_time_force_display(0, DISPLAY_DATE_TIME_BROWSE);
            ap_browse_volume_icon_clear_all();
            ap_browse_string_icon_clear();

            ap_browse_no_media_show(STR_NO_SD);
            DBG_PRINT("[State Browse Mount FAIL]\r\n");
            break;

        case MSG_APQ_POWER_KEY_ACTIVE:
            ap_browse_exit(MSG_APQ_POWER_KEY_ACTIVE);

            ap_state_handling_power_off(0);
            break;

        case MSG_APQ_MENU_KEY_ACTIVE:
#if 1
            if(g_browser_reply_action_flag && (ap_display_get_device() != DISP_DEV_HDMI))
            {
                OSQPost(StateHandlingQ, (void *) STATE_SETTING);
                exit_flag = EXIT_BREAK;
            }
#endif
            break;

        case MSG_APQ_MODE:
            DBG_PRINT("brose_mode_key\r\n");
            brose_flag = 0;
#if 1
            if(ap_display_get_device() != DISP_DEV_HDMI)
            {
                if(g_browser_reply_action_flag)
                {
                    OSQPost(StateHandlingQ, (void *) STATE_VIDEO_RECORD);
                    exit_flag = EXIT_BREAK;
                }
                else if(prev_state == STATE_SETTING)
                {
                    OSTimeDly(5);
                    msgQSend(ApQ, MSG_APQ_MODE, NULL, NULL, MSG_PRI_NORMAL);
                }
            }
#else
            //	ap_state_handling_str_draw_exit();
            if(g_browser_reply_action_flag && (ap_display_get_device() != DISP_DEV_HDMI))
            {
                OSQPost(StateHandlingQ, (void *) STATE_SETTING);
                exit_flag = EXIT_BREAK;
            }
#endif
            break;

        case MSG_APQ_FUNCTION_KEY_ACTIVE:

            if(g_browser_reply_action_flag)
            {
                if (func_key_act == 0)
                {
                    ap_browse_func_key_active();
                }
            }
            break;

        case MSG_STORAGE_SERVICE_BROWSE_REPLY:
            g_browser_reply_action_flag = 1;
            ap_browse_reply_action((STOR_SERV_PLAYINFO *) ApQ_para);
            DBG_PRINT("delayed_action_flag=%d\r\n", delayed_action_flag);
            if(delayed_action_flag == 1)
            {
                msgQSend(ApQ, MSG_APQ_HDMI_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
            }
            else if(delayed_action_flag == 2)
            {
                msgQSend(ApQ, MSG_APQ_HDMI_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
            }
            else if(delayed_action_flag == 3)
            {
                msgQSend(ApQ, MSG_APQ_TV_PLUG_IN, NULL, NULL, MSG_PRI_NORMAL);
            }
            else if(delayed_action_flag == 4)
            {
                msgQSend(ApQ, MSG_APQ_TV_PLUG_OUT, NULL, NULL, MSG_PRI_NORMAL);
            }
            else if(delayed_action_flag == 5)
            {
                msgQSend(ApQ, MSG_APQ_INIT_THUMBNAIL, NULL, NULL, MSG_PRI_NORMAL);
            }
            break;

        case MSG_APQ_NEXT_KEY_ACTIVE:
            if(g_browser_reply_action_flag)
            {
                ap_browse_next_key_active(ApQ_para[0]);
            }
            break;
        case MSG_APQ_FORWARD_FAST_PLAY:
            if ((ap_browse_sts_get() == 0) || (ap_browse_sts_get() & BROWSE_PLAYBACK_PAUSE))
            {

            }
            else
            {
                INT8U speed_temp;
                if(browser_play_speed >= 3)
                {
                    browser_play_speed = 3;
                    break;
                }
                browser_play_speed++;

                ap_browse_fast_play_icon_show(browser_play_speed);

                if(browser_play_speed >= 0)
                {
                    if (ap_browse_get_curr_file_type() == TK_IMAGE_TYPE_WAV)
                    {
                        ap_state_audio_reverse_set(0);
                        ap_state_audio_play_speed_set(browser_play_speed);
                    }
                    else
                    {
                        vid_dec_set_reverse_play(0);
                        vid_dec_set_play_speed(0x10000 << browser_play_speed);
                    }
                }
                else
                {
                    speed_temp = ~browser_play_speed;
                    if (ap_browse_get_curr_file_type() == TK_IMAGE_TYPE_WAV)
                    {
                        ap_state_audio_reverse_set(1);
                        ap_state_audio_play_speed_set(speed_temp);
                    }
                    else
                    {
                        vid_dec_set_reverse_play(1);
                        vid_dec_set_play_speed(0x10000 << speed_temp);
                    }
                }
            }
            break;

        case MSG_APQ_PREV_KEY_ACTIVE:
            if(g_browser_reply_action_flag)
            {
                ap_browse_prev_key_active(ApQ_para[0]);
            }
            break;
        case MSG_APQ_BACKWORD_FAST_PLAY:
            if ((ap_browse_sts_get() == 0) || (ap_browse_sts_get() & BROWSE_PLAYBACK_PAUSE))
            {
            }
            else
            {
                INT8U speed_temp;
                if(browser_play_speed <= -3)
                {
                    browser_play_speed = -3;
                    break;
                }
                browser_play_speed--;

                ap_browse_fast_play_icon_show(browser_play_speed);

                if(browser_play_speed >= 0)
                {
                    if (ap_browse_get_curr_file_type() == TK_IMAGE_TYPE_WAV)
                    {
                        ap_state_audio_reverse_set(0);
                        ap_state_audio_play_speed_set(browser_play_speed);
                    }
                    else
                    {
                        vid_dec_set_reverse_play(0);
                        vid_dec_set_play_speed(0x10000 << browser_play_speed);
                    }
                }
                else
                {
                    speed_temp = ~browser_play_speed;
                    if (ap_browse_get_curr_file_type() == TK_IMAGE_TYPE_WAV)
                    {
                        ap_state_audio_reverse_set(1);
                        ap_state_audio_play_speed_set(speed_temp);
                    }
                    else
                    {
                        vid_dec_set_reverse_play(1);
                        vid_dec_set_play_speed(0x10000 << speed_temp);
                    }
                }
            }
            break;
        case MSG_APQ_VOLUME_SHOW_END:
            ap_browse_volume_icon_clear_all();
            break;

        case MSG_APQ_MJPEG_DECODE_END:
            DBG_PRINT("Err1\r\n");
            ap_browse_mjpeg_stop();
            ap_browse_mjpeg_play_end();
            close(browse_curr_avi.file_handle);

            g_browser_reply_action_flag = 0;
            search_type = STOR_SERV_SEARCH_ORIGIN;
            msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_BROWSE_REQ, (void *)&search_type, sizeof(INT32U), MSG_PRI_NORMAL);
            break;

        case MSG_APQ_CONNECT_TO_PC:
            brose_flag = 0;
            led_type = LED_USB_CONNECT; // LED_RECORD;
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
            if(ap_display_get_device() != DISP_DEV_TFT) break;

            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_STOP, NULL, NULL, MSG_PRI_NORMAL);
            state_browse_exit(msg_id);
            OSTimeDly(100);
            /*
            	先將Display 切換到 dummy address,再啟動sensor preview
            */
            video_encode_preview_off();
            ap_state_handling_connect_to_pc(STATE_BROWSE);
            break;

        case MSG_APQ_DISCONNECT_TO_PC:
            led_type = LED_USB_DISCONNECT; // LED_RECORD;
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_START, NULL, NULL, MSG_PRI_NORMAL);
            ap_state_handling_disconnect_to_pc();
            OSTimeDly(100);
            /*
            	將sensor preview 導到 dummy address , 再還原之前Display 切換到 dummy address
            */
            vid_enc_disable_sensor_clock();
            video_encode_preview_on();
            ap_browse_init(STATE_SETTING, play_index);
            break;

#if C_BATTERY_DETECT == CUSTOM_ON
        case MSG_APQ_BATTERY_LVL_SHOW:
            DBG_PRINT("go1\r\n");
            ap_state_handling_battery_icon_show(ApQ_para[0]);
            ap_browse_display_update_command();	// give DRAW command
            break;
        case MSG_APQ_BATTERY_CHARGED_SHOW:

            DBG_PRINT("go2\r\n");
            ap_state_handling_charge_icon_show(1);
            ap_browse_display_update_command();	// give DRAW command
            break;
        case MSG_APQ_BATTERY_CHARGED_CLEAR:

            DBG_PRINT("go3\r\n");
            ap_state_handling_charge_icon_show(0);
            ap_browse_display_update_command();	// give DRAW command
            break;
        case MSG_APQ_BATTERY_LOW_SHOW:
            DBG_PRINT("");
            ap_state_handling_clear_all_icon();
            OSTimeDly(5);
            ap_state_handling_str_draw_exit();
            ap_state_handling_str_draw(STR_BATTERY_LOW, WARNING_STR_COLOR);
            msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_BATTERY_LOW_BLINK_START, NULL, NULL, MSG_PRI_NORMAL);
            break;
#endif

#if C_SCREEN_SAVER == CUSTOM_ON
        case MSG_APQ_KEY_IDLE:
            ap_state_handling_lcd_backlight_switch(0);
            break;

        case MSG_APQ_KEY_WAKE_UP:
            ap_state_handling_lcd_backlight_switch(1);
            break;
#endif

        case MSG_APQ_INIT_THUMBNAIL:
            if(ap_display_get_device() != DISP_DEV_HDMI)
            {
                if(g_browser_reply_action_flag)
                {
                    OSQPost(StateHandlingQ, (void *) STATE_THUMBNAIL);
                    exit_flag = EXIT_BREAK;
                }
                else
                {
                    delayed_action_flag = 5;
                }
            }
            break;

        case MSG_APQ_HDMI_PLUG_IN:
            if(g_browser_reply_action_flag)
            {
#if TV_DET_ENABLE
                msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_STOP, NULL, NULL, MSG_PRI_NORMAL);
                OSTimeDly(10);
#endif
                ap_browse_exit(msg_id);
                ap_state_handling_str_draw_exit();
                OSTimeDly(50);
                g_browser_reply_action_flag = 0;
                browser_play_speed = 0;
                delayed_action_flag = 0;
                ap_state_handling_hdmi_init();
                ap_browse_init(STATE_BROWSE, 0);
            }
            else
            {
                delayed_action_flag = 1;
            }
            break;

        case MSG_APQ_HDMI_PLUG_OUT:
            if(g_browser_reply_action_flag)
            {
                ap_browse_exit(msg_id);
                ap_state_handling_str_draw_exit();
                OSTimeDly(50);
                g_browser_reply_action_flag = 0;
                browser_play_speed = 0;
                delayed_action_flag = 0;
                ap_state_handling_hdmi_uninit();
#if TV_DET_ENABLE
                if(tv_plug_status_get())
                {
                    ap_state_handling_tv_init();
                }
                msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TV_POLLING_START, NULL, NULL, MSG_PRI_NORMAL);
                OSTimeDly(10);
#endif
                ap_browse_init(STATE_BROWSE, 0);
                //if(seninit_flag==0)
                msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            }
            else
            {
                delayed_action_flag = 2;
            }
            //	if(gStop_Sensor)
            //	msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
            break;

        case MSG_APQ_USER_CONFIG_STORE:
            ap_state_config_store();
            break;

        case MSG_APQ_AUDIO_EFFECT_OK:
            if ((ap_browse_sts_get() == 0) || (ap_browse_sts_get() & BROWSE_UNMOUNT))
            {
                if(g_browser_reply_action_flag)
                {
                    if(ap_state_common_handling(MSG_APQ_AUDIO_EFFECT_OK) == 0)
                    {
                        func_key_act = 1;
                    }
                }
            }
            break;

        case MSG_APQ_DISPLAY_DRAW_TIMER:
            //	set_pa_cnt(128);
            if(g_browser_reply_action_flag)
            {
                ap_browse_display_timer_draw();
            }
            break;

        case MSG_APQ_AUDIO_EFFECT_UP:
        case MSG_APQ_AUDIO_EFFECT_DOWN:
            break;

        case MSG_APQ_AUDIO_EFFECT_MODE:
        case MSG_APQ_AUDIO_EFFECT_MENU:
            mode_key_enabled = 1;
            break;

        case MSG_APQ_TV_PLUG_OUT:
        case MSG_APQ_TV_PLUG_IN:
            if(g_browser_reply_action_flag)
            {
                ap_browse_exit(msg_id);
                OSTimeDly(20);

                g_browser_reply_action_flag = 0;
                browser_play_speed = 0;
                delayed_action_flag = 0;

                if(msg_id == MSG_APQ_TV_PLUG_IN)
                {
                    ap_state_handling_tv_init();
                }
                else
                {
                    ap_state_handling_tv_uninit();
                }
                ap_browse_init(STATE_BROWSE, 0);
            }
            else
            {
                if(msg_id == MSG_APQ_TV_PLUG_IN)
                {
                    delayed_action_flag = 3;
                }
                else
                {
                    delayed_action_flag = 4;
                }
            }
            break;

        default:
            break;
        }
    }

    state_browse_exit(msg_id);
}

void state_browse_exit(INT32U next_state_msg)
{
    g_browser_reply_action_flag = 0;
    delayed_action_flag = 0;
    ap_browse_exit(next_state_msg);
    ap_state_handling_str_draw_exit(); //wwj add to prevent "No SD card" from showing too long time
    if((next_state_msg != MSG_APQ_INIT_THUMBNAIL) && (next_state_msg != MSG_APQ_CONNECT_TO_PC) && (next_state_msg != MSG_APQ_MENU_KEY_ACTIVE))
    {
        OSTimeDly(10);
        frame_mode_en = 0;
    }
    if(mode_key_enabled == 1)
    {
        audio_effect_play(EFFECT_CLICK);
        mode_key_enabled = 0;
    }
    brose_flag = 0;

    DBG_PRINT("Exit browse state\r\n");
}
