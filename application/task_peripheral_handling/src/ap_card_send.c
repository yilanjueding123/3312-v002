#include "string.h"
#include "ap_storage_service.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"
#include "ap_display.h"

#include "fs_driver.h"
#include "avi_encoder_app.h"
#include "stdio.h"
extern INT32S drvl2_sd_init(void);
extern INT32S fat_l1_cache_init(void);
extern void fs_sd_ms_plug_out_flag_reset(void);
static STR_ICON_EXT card_str;
void  ui_draw_cardtran_main();
extern INT32U copy_filename_inf(void);
#if 0
static void cpu_draw_tran_percent(INT32U val, INT32U target_buffer)
{
#if 0
    STRING_ASCII_INFO ascii_str;

    ascii_str.font_color = 0xffff;
    ascii_str.font_type = 0;
    ascii_str.pos_x = 0;
    ascii_str.pos_y = 0;

    ascii_str.str_ptr = "fankun";
    card_str.pos_y = 10;
    card_str.pos_x = 10;

    ascii_str.buff_h = card_str.h = ASCII_draw_char_height;
    ascii_str.buff_w = card_str.w = ASCII_draw_char_width * 6;
    ap_state_handling_ASCII_str_draw(&card_str, &ascii_str);
#endif
#if 1
    STRING_INFO str_info = {0};
    INT16U temp;

    str_info.language = LCD_EN;
    str_info.font_color = WHITE_COLOR;	//white
    str_info.pos_x = 130;
    str_info.pos_y = 110;
    str_info.buff_w = TFT_WIDTH;
    str_info.buff_h = TFT_HEIGHT;

    temp = val / 10;
    ap_state_resource_char_draw(temp + 0x30, (INT16U *) target_buffer, &str_info, RGB565_DRAW, 0);
    temp = val - temp * 10;
    ap_state_resource_char_draw(temp + 0x30, (INT16U *) target_buffer, &str_info, RGB565_DRAW, 0);


#endif

}
#endif
static void init_sd_io(void)
{
    gpio_drving_init_io(IO_B14, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_B15, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_B10, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_B11, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_B12, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_B13, (IO_DRV_LEVEL) SD_DRIVING);

    gpio_drving_init_io(IO_D10, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_D11, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_D12, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_D13, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_D14, (IO_DRV_LEVEL) SD_DRIVING);
    gpio_drving_init_io(IO_D15, (IO_DRV_LEVEL) SD_DRIVING);

}
static void sd_choice(INT32U sd_pos)
{
    INT8S ret;
    INT8U type;
    msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
    drvl2_sd_card_remove();
    R_FUNPOS0 &= (~0x1E00000);
    if(sd_pos == SDC0_IOB14_IOB15_IOB10_IOB11_IOB12_IOB13)
    {
        SD_USED_NUM = 0;
        R_FUNPOS0 |= 0x0200000;
        R_IOB_DIR &= ~0x7C00;
        R_IOB_ATT &= ~0x7C00;
        R_IOB_O_DATA |= 0x7C00;


    }
    else
    {
        SD_USED_NUM = 1;
        R_FUNPOS0 |= 0x0000000;
        R_IOD_DIR &= ~0xF400;
        R_IOD_ATT &= ~0xF400;
        R_IOD_O_DATA |= 0xF400;


    }
    ret = drvl2_sd_init();
    if(ret == 0)
    {
    }
    fs_sd_ms_plug_out_flag_reset();
    ap_storage_service_usb_plug_in();
    DBG_PRINT("AA\r\n");
    type = 0x55;
    msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_START, &type, sizeof(INT8U), MSG_PRI_NORMAL);
    OSTimeDly(10);//10
    watchdog_clear();
    //看门狗不清会死

}

//struct f_info	file_info;
INT32U *data_buffer;
INT32U file_total_size;
INT32U file_inf;

void copy_data(INT8U *result_file, INT8U *read_file, INT32U size)
{
    while(size --)
    {
        *result_file++ = *read_file++;
    }
}
INT8S  Write_Sector(INT32U size, INT32U seek)
{
    INT16S fd;
    INT32U addr, i;
    INT32U write_size;
    INT32U seek_cnt;
    //	INT8U  B_FileName_Path[4+6+8+4+1];
    //	gp_strcpy((INT8S*)B_FileName_Path,(INT8S*)"C:\\DCIM\\");
    //    gp_strcat((INT8S*)B_FileName_Path,(INT8S*)file_info.f_name);
    msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
    if(seek)
    {
#if 0
        seek_cnt = seek * 1024 * 2560;
        fd = open((CHAR *)B_FileName_Path, O_WRONLY);
        if(seek_cnt > (file_total_size / 2))
            lseek(fd, file_total_size - seek_cnt, SEEK_END);
        else
            lseek(fd, seek_cnt, SEEK_SET);
#endif
        fd = open((CHAR *)file_inf, O_WRONLY);
        lseek(fd, 0, SEEK_END);
    }
    else
    {
        mkdir("C:\\DCIM");
        chdir("C:\\DCIM");
        fd = open((CHAR *)file_inf, O_RDWR | O_TRUNC | O_CREAT);
    }
    if(fd < 0)
    {

        sd_choice(SDC1_IOD10_IOD11_IOD12_IOD13_IOD14_IOD15);
        return -1;
    }
    //addr = (INT32U)gp_malloc(size);

    //if(!addr)
    //    {
    //	 close(fd);
    //	 sd_choice(SDC1_IOD10_IOD11_IOD12_IOD13_IOD14_IOD15);
    //	 return -1;
    //	}


    //	copy_data((INT8U*)addr, (INT8U*)data_buffer,size);
    write_size = write(fd, (INT32U)data_buffer, size);
    // DBG_PRINT("Write:%d\r\n",write_size);
    //OSTimeDly(1);
    //lseek(fd, size/2, SEEK_CUR);
    //write(fd, (INT32U)(data_buffer+(size/2)), size/2);
    close(fd);
    sd_choice(SDC1_IOD10_IOD11_IOD12_IOD13_IOD14_IOD15);
    //gp_free((void*) addr);
    return 0;
}
static INT32S card_ptr;
INT32U card_display_buf;
extern INT32U display_isr_queue[];
#if 0
void show_jdu(INT8U step)
{
    INT8U i;
    STRING_INFO str_info = {0};
    static INT8U flash_cnt = 0;
    str_info.language = LCD_EN;
    str_info.font_color = RED_COLOR;	//white
    str_info.pos_x = 124;
    str_info.pos_y = 130;
    str_info.buff_w = TFT_WIDTH;
    str_info.buff_h = TFT_HEIGHT;
    for(i = 0; i < (step / 20); i++)
        ap_state_resource_char_draw(45, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
    if(flash_cnt == 0)
    {
        flash_cnt = 1;
        ap_state_resource_char_draw(45, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
    }
    else
    {
        flash_cnt = 0;
    }

}
#endif
INT8S Card_ReadSenddata_main(void)
{

    INT32U k, i;
    //INT8U	FileName_Path[4+6+8+4+1];
    INT16S fd, nRet;
    INT8S  ret;
    INT32U dipaly_buff, val_value, prev_val_value = 101;
    struct stat_t statetest;
    STRING_INFO str_info = {0};
    INT32U temp;
    INT16U INDEX, TYPE;
    // cpu_draw_tran_percent();
    file_inf = copy_filename_inf();
#if 0
    TYPE = (INT16U)(file_inf & 0x000000ff);
    INDEX = (INT16U)((file_inf & 0xffffff00) >> 8);
    if(TYPE == 1)//jpg
        sprintf((char *)FileName_Path, (const char *)"PICT%04d.avi", g_play_index);
    else if(TYPE == 3)//avi
        sprintf((char *)FileName_Path, (const char *)"MOVI%04d.avi", g_play_index);




    DBG_PRINT("-----------copy_file_start------------\r\n");
    gp_strcpy((INT8S *)FileName_Path, (INT8S *)"C:\\DCIM\\MOVI");
    gp_strcat((INT8S *)FileName_Path, (INT8S *)"*.avi");
    nRet = _findfirst((CHAR *)FileName_Path, &file_info , D_ALL);
    if (nRet < 0)
    {

        return -1 ;
    }
    gp_strcpy((INT8S *)FileName_Path, (INT8S *)"C:\\DCIM\\");
    gp_strcat((INT8S *)FileName_Path, (INT8S *)file_info.f_name);
    fd = open((CHAR *)FileName_Path, O_RDONLY);
#endif
    fd = open((CHAR *) file_inf, O_RDONLY);

    if (fd < 0)
    {

        return -1 ;
    }

    if (fstat(fd, &statetest))
    {
        close(fd);
        return -1;
    }
    file_total_size = statetest.st_size;


    //申请5120K空间
    data_buffer = (INT32U *) gp_malloc(1024 * 5120);
    if (!data_buffer)
    {
        gp_free((void *)data_buffer);
        DBG_PRINT("space_fail\r\n");
        return (-1);
    }
    str_info.language = LCD_EN;
    str_info.font_color = WHITE_COLOR;	//white
    str_info.pos_x = 140;
    str_info.pos_y = 110;
    str_info.buff_w = TFT_WIDTH;
    str_info.buff_h = TFT_HEIGHT;
    for(k = 0; k < file_total_size / (1024 * 5120); k++)
    {

        msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
        if (read(fd, (INT32U)data_buffer, 1024 * 5120) <= 0)
        {
            gp_free((void *)data_buffer);
            close(fd);
            return (-1);
        }
        //现在切换卡到备份卡
        // DBG_PRINT("percent:%d%\r\n",100*k/(file_total_size/(1024*2560)));

        val_value = 100 * k / (file_total_size / (1024 * 5120));
        if(val_value != prev_val_value)
        {
            str_info.pos_x = 140;
            ui_draw_cardtran_main();
            prev_val_value = val_value;
            //cpu_draw_tran_percent(val_value,card_display_buf);
            temp = val_value / 10;
            ap_state_resource_char_draw(temp + 0x30, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
            temp = val_value - temp * 10;
            ap_state_resource_char_draw(temp + 0x30, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);

            ap_state_resource_char_draw(37, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
            // show_jdu(val_value);
        }
        sd_choice(SDC0_IOB14_IOB15_IOB10_IOB11_IOB12_IOB13);
        ret = Write_Sector(1024 * 5120, k);
        if(ret == -1)
        {
            gp_free((void *)data_buffer);
            close(fd);
            return (-1);
        }
        // DBG_PRINT("yy\r\n");
    }
#if 1
    if(file_total_size % (1024 * 5120) != 0)
    {
        if (read(fd, (INT32U)data_buffer, file_total_size % (1024 * 5120)) <= 0)
        {
            gp_free((void *)data_buffer);
            close(fd);
            return (-1);
        }
        //现在切换卡到备份卡
        //  DBG_PRINT("zz\r\n");

        // ap_state_resource_char_draw(0x25, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);

        sd_choice(SDC0_IOB14_IOB15_IOB10_IOB11_IOB12_IOB13);
        ret = Write_Sector(file_total_size % (1024 * 5120), k);
        if(ret == -1)
        {
            gp_free((void *)data_buffer);
            close(fd);
            return (-1);
        }
        DBG_PRINT("aa\r\n");

        // ui_draw_cardtran_main();

        //ap_state_resource_char_draw(79, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
        //ap_state_resource_char_draw(75, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
        //OSTimeDly(100);
    }
#endif
    gp_free((void *)data_buffer);
    close(fd);
    return 0;
}


void  ui_draw_cardtran_main()
{
    INT16U ui_fd;
    INT32U  size;
    INT32S  status;
    ui_fd = nv_open((INT8U *) "CARDTRAN.JPG");
    if(ui_fd != 0xffff)
    {
        do
        {

            card_display_buf = ap_display_queue_get(display_isr_queue);
            OSTimeDly(5);

        }
        while(!card_display_buf);

        size = nv_rs_size_get(ui_fd);
        card_ptr = (INT32S) gp_malloc(size);
        if(card_ptr)
        {
            if (nv_read(ui_fd, (INT32U) card_ptr, size) == 0)
            {

                IMAGE_DECODE_STRUCT img_info;
                img_info.image_source = (INT32S) card_ptr;
                img_info.source_size = size;
                img_info.source_type = TK_IMAGE_SOURCE_TYPE_BUFFER;
                img_info.output_format = C_SCALER_CTRL_OUT_RGB565;
                img_info.output_ratio = 0;
                img_info.out_of_boundary_color = 0x008080;
                if(ap_display_get_device() == DISP_DEV_TFT)
                {
                    //TFT
                    img_info.output_buffer_width = TFT_WIDTH;
                    img_info.output_buffer_height = TFT_HEIGHT;
                    img_info.output_image_width = TFT_WIDTH;
                    img_info.output_image_height = TFT_HEIGHT;
                }
                img_info.output_buffer_pointer = card_display_buf;
                status = jpeg_buffer_decode_and_scale(&img_info);
            }

            if(status != STATUS_FAIL)
            {
                OSQPost(DisplayTaskQ, (void *) (card_display_buf | MSG_DISPLAY_TASK_MJPEG_DRAW));
                // OSTimeDly(100);
            }
            gp_free((void *) card_ptr);
        }
    }

}
//extern void ap_video_record_resolution_display();
void Card_trans_loop(void)
{
    INT8S ret;
    INT8U type;
    STRING_INFO str_info = {0};
    str_info.language = LCD_EN;
    str_info.font_color = WHITE_COLOR;	//white
    str_info.pos_x = 140;
    str_info.pos_y = 110;
    str_info.buff_w = TFT_WIDTH;
    str_info.buff_h = TFT_HEIGHT;
    ap_state_handling_str_draw_exit();
    ap_state_handling_clear_all_icon();
    OSTimeDly(3);
    vid_enc_disable_sensor_clock();
    msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
    init_sd_io();
    //  ap_video_record_resolution_display();
    // cpu_draw_tran_percent();
    //   ui_draw_cardtran_main();

    //OSQPost(DisplayTaskQ, (void *) MSG_DISPLAY_TASK_EFFECT_INIT);
    // cpu_draw_tran_percent();
    //
    //OSQPost(DisplayTaskQ, (void *) (card_display_buf|MSG_DISPLAY_TASK_MJPEG_DRAW));
    // ap_state_handling_icon_show_cmd(ICON_MP3_PAUSE, NULL, NULL);
    // OSTimeDly(10);
    type = DISABLE_KEY;
    msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
    ret = Card_ReadSenddata_main();

    ui_draw_cardtran_main();
    //卡传输失败
    if(ret == -1)
    {
        ap_state_resource_char_draw(70, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
        ap_state_resource_char_draw(65, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
        ap_state_resource_char_draw(73, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
        ap_state_resource_char_draw(76, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
        DBG_PRINT("card_copy_fail\r\n");
    }
    else
    {

        ap_state_resource_char_draw(79, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);
        ap_state_resource_char_draw(75, (INT16U *) card_display_buf, &str_info, RGB565_DRAW, 0);

        DBG_PRINT("card_copy_suc\r\n");
    }
    type = GENERAL_KEY;
    msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
    OSTimeDly(100);
}


