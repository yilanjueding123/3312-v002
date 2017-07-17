#include "my_avi_encoder_state.h"


#if ENABLE_DYNAMIC_TUNING_JPEG_Q
extern INT32S current_Y_Q_value;
extern INT32S target_Y_Q_value;

extern INT32S current_UV_Q_value;
extern INT32S target_UV_Q_value;

extern INT32U current_VLC_size;
extern INT32U max_VLC_size;
#endif
/****************************************************************************/
/*
 *	Dynamic_Tune_Q
 */
#define Q_Y_STEP 5
#define Q_UV_STEP 10
#define Q_DISTANCE	15//20

// 1080P/1080FHD
#define Q_LARGE_Y_MAX 		60//70
#define Q_LARGE_Y_MIN		30//10
#define Q_LARGE_UV_MAX 	45//(Q_LARGE_Y_MAX-Q_DISTANCE)
#define Q_LARGE_UV_MIN 	30//10
// 720P/WVGA/VGA
#define Q_SMALL_Y_MAX		70
#define Q_SMALL_Y_MIN		35
#define Q_SMALL_UV_MAX		(Q_SMALL_Y_MAX-Q_DISTANCE)
#define Q_SMALL_UV_MIN		30

static void jpeg_Q_adjust(INT32S val)
{

    if (val < 0)
    {
        if ((current_Y_Q_value - current_UV_Q_value) >= Q_DISTANCE)
        {
            current_Y_Q_value -= Q_Y_STEP;
        }
        else
        {
            current_UV_Q_value -= Q_UV_STEP;
        }
    }
    else if (val > 0)
    {
        if ((current_Y_Q_value - current_UV_Q_value) >= Q_DISTANCE)
        {
            current_UV_Q_value += Q_UV_STEP;
        }
        else
        {
            current_Y_Q_value += Q_Y_STEP;
        }
    }


}

void Dynamic_Tune_Q(INT32U jpeg_size, INT32U full_size_flag)
{

    if ( (my_pAviEncVidPara->encode_width == AVI_WIDTH_1080FHD) || (my_pAviEncVidPara->encode_width == AVI_WIDTH_1080P) )
    {
        if (full_size_flag)
        {
            current_Y_Q_value -= Q_DISTANCE;
            current_UV_Q_value -= Q_DISTANCE;
        }
        else if (jpeg_size > (max_VLC_size - (30 * 1024)))
        {
            jpeg_Q_adjust(-1);
        }
        else if (jpeg_size < (max_VLC_size - (60 * 1024)))
        {
            jpeg_Q_adjust(+1);
        }

        if  ( current_Y_Q_value < Q_LARGE_Y_MIN )
        {
            current_Y_Q_value = Q_LARGE_Y_MIN;
        }
        if  ( current_UV_Q_value < Q_LARGE_UV_MIN )
        {
            current_UV_Q_value = Q_LARGE_UV_MIN;
        }
        if  (current_Y_Q_value > Q_LARGE_Y_MAX)
        {
            current_Y_Q_value = Q_LARGE_Y_MAX;
        }
        if  ( current_UV_Q_value > Q_LARGE_UV_MAX )
        {
            current_UV_Q_value = Q_LARGE_UV_MAX;
        }
    }
    else	 // other size (720P, WVGA, VGA)
    {
        if (full_size_flag)
        {
            current_Y_Q_value -= Q_DISTANCE;
            current_UV_Q_value -= Q_DISTANCE;
        }
        else if (jpeg_size > (max_VLC_size - (30 * 1024)))
        {
            jpeg_Q_adjust(-1);
        }
        else if (jpeg_size < (max_VLC_size - (60 * 1024)))
        {
            jpeg_Q_adjust(1);
        }

        if  ( current_Y_Q_value < Q_SMALL_Y_MIN )
        {
            current_Y_Q_value = Q_SMALL_Y_MIN;
        }
        if  ( current_UV_Q_value < Q_SMALL_UV_MIN )
        {
            current_UV_Q_value = Q_SMALL_UV_MIN;
        }
        if   (current_Y_Q_value > Q_SMALL_Y_MAX)
        {
            current_Y_Q_value = Q_SMALL_Y_MAX;
        }
        if  ( current_UV_Q_value > Q_SMALL_UV_MAX )
        {
            current_UV_Q_value = Q_SMALL_UV_MAX;
        }
    }
}