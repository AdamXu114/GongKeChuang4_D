// https://blog.csdn.net/m0_53966219/article/details/126711218
// https://blog.csdn.net/zhuoqingjoking97298/article/details/120093315
// PID: https://zhuanlan.zhihu.com/p/586532545?utm_id=0
// https://blog.csdn.net/weixin_42208428/article/details/122173575
// https://blog.csdn.net/weixin_43964993/article/details/112383192

#include "Project1.h"

extern image2_t image2_use;			// use 1/3 of the original image (40 continuous lines in the middle)
extern image2_t image2_show;		// show 1/3 of the full size screen
extern image2_t image2_temp;		// show 1/3 of the full size screen
extern uint8_t image_ready;			// MT9V034 new data frame ready in buffer
extern uint8_t image_size;				// 0: full size; 1: half size; 2: 1/3 sized
extern int16_t encoder_speed;

float kp_pw = 6, ki_pw = 0, kd_pw = 14;

// Project#1: Line Following Robot (LFR)
void Project_LFR(void)
{
	uint16_t pw = 1500, pwt = 0;
	uint16_t dc = 0;
	uint16_t tloss = 0;				// target lost loop counter
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;
	
	image_size = 2;         // use 1/3 of the full size
	MEN_HIGH();					// enable H-bridge

	state_pha = PHA2();			state_phb = PHB2();
	state_pha_t = state_pha;	state_phb_t = state_phb;
	image_ready = RESET;
	
	hsp_tft18_clear(BLACK);
	
	while(1)
	{
//		if (!PUSH())			// push button pressed        
//		{
//			//delay_1ms(50);		// de-jitter
//			if (!PUSH())
//			{
//				while(!PUSH());
//				dc = 0;
//			}
//		}
//
//		state_pha = PHA2();			state_phb = PHB2();
//		if((state_pha_t != state_pha) || (state_phb_t != state_phb))
//		{
//			if(state_phb_t == state_phb)
//			{
//				if(SET == state_phb)
//				{
//					if(RESET == state_pha) dc++;
//					else if(0 < dc) dc--;
//				}
//				else
//				{
//					if(SET == state_pha) dc++;
//					else if(0 < dc) dc--;
//				}
//			}
//			else
//			{
//				if(SET == state_pha)
//				{
//					if(SET == state_phb) dc++;
//					else if(0 < dc) dc--;
//				}
//				else
//				{
//					if(RESET == state_pha) dc++;
//					else if(0 < dc) dc--;
//				}
//			}
//			state_pha_t = state_pha;
//			state_phb_t = state_phb;
//            //delay_1ms(10);		// de-jitter
//		}
//		// PWM output stage, subjected to duty cycle limits
//		if(35 < dc)
//			dc = 35;
//		if(SW2())
//		{
//			hsp_motor_voltage(MOTORF, dc);		// run forward
//		}
//		else
//		{
//			hsp_motor_voltage(MOTORB, dc);		// run backward
//		}
		
		// camera image processing
		if(image_ready == SET)
		{
			//threshold = hsp_image2_threshold_otsu(image2_use);
			//threshold = hsp_image2_threshold_mean(image2_use);
			//threshold = hsp_image2_threshold_minmax(image2_use);
			//hsp_image2_show_dma(image2_use);
			//hsp_image2_show_dma(image2_show);
			//hsp_image2_binary_minmax(image2_use, image2_temp);
			hsp_image2_binary_sobel(image2_use, image2_temp);

			pw = hsp_image_judge2(image2_temp);
			if(pw == 0)
			{
				tloss++;
				pw = pwt;		// use previous result
				if(tloss > 10)	// off-road protection
				{
					dc = 0;
					pw = 1500;
					tloss = 0;
				}
			}
			else
			{
				tloss = 0;
			}
			
			// apply steering angle limits
			if(1700 < pw)
				pw = 1700;
			if(1300 > pw)
				pw = 1300;
			if(pwt != pw)
			{
				hsp_servo_angle(SERVO1, pw);
				hsp_servo_angle(SERVO2, pw);
				hsp_servo_angle(SERVO3, pw);
				hsp_servo_angle(SERVO4, pw);
				pwt = pw;
			}
			hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
			hsp_tft18_show_int16(0, 0, pw);
			hsp_tft18_show_int16_color(56, 0, encoder_speed, WHITE, BLACK);
			
			if(!SW1())
			{
				hsp_image2_show_dma(image2_use);
			}
			else
			{
				hsp_image2_show_dma(image2_temp);
			}
			image_ready = RESET;
		}
		
		if(!S3()) break;
	}
	
	hsp_servo_angle(SERVO1, 1500);
	hsp_servo_angle(SERVO2, 1500);
	hsp_servo_angle(SERVO3, 1500);
	hsp_servo_angle(SERVO4, 1500);
}

uint16_t hsp_image_judge2(image2_t image)
{
	uint16_t pw = 1500;
	uint8_t up_black_num = 0;
	uint8_t mid_black_num = 0;
	uint8_t low_black_num = 0;
	uint8_t up_left = 255, up_right = 255, mid_left = 255, mid_right = 255, low_left = 255, low_right = 255;	// 255 is an invalid value
	uint8_t up_mid_index = 255, mid_mid_index = 255, low_mid_index = 255;	// 255 is an invalid value
	uint8_t k;
	static uint8_t mid_index = 0, last_mid_index = 0;


	// 检测上、中、下三条线的边缘线
	for (uint8_t i = 1; i < IMAGEW2-1; i++)
	{
		if(image[5][i] == 0) 
		{
			up_black_num++;
			if(image[5][i-1] != 0)	// 检测到边缘
			{
				if(up_left == 255) up_left = i;
				else up_right = i;
			}
		}
		if(image[20][i] == 0) 
		{
			mid_black_num++;
			if(image[20][i-1] != 0)	// 检测到边缘
			{
				if(mid_left == 255) mid_left = i;
				else mid_right = i;
			}
		}
		if(image[35][i] == 0) 
		{
			low_black_num++;
			if(image[35][i-1] != 0)	// 检测到边缘
			{
				if(low_left == 255) low_left = i;
				else low_right = i;
			}
		}
	}
	// 显示上、中、下三条线的黑点数
	hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
	hsp_tft18_show_uint8(60, 0, up_black_num);
	hsp_tft18_show_uint8(60, 1, mid_black_num);
	hsp_tft18_show_uint8(60, 2, low_black_num);

	// 判断三条线数据的有效性
	if(up_black_num <= 10 && up_black_num >2)
	{
		if(up_right == 255)
		{
			if(up_left < 94) 
			{
				if(up_left < 10) up_mid_index = 0;
				else up_mid_index = up_left / 2;
			}
			else
			{
				if(up_left > 177) up_mid_index = 187;
				else up_mid_index = (up_left+188) / 2;
			}
		}
		else up_mid_index = (up_left + up_right) / 2;
	}
	if(mid_black_num <= 10 && mid_black_num >2)
	{
		if(mid_right == 255)
		{
			if(mid_left < 94) 
			{
				if(mid_left < 10) mid_mid_index = 0;
				else mid_mid_index = mid_left / 2;
			}
			else
			{
				if(mid_left > 177) mid_mid_index = 187;
				else mid_mid_index = (mid_left+188) / 2;
			}
		}
		else mid_mid_index = (mid_left + mid_right) / 2;
	}
	if(low_black_num <= 10 && low_black_num >2)
	{
		if(low_right == 255)
		{
			if(low_left < 94) 
			{
				if(low_left < 10) low_mid_index = 0;
				else low_mid_index = low_left / 2;
			}
			else
			{
				if(low_left > 177) low_mid_index = 187;
				else low_mid_index = (low_left+188) / 2;
			}
		}
		else low_mid_index = (low_left + low_right) / 2;
	}
	
	if(up_mid_index != 255) mid_index = up_mid_index;
	else if(mid_mid_index != 255) mid_index = mid_mid_index;
	else if(low_mid_index != 255) mid_index = low_mid_index;
	else mid_index = last_mid_index;

	int8_t cur_pw_error = 94 - mid_index;						//
	int8_t last_pw_error = 94 - last_mid_index;					//
	int8_t diff_pw_error = cur_pw_error - last_pw_error;		//
	pw = 1500 + kp_pw * cur_pw_error + kd_pw * diff_pw_error;
	last_mid_index = mid_index;
	
	return pw;
}

uint16_t hsp_image_judge(image2_t image)
{
	uint16_t pw;			// pulse-width control steering angle
	uint8_t i, j;
	uint8_t gte_l, gte_r, gte_ok;				// guide tape edge flag
	uint8_t gte_l_idx, gte_r_idx, gte_c_idx;		// guide tape index
	
	gte_l = RESET;
	gte_r = RESET;
	gte_ok = RESET;
	for(i=2; i<(IMAGEW2-2); i++)
	{
		if(RESET == gte_l)
		{
			if((255 == image[20][i]) && (0 == image[20][i+1]))	// left edge found
			{
				gte_l = SET;
				gte_l_idx = i;									// left edge index
			}
		}
		if((SET == gte_l) && (RESET == gte_r))
		{
			if((0 == image[20][i]) && (255 == image[20][i+1]))	// right edge found
			{
				gte_r = SET;
				gte_r_idx = i;									// right edge index
			}
		}
		if((SET == gte_l) && (SET == gte_r) && (RESET == gte_ok))		// both edges found
		{
			if(((gte_r_idx - gte_l_idx) > 6) && ((gte_r_idx - gte_l_idx) < 30))		// proper tape width
			{
				gte_ok = SET;
				gte_c_idx = (gte_r_idx + gte_l_idx) >> 1;	// tape center index
			}
			else
			{
				gte_l = RESET;
				gte_r = RESET;
				gte_ok = RESET;
			}
		}
	}
	
	if(SET == gte_ok)
		pw = 1500 + 10 * (94 - gte_c_idx);
	else
		pw = 0;
	
	return pw;
}
