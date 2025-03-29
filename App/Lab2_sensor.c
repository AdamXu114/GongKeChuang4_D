#include "Lab2.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "systick.h"
#include "Ex3.h"
#include "Ex7.h"

extern uint16_t RES_value;
extern image_t image_use;					// original image fit to LCD resolution
extern image2_t image2_use;				// use 1/3 of the original image (40 continuous lines in the middle)
extern image_t image_show;					// resized CAM data to fit LCD
extern image2_t image2_show;				// show 1/3 of the full size screen
extern image_t image_temp;					// resized CAM data to fit LCD
extern image2_t image2_temp;				// show 1/3 of the full size screen
extern image_t image_gauss, image_binary;
extern image2_t image2_gauss, image2_binary;
extern uint8_t image_ready;				// MT9V034 new data frame ready in buffer

extern uint8_t image_size;				// 0: full size; 1: half size; 2: 1/3 sized

extern uint16_t loop_count;
extern uint16_t pit_counter1;
extern uint32_t sys_tick_counter;

__attribute__((aligned(32))) extern uint8_t img[35840];
__attribute__((aligned(32))) extern uint8_t img2[12800];
void Lab2_mma8451_daq(void)
{
	
}

void Lab2_mma8451_gradienter(void)
{
	
}

void Lab2_mma8451_pedometer(void)
{
	
}

void image_process(void){
    uint8_t mode;	
	uint8_t i, j, threshold;
	uint8_t THR, dT=0;
	char line[20];
	
	image_size = 2;         // full-sized image
	image_ready = RESET;
	hsp_tft18_clear(BLACK);
	hsp_tft18_show_str(0, 0, "Select mode by SW1~4");
    hsp_tft18_show_str(0, 1, "mode:");
	
	while(1)
	{   delay_1ms(10);
        loop_count++;
        if(sys_tick_counter >= 1000)
      {
          sys_tick_counter = 0;
          hsp_tft18_show_str(0,2,"Loop:");
          hsp_tft18_show_uint16(30, 2, loop_count);
          hsp_tft18_show_str(70,2,"Pit:");
          hsp_tft18_show_uint16(100, 2, pit_counter1);
          pit_counter1 = 0;
          loop_count = 0;
      }
		mode = hsp_get_taskid();
        //hsp_tft18_show_uint8(1, 30, mode);
		if(image_ready == SET)
		{
			switch(mode)
			{
				case 0:		// 显示原始图像
					hsp_image2_show_dma(image2_use);
					break;
				case 1:		// 调试二值化，在不按下S1或S2时，旋转调整阈值
                if(!S1() || !S2())
                {
                   if(!S1())
                       {
                           //mode = 1;
                           threshold = hsp_image2_threshold_otsu(image2_use);
                       }
                   if(!S2())
                       {
                           //mode = 2;
                           threshold = hsp_image2_threshold_mean(image2_use);
                       }
                       
                       if((255-threshold) < dT)
                           THR = 255;
                       else
                           THR = threshold + dT;
                   
                       for(j=0; j<IMAGEH2; j++)
                       {
                           for(i=0; i<IMAGEW2; i++)
                               //image2_show[j][i] = (image2_use[j][i]>threshold) ? WHITE : BLACK;
                               image2_show[j][i] = (image2_use[j][i]>THR) ? WHITE : BLACK;
                       }
                   }
                else
                {
                   //mode = 0;
                   //上课改的
                   if (RES_value>255)RES_value=255;
                   threshold = RES_value;
                   hsp_tft18_show_uint8(0,0,threshold);
                   for(j=0; j<IMAGEH2; j++)
                       {
                           for(i=0; i<IMAGEW2; i++)
                               image2_show[j][i] = (image2_use[j][i]>threshold) ? WHITE : BLACK;
                               //image2_show[j][i] = (image2_use[j][i]>THR) ? WHITE : BLACK;
                       }
                       //到这结束
       
                   // for(j=0; j<IMAGEH; j++)
                   // {
                   //    for(i=0; i<IMAGEW; i++)
                   //       image2_show[j][i] = image2_use[j][i];
                   // }
                }
                   hsp_image2_show_dma(image2_show);
                   image_ready = RESET;
					break;
                case 2:		// Otus binarized
					threshold = hsp_image2_threshold_otsu(image2_use);
					hsp_image2_binary(image2_use, image2_show, threshold);
					//添加腐蚀
					if(!S1() || !S2()){
						if(!S1())
						hsp_image2_erode(image2_show, image2_temp);
						if(!S2())
						hsp_image2_dilate(image2_show, image2_temp);
						hsp_image2_show_dma(image2_temp);
					}
					else {
						hsp_image2_show_dma(image2_show);
					}
					break;
				case 3:		// 大津+腐蚀+膨胀
                    threshold = hsp_image2_threshold_mean(image2_use);
                    hsp_image2_binary(image2_use, image2_show, threshold);
                    hsp_image2_erode(image2_show, image2_temp);
                    hsp_image2_dilate(image2_temp, image2_show);
                    hsp_image2_show_dma(image2_show);
                case 4:		// 大津+腐蚀+膨胀+Sobel1本来可以，后来不知道为什么图像有闪烁
                    
                    hsp_image2_fast_gauss_conv(image2_use,image2_gauss);
                    threshold = hsp_image2_threshold_otsu(image2_gauss);
                    hsp_image2_binary(image2_gauss, image2_binary, threshold);
                    hsp_image2_erode(image2_binary, image2_temp);
                    hsp_image2_dilate(image2_temp, image2_show);
                    hsp_image2_sobel1(image2_show, image2_temp);
                    hsp_image2_show_dma(image2_temp);
                case 5:		// 大津+腐蚀+膨胀+Sobel2  可以
                    
                    hsp_image2_fast_gauss_conv(image2_use,image2_gauss);
                    threshold = hsp_image2_threshold_otsu(image2_gauss);
                    hsp_image2_binary(image2_gauss, image2_binary, threshold);
                    hsp_image2_erode(image2_binary, image2_temp);
                    hsp_image2_dilate(image2_temp, image2_show);
                    hsp_image2_sobel2(image2_show, image2_temp);
                    hsp_image2_show_dma(image2_temp);
                case 6:		// 大津+腐蚀+膨胀+Canny 本来可以，后来不知道为什么图像有闪烁
                    
                    hsp_image2_fast_gauss_conv(image2_use,image2_gauss);
                    threshold = hsp_image2_threshold_otsu(image2_gauss);
                    hsp_image2_binary(image2_gauss, image2_binary, threshold);
                    hsp_image2_erode(image2_binary, image2_temp);
                    hsp_image2_dilate(image2_temp, image2_show);
                    hsp_image2_canny(image2_show, image2_temp, 20, 80);
                    hsp_image2_show_dma(image2_temp);
                case 13:		// sobel二值化 效果最好
                    hsp_image2_fast_gauss_conv(image2_use,image2_gauss);
                    hsp_image2_binary_sobel(image2_gauss, image2_show);
                    hsp_image2_show_dma(image2_show);
                case 14:		// 大津+腐蚀+膨胀+Canny 效果可行
                    hsp_image2_fast_gauss_conv(image2_use,image2_gauss);
                    threshold = hsp_image2_threshold_otsu(image2_gauss);
                    hsp_image2_binary(image2_gauss, image2_show, threshold);
                    hsp_image2_canny(image2_show, image2_temp, 20, 80);
                    hsp_image2_show_dma(image2_temp);
				default:
					break;
			}
			image_ready = RESET;
		}

		if(!S3()) break;
	}
	
	while(!S3()) {}
}