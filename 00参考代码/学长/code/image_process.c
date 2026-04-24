/*
 * image_process.c
 *
 *  Created on: 2025年2月25日
 *      Author: 15958
 */
#include "image_process.h"
#include "zf_driver_delay.h"
#include "zf_common_headfile.h"

IFX_ALIGN(4) uint8 Image_Use[LCDH][LCDW];      //用来存储压缩之后灰度图像的二维数组
IFX_ALIGN(4) uint8 Pixle[LCDH][LCDW];          //图像处理时真正处理的二值化图像数组
IFX_ALIGN(4) uint8 Pixle_hb[LCDH][LCDW];
uint8 Threshold;                                //通过大津法计算出来的前20行二值化阈值

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Compress
//  @brief          原始灰度图像压缩处理
//  @brief          作用就是将原始尺寸的灰度图像压缩成你所需要的大小，这里我是把原始80行170列的灰度图像压缩成60行80列的灰度图像。
//  @brief          为什么要压缩？因为我不需要那么多的信息，60*80图像所展示的信息原则上已经足够完成比赛任务了，当然你可以根据自己的理解修改。
//  @parameter      void
//  @return         void
//  @time           2022年1月18日
//  @Author         陈海涛
//  Sample usage:   Image_Compress();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Image_Compress(void)
{
    //const static int Y_OFFSITE = 0;
    int XSITE, YSITE;
    for (YSITE = 0; YSITE < LCDH; YSITE++)
    {
        for (XSITE = 0; XSITE < LCDW; XSITE++)
        {
           // Image_Use[YSITE][XSITE] = mt9v03x_image[(int)(1.7*YSITE)][(int)(1.0*XSITE * 2.35)];
            Image_Use[YSITE][XSITE] = mt9v03x_image[(int)(1.0*YSITE*1.2)][(int)(1.0*XSITE * 2.35)];
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_Threshold
//  @brief          优化之后的的大津法。大津法就是一种能够算出一幅图像最佳的那个分割阈值的一种算法。
//  @brief          这个东西你们可以如果实在不能理解就直接拿来用，什么参数都不用修改，只要没有光照影响，那么算出来的这个阈值就一定可以得到一幅效果还不错的二值化图像。
//  @parameter      image  原始的灰度图像数组
//  @parameter      clo    图像的宽（图像的列）
//  @parameter      row    图像的高（图像的行）
//  @return         uint8
//  @time           2022年1月17日
//  @Author         陈海涛
//  Sample usage:   Threshold = Threshold_deal(Image_Use[0], 80, 60); 把存放60行80列的二维图像数组Image_Use传进来，求出这幅图像的阈值，并将这个阈值赋给Threshold。
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8 Get_Threshold(uint8* image,uint16 col, uint16 row)
{
  #define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8 threshold = 0;
  uint8 threshold_j=0;
  uint8* data = image;                               //定义一个指向传进来这个image数组的指针变量data
  for (i = 0; i < GrayScale; i++)                    //先把pixelCount和pixelPro两个数组元素全部赋值为0
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/
  for (i = 0; i < height; i += 1)                   //遍历图像的每一行，从第零行到第39行。
  {
    for (j = 0; j < width; j += 1)                  //遍历图像的每一列，从第零列到第93列。
    {
      pixelCount[(int)data[i * width + j]]++;       //将当前的像素点的像素值（灰度值）作为计数数组的下标。
      gray_sum += (int)data[i * width + j];         //计算整幅灰度图像的灰度值总和。
    }
  }
  /**************************************统计每个灰度值(0-255)在整幅图像中出现的次数**************************************/



  /**************************************计算每个像素值（灰度值）在整幅灰度图像中所占的比例*************************************************/
  for (i = 0; i < GrayScale; i++)
  {
      pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  /**************************************计算每个像素值（灰度值）在整幅灰度图像中所占的比例**************************************************/



  /**************************************开始遍历整幅图像的灰度值（0-255），这一步也是大津法最难理解的一步***************************/
  /*******************为什么说他难理解？因为我也是不理解！！反正好像就是一个数学问题，你可以理解为数学公式。***************************/
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (threshold_j = 0; threshold_j < GrayScale; threshold_j++)
  {
    w0 += pixelPro[threshold_j];                          //求出背景部分每个灰度值的像素点所占的比例之和，即背景部分的比例。
    u0tmp += threshold_j * pixelPro[threshold_j];

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;                            //背景平均灰度
    u1 = u1tmp / w1;                            //前景平均灰度
    u = u0tmp + u1tmp;                          //全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = threshold_j;
    }
    if (deltaTmp < deltaMax)
    {
      break;
    }
  }
  return threshold;                             //把上面这么多行代码算出来的阈值给return出去。
}



//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Get_BinaryImage
//  @brief          灰度图像二值化处理
//  @brief          整体思路就是：先调用Get_Threshold（）函数得到阈值，然后遍历原始灰度图像的每一个像素点，用每一个像素点的灰度值来跟阈值计较。
//  @brief          大于阈值的你就把它那个像素点的值赋值为1（记为白点），否则就赋值为0（记为黑点）。当然你可以把这个赋值反过来，只要你自己清楚1和0谁代表黑谁代表白就行。
//  @brief          所以我前面提到的60*80现在你们就应该明白是什么意思了吧！就是像素点嘛，一行有80个像素点，一共60行，也就是压缩后的每一幅图像有4800个像素点。
//  @parameter      void
//  @return         void
//  @time           2022年3月21日
//  @Author
//  Sample usage:   Get_BinaryImage();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
extern Dispose_Image DI;       //图像处理结构体声明
void Get_BinaryImage(void)
{
    Threshold = Get_Threshold(Image_Use[0], LCDW, LCDH);      //这里是一个函数调用，通过该函数可以计算出一个效果很不错的二值化阈值。
  uint8 i, j = 0;
  for (i = 0; i < LCDH; i++)                                //遍历二维数组的每一行
  {
    for (j = 0; j < LCDW; j++)                              //遍历二维数组的每一列
    {
      if (Image_Use[i][j] > Threshold)                      //如果这个点的灰度值大于阈值Threshold
      {
//          Pixle[i][j] = 1;                                  //那么这个像素点就记为白点
          DI.ui8_ImageArray[i][j] = 1;
          Pixle_hb[i][j] = 255;
      }
      else //如果这个点的灰度值小于阈值Threshold
      {
//          Pixle[i][j] = 0;                                  //那么这个像素点就记为黑点
          DI.ui8_ImageArray[i][j] = 0;
          Pixle_hb[i][j] = 0;
      }
    }
  }
}












//像素滤波
void Pixle_Filter()
{
  int nr;  //行
  int nc;  //列
  for (nr = 5; nr < 55; nr++)
  {
    for (nc = 14; nc < 79; nc = nc + 1)
    {
      if ((Pixle[nr][nc] == 0) && (Pixle[nr - 1][nc] + Pixle[nr + 1][nc] + Pixle[nr][nc + 1] + Pixle[nr][nc - 1] >= 3))
      {
          Pixle[nr][nc] = 1;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Image_Process
//  @brief          整个图像处理的主函数，里面包含了所有的图像处理子函数
//  @parameter      void
//  @time           2022年1月19日
//  @Author
//  Sample usage:   Image_Process();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Image_Process(void)
{

    if (mt9v03x_finish_flag == 1)                         //如果一帧图像采集完了，那么就可以对这副图像进行处理了。
    {
        //清零
        /*for (Ysite = 39; Ysite >= 0; Ysite--)
        {
          ImageDeal[Ysite].IsLeftFind = 'F';
          ImageDeal[Ysite].IsRightFind = 'F';
          ImageDeal[Ysite].LeftBorder = 0;
          ImageDeal[Ysite].RightBorder = 93;
          ImageDeal[Ysite].LeftTemp = 0;
          ImageDeal[Ysite].RightTemp = 93;
          ImageDeal[Ysite].Fork_Left = 39;
          ImageDeal[Ysite].Fork_Right = 39;
          ImageDeal[Ysite].Fork_Wid = 0;
        }                     //边界与标志位初始化*/
        Image_Compress();                                 //图像压缩，把原始的80*170的图像压缩成60*80的,因为不需要那么多的信息，60*80能处理好的话已经足够了。
        Get_BinaryImage();                                //图像二值化处理，把采集到的原始灰度图像变成二值化图像，也就是变成黑白图像。
//        if(Enter_Rings_Process == 0)
//        {
//            Garage_Identification();                          //起跑线识别
//        }
        Pixle_Filter();
//        Get_BaseLine();                                   //优化之后的搜线算法：得到一副图像的基础边线，也就是最底下五行的边线信息，用来后续处理。
//        Get_AllLine();                                    //优化之后的搜线算法：得到一副图像的全部边线。
//        Control_OFFLine();
//        Get_ExtensionLine();                             //在得到了全部边线的基础上进行补线。
//        if(Enter_Crosses_Process==0)
//        {
//            Mid_Line_Repair();                               //弯道中线拟合
//        }
//        get_error();                                     //获取误差值
//        Speed_decision();
//
//        Track_test();
//        Test_Half_width();
//        show_ips_flag=1;
        mt9v03x_finish_flag = 0;

    }
}




