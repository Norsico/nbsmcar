/*
 * C_H.c
 *
 *  Created on: 2024年3月1日
 *      Author: Kurumi
 */

#include <image.h>
#include "zf_common_headfile.h"
#include "stdlib.h"
#include <math.h>
#include "motor.h"
#include "Fuzzy.h"
//#include "steer.h"

extern int16 Speed_Goal;
extern int16 Speed_Goal_;
extern uint8 menu_flag_; //二级菜单页面标志位
uint32 exposure_time;
uint16 Speed_Goal_add;
uint8  Element_on;
uint8  banmaxian_on;
extern int speed1,speed2;
extern int bmq_record;

int16 bianmaqi_rings;  //编码器圆环累加

uint8 keep_rings;//圆环保持标志位
uint16 keep_rings_help;//圆环辅助标志位

int32 Black;   //黑子数量

extern float icm_x;
extern float icm_y;
extern float icm_z;
extern float icm_x_last;
extern float icm_y_last;
extern float icm_z_last;
extern float icm_x_out;
extern float icm_y_out;
extern float icm_z_out;

int16 bianmaqi_ring;   //编码器累加
int16 gypo_right;      //右陀螺仪
int16 gypo_left;       //左陀螺仪
uint8 rings_choose;    //圆环方式选择
uint8 motor_choose;   //电机方案选择

int Zebras;   //斑马线积分
int Zebras_;    //堵转积分

int speed_;        //直道加速累加标志位

int ImageScanInterval;                         //扫边范围    上一行的边界+-ImageScanInterval
int ImageScanInterval_Cross;                   //270°的弯道后十字的扫线范围
IFX_ALIGN(4) uint8 Image_Use[LCDH][LCDW];          //灰度图像
IFX_ALIGN(4) uint8 Pixle[LCDH][LCDW];              //用于处理的二值化图像
//uint8 uPixle[uLCDH][uLCDW];                      //用于显示解压后的二值化图像
//uint8 UImage_Use[uLCDH][uLCDW];                  //用于显示解压后的灰度图像
static int Ysite = 0, Xsite = 0;                   //Y坐标=列
static uint8* PicTemp;                             //保存单行图像
static int IntervalLow = 0, IntervalHigh = 0;      //定义高低扫描区间
static int ytemp = 0;                              //存放行
static int TFSite = 0, FTSite = 0;                 //存放行
static float DetR = 0, DetL = 0;                   //存放斜率
static int BottomBorderRight = 79,                 //59行右边界
BottomBorderLeft = 0,                              //59行左边界
BottomCenter = 0;                                  //59行中点
ImageDealDatatypedef ImageDeal[60];                //记录单行的信息
ImageStatustypedef ImageStatus;                    //图像的全局变量
ImageStatustypedef ImageData;             ///////需要修改的图像阈值参数
SystemDatatypdef SystemData;
ImageFlagtypedef ImageFlag;
uint8 Ring_Help_Flag = 0;                      //进环辅助标志
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //左圆环判断的两点纵坐标
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //右圆环判断的两点纵坐标
int Point_Xsite,Point_Ysite;                   //拐点横纵坐标
int Repair_Point_Xsite,Repair_Point_Ysite;     //补线点横纵坐标
int forklenth;                                 //确定三叉位置
int barnlenth;                                 //确定车库位置
int ramplenth;                                 //确定坡道位置
float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77,0.71, 0.65, 0.59, 0.53, 0.47};//10行权重参数，随意更改，基本不影响，大致按照正态分布即可
uint8 ExtenLFlag = 0;  //是否左延长标志
uint8 ExtenRFlag = 0;  //是否右延长标志
float ang_l;
int ycircle=0;
int xcircle=0;
int axcircle=0;
int aycircle=0;
int bxcircle=0;
int bycircle=0;
float K;
uint8 Half_Road_Wide[60] =                      //直道赛道半宽
{  6, 7, 7, 8, 8, 9, 9, 9, 10, 10,
        11, 11,11,11,11,12,12,13,13,14,
       14,14,14,15,15,16,16,16,17,17,
       17,18,18,19,19,20,20,20,21,21,
       21,22,22,23,23,23,24,24,25,25,
       26,26,26,26,27,27,27,28,28,30,
};

uint8 Half_Bend_Wide[60] =                      //弯道赛道半宽
{   22,22,22,22,22,22,22,22,22,22,
    22,22,22,22,22,22,22,22,22,22,
    22,27,22,22,22,22,22,22,22,22,
    22,22,22,22,22,22,22,22,21,21,
    20,20,19,19,19,19,19,19,19,19,
    19,19,19,18,18,18,18,18,17,28,
};

uint8 buxianwide[60]={6,7,8,9,10,11,12,13,14,15,
                      16,17,18,19,20,21,22,23,24,25,
                      26,27,28,29,30,31,32,33,34,35,
                      36,37,38,39,40,41,42,43,44,45,
                      46,47,48,49,50,51,52,53,54,55,
                      56,57,58,59,60,61,62,63,64,65};


int adczongzhi;
int tuoluoyijifen;//避免错误

int Limit(int num, int numH, int numL) {
  if (num > numH)
    num = numH;
  if (num < numL)
    num = numL;
  return num;
}

/*****************直线判断******************/
float Straight_Judge(uint8 dir, uint8 start, uint8 end)     //返回结果小于1即为直线
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir)
    {
    case 1:k = (float)(ImageDeal[start].LeftBorder - ImageDeal[end].LeftBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder) * (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 2:k = (float)(ImageDeal[start].RightBorder - ImageDeal[end].RightBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder) * (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    }
    return S;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      最小二乘法拟合中线
//  @param      start    开始行
//  @param      num      数据个数
//  @return     void
//  @since      v1.0
//  Sample usage:  Fit1(119,20);
//-------------------------------------------------------------------------------------------------------------------
void Fit1(int start,int num)
{
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_x2 = 0.0;
    float sum_xy = 0.0;
    float a,b;
    float y = 0;

    for (int i = 0; i < num; ++i)
    {
        sum_x += y++;
        sum_y += ImageDeal[start-num+i].Center;
        sum_x2 += (y-1) * (y-1);
        sum_xy += (y-1) * ImageDeal[start-num+i].Center;
    }

    sum_x /= num;
    sum_y /= num;
    sum_x2 /= num;
    sum_xy /= num;

    a = (sum_xy - sum_x * sum_y) / (sum_x2 - sum_x * sum_x);
    b = (sum_x2 * sum_y - sum_x * sum_xy) / (sum_x2 - sum_x * sum_x);
    for(int i = 0;i<num;i++)
    {
     ImageDeal[start-num+i].Center = a*i+b;
    }
}




float Get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy)
{

    float BA = 0.00;//向量BA的模
    float BC = 0.00;
    float SBA_BC = 0.00;//向量点乘的值
    float angle = 0.00;

    BA = sqrt((Ax-Bx)*(Ax-Bx)+(Ay-By)*(Ay-By));
    BC = sqrt((Cx-Bx)*(Cx-Bx)+(Cy-By)*(Cy-By));

    SBA_BC = (Ax-Bx)*(Cx-Bx)+(Ay-By)*(Cy-By);

    angle =  acos(SBA_BC*1.00/(BA*BC));

    return angle*57.3;
}


float Fit1_k(int start,int num)
{
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_x2 = 0.0;
    float sum_xy = 0.0;
    float a,b;
    float y = 0;
    uint16 middle_NO[60] = {0};

    for (int i = 0; i < num; ++i)
    {
        sum_x += y++;
        sum_y += ImageDeal[start-num+i].Center;
        sum_x2 += (y-1) * (y-1);
        sum_xy += (y-1) * ImageDeal[start-num+i].Center;
    }

    sum_x /= num;
    sum_y /= num;
    sum_x2 /= num;
    sum_xy /= num;

    a = (sum_xy - sum_x * sum_y) / (sum_x2 - sum_x * sum_x);
    b = (sum_x2 * sum_y - sum_x * sum_xy) / (sum_x2 - sum_x * sum_x);
    for(int i = 0;i<num;i++)
    {
        middle_NO[start-num+i] = a*i+b;
    }

    float Y = 59-ImageStatus.OFFLine+10-1 - 59-ImageStatus.OFFLine+1;                          //求Y轴长度
    float X = middle_NO[59-ImageStatus.OFFLine+10-1] - middle_NO[59-ImageStatus.OFFLine+1];    //求X轴长度(带方向)
    float k = 1.00*(X/Y);                             //求斜率(以y轴为底)
    if(k<0)  k = k*-1;
    return k;
}

float Mh = MT9V03X_H;
float Lh = LCDH;
float Mw = MT9V03X_W;
float Lw = LCDW;

void compressimage() {
  int i, j, row, line;
  const float div_h = Mh / Lh, div_w = Mw / Lw;

  for (i = 0; i < LCDH; i++) {
    row = i * div_h + 0.5;
    for (j = 0; j < LCDW; j++) {
      line = j * div_w + 0.5;
      Image_Use[i][j] =mt9v03x_image[row][line];
    }
  }
  mt9v03x_finish_flag = 0;  //使用完一帧DMA传输的图像图像  可以开始传输下一帧
}


int HD_thre;  //临时观测变量
//二值化
void Get01change() {
  uint8 thre;
  uint8 i, j;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold_static - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold_static - 15;
      else if (j >= 65)
        thre = ImageStatus.Threshold_static - 15;
      else
        thre = ImageStatus.Threshold_static;

      if (Image_Use[i][j] >
          (thre))  //数值越大，显示的内容越多，较浅的图像也能显示出来
        Pixle[i][j] = 1;  //白
      else
        Pixle[i][j] = 0;  //黑
    }
  }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      优化的大津法
//  @param      image  图像数组
//  @param      clo    宽
//  @param      row    高
//  @param      pixel_threshold 阈值分离
//  @return     uint8
//  @since      2021.6.23
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 Threshold_deal(uint8* image,
                     uint16 col,
                     uint16 row,
                     uint32 pixel_threshold) {
#define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8 threshold = 0;
  uint8* data = image;  //指向像素数据的指针
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  //统计灰度级中每个像素在整幅图像中的个数
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
      //{
      pixelCount[(
          int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
      gray_sum += (int)data[i * width + j];  //灰度值总和
      //}
    }
  }

  //计算每个像素值的点在整幅图像中的比例
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }


  //遍历灰度级[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < pixel_threshold; j++) {
    w0 +=
        pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
    u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    //背景平均灰度
    u1 = u1tmp / w1;    //前景平均灰度
    u = u0tmp + u1tmp;  //全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax) break;
  }
  return threshold;
}

void Get01change_dajin() {
  ImageStatus.Threshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
  if (ImageStatus.Threshold < ImageStatus.Threshold_static)
    ImageStatus.Threshold = ImageStatus.Threshold_static;
  uint8 i, j = 0;
  uint8 thre;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold - 10;
      else if (j >= 65)
        thre = ImageStatus.Threshold - 10;
      else
        thre = ImageStatus.Threshold;

      if (Image_Use[i][j] >(thre))         //数值越大，显示的内容越多，较浅的图像也能显示出来
        Pixle[i][j] = 1;  //白
      else
        Pixle[i][j] = 0;  //黑
    }
  }
}
uint8 Num=0;
int left_Num,right_Num;
uint8 Cirque_or_Cross(uint8 type, uint8 startline)         //第一个左右环，第二个左右边界
{//1为左圆环，2为右圆环
    uint8 num =0;
    if(type ==1)
    {
        for(uint8 y=startline; y<startline+10; y++)
        {
            for(uint8 x=ImageDeal[y].LeftBorder; x>1; x--)
            {
                if(Pixle[y][x] !=0)
                {
                    num ++;
                }
            }
        }
    }
    if(type ==2)
    {
        for(uint8 y=startline; y<startline+10; y++)
        {
            for(uint8 x=ImageDeal[y].RightBorder; x<78; x++)
            {
                if(Pixle[y][x] !=0)
                {
                    num ++;
                }
            }
        }
    }
    Num=num;
    return Num;
}

//像素滤波
void Pixle_Filter() {
  int nr;  //行
  int nc;  //列

  for (nr = 10; nr < 40; nr++) {
    for (nc = 10; nc < 70; nc = nc + 1) {
      if ((Pixle[nr][nc] == 0) && (Pixle[nr - 1][nc] + Pixle[nr + 1][nc] +
                                       Pixle[nr][nc + 1] + Pixle[nr][nc - 1] >=
                                   3)) {
        Pixle[nr][nc] = 1;
      }
    }
  }
}




void GetJumpPointFromDet(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)  //第一个参数是要查找的数组（80个点）
                                                                               //第二个扫左边线还是扫右边线
{                                                                              //三四是开始和结束点
  int i = 0;
  if (type == 'L')                              //扫描左边线
  {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //由黑变白
      {
        Q->point = i;                           //记录左边线
        Q->type = 'T';                          //正确跳变
        break;
      } else if (i == (L + 1))                  //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0)            //如果中间是白的
        {
          Q->point = (L + H) / 2;               //认为左边线是中点
          Q->type = 'W';                        //非正确跳变且中间为白，认为没有边
          break;
        } else                                  //非正确跳变且中间为黑
        {
          Q->point = H;                         //如果中间是黑的
          Q->type = 'H';                        //左边线直接最大值，认为是大跳变
          break;
        }
      }
    }
  } else if (type == 'R')                       //扫描右边线
  {
    for (i = L; i <= H; i++)                    //从右往左扫
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //找由黑到白的跳变
      {
        Q->point = i;                           //记录
        Q->type = 'T';
        break;
      } else if (i == (H - 1))                  //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0)            //如果中间是白的
        {
          Q->point = (L + H) / 2;               //右边线是中点
          Q->type = 'W';
          break;
        } else                                  //如果中点是黑的
        {
          Q->point = L;                         //左边线直接最大值
          Q->type = 'H';
          break;
        }
      }
    }
  }
}





static uint8 DrawLinesFirst(void) {
  PicTemp = Pixle[59];
  if (*(PicTemp + ImageSensorMid) == 0)                 //如果底边图像中点为黑，异常情况
  {
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    //找左右边线
    {
      if (*(PicTemp + ImageSensorMid - Xsite) != 0)     //一旦找到左或右赛道到中心距离，就break
        break;                                          //并且记录Xsite
      if (*(PicTemp + ImageSensorMid + Xsite) != 0)
        break;
    }

    if (*(PicTemp + ImageSensorMid - Xsite) != 0)       //赛道如果在左边的话
    {
      BottomBorderRight = ImageSensorMid - Xsite + 1;   // 59行右边线有啦
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)  //开始找59行左边线
      {
        if (*(PicTemp + Xsite) == 0 &&
            *(PicTemp + Xsite - 1) == 0)                //连续两个黑点，滤波
        {
          BottomBorderLeft = Xsite;                     //左边线找到
          break;
        } else if (Xsite == 1) {
          BottomBorderLeft = 0;                         //搜索到最后了，看不到左边线，左边线认为是0
          break;
        }
      }
    } else if (*(PicTemp + ImageSensorMid + Xsite) != 0)  //赛道如果在右边的话
    {
      BottomBorderLeft = ImageSensorMid + Xsite - 1;    // 59行左边线有啦
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)  //开始找59行左边线
      {
        if (  *(PicTemp + Xsite) == 0
            &&*(PicTemp + Xsite + 1) == 0)              //连续两个黑点，滤波
        {
          BottomBorderRight = Xsite;                    //右边线找到
          break;
        } else if (Xsite == 78) {
          BottomBorderRight = 79;                       //搜索到最后了，看不到右边线，左边线认为是79
          break;
        }
      }
    }
  }
  else                                                //左边线中点是白的，比较正常的情况
  {
    for (Xsite = 79; Xsite >ImageSensorMid; Xsite--)   //一个点一个点地搜索右边线
    {
      if (  *(PicTemp + Xsite) == 1
          &&*(PicTemp + Xsite - 1) == 1)                //连续两个黑点，滤波     //两个白点
      {
        BottomBorderRight = Xsite;                      //找到就记录
        break;
      } else if (Xsite == 40) {
        BottomBorderRight = 39;                         //找不到认为79
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    //一个点一个点地搜索左边线
    {
      if (  *(PicTemp + Xsite) == 1
          &&*(PicTemp + Xsite + 1) == 1)                //连续两个黑点，滤波
      {
        BottomBorderLeft = Xsite;                       //找到就记录
        break;
      } else if (Xsite == 38) {
        BottomBorderLeft = 39;                           //找不到认为0
        break;
      }
    }
  }
  BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;   // 59行中点直接取平均
  ImageDeal[59].LeftBorder = BottomBorderLeft;                //在数组里面记录一下信息，第一行特殊一点而已
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;                        //确定最底边
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;  //存储宽度信息
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  for (Ysite = 58; Ysite > 54; Ysite--)                       //由中间向两边确定底边五行
  {
    PicTemp = Pixle[Ysite];
    for (Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center;Xsite--)                                             //和前面一样的搜索
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1) {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center+1)) {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center;Xsite++)                                             //和前面一样的搜索
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1) {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center-1)) {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T';                        //这些信息存储到数组里
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /2; //存储中点
    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;      //存储宽度
  }
  return 'T';
}                                                             //最基本的要求，最近的五行首先不会受到干扰，这需要在安装的时候调整摄像头的视角

/*边线追逐大致得到全部边线*/
static void DrawLinesProcess(void)  //////不用更改
{
  uint8 L_Found_T = 'F';  //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_L_line = 'F';  //找到这一帧图像的基准左斜率
  uint8 R_Found_T = 'F';  //确定无边斜率的基准有边行是否被找到的标志
  uint8 Get_R_line = 'F';  //找到这一帧图像的基准右斜率
  float D_L = 0;           //延长线左边线斜率
  float D_R = 0;           //延长线右边线斜率
  int ytemp_W_L;           //记住首次左丢边行
  int ytemp_W_R;           //记住首次右丢边行
  ExtenRFlag = 0;          //标志位清0
  ExtenLFlag = 0;
   ImageStatus.Left_Line = 0;
   ImageStatus.WhiteLine = 0;
   ImageStatus.Right_Line = 0;
  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)            //前5行处理过了，下面从55行到（设定的不处理的行OFFLine）
  {                        //太远的图像不稳定，OFFLine以后的不处理
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                          // 0左1右
    if (ImageStatus.Road_type != Cross_ture
           /* &&SystemData.SpeedData.Length*OX>500*/) {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;             //从上一行右边线-Interval的点开始（确定扫描开始点）
      IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           //到上一行右边线+Interval的点结束（确定扫描结束点）
    } else {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval_Cross;       //从上一行右边线-Interval_Cross的点开始（确定扫描开始点）
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;    //到上一行右边线+Interval_Cross的点开始（确定扫描开始点）
    }

    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     //扫右边线

    IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                //从上一行左边线-5的点开始（确定扫描开始点）
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               //到上一行左边线+5的点结束（确定扫描结束点）

    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);

    if (JumpPoint[0].type =='W')                                                    //如果本行左边线不正常跳变，即这10个点都是白的
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                 //本行左边线用上一行的数值
    } else                                                                          //左边线正常
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                             //记录下来啦
    }

    if (JumpPoint[1].type == 'W')                                                   //如果本行右边线不正常跳变
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;               //本行右边线用上一行的数值
    } else                                                                          //右边线正常
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                            //记录下来啦
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                 //记录本行是否找到边线，即边线类型
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    //重新确定那些大跳变的边缘
    if (( ImageDeal[Ysite].IsLeftFind == 'H'
         ||ImageDeal[Ysite].IsRightFind == 'H')) {
      if (ImageDeal[Ysite].IsLeftFind == 'H')                                   //如果左边线大跳变
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
             Xsite <= (ImageDeal[Ysite].RightBorder - 1);
             Xsite++)                                                           //左右边线之间重新扫描
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder =Xsite;                                 //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)                                   //一旦出现白点则直接跳出
            break;
          else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))
          {
             ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <=
          7)                              //图像宽度限定
      {
        ImageStatus.OFFLine = Ysite + 1;  //如果这行比7小了后面直接不要了
        break;
      }
      if (ImageDeal[Ysite].IsRightFind == 'H')
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
             Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--) {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0)) {
            ImageDeal[Ysite].RightBorder =
                Xsite;                    //如果右边线的左边还有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
    }

 /***********重新确定无边行************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;


    if(  ImageStatus.Road_type != Ramp)
    {
    if (    ImageDeal[Ysite].IsRightFind == 'W'
          &&Ysite > 10
          &&Ysite < 50
          &&ImageStatus.Road_type!=Barn_in
          )                     //最早出现的无边行
    {
      if (Get_R_line == 'F')    //这一帧图像没有跑过这个找基准线的代码段才运行
      {
        Get_R_line = 'T';       //找了  一帧图像只跑一次 置为T
        ytemp_W_R = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsRightFind =='T')  //往无边行下面搜索  一般都是有边的
            R_found_point++;
        }
        if (R_found_point >8)                      //找到基准斜率边  做延长线重新确定无边   当有边的点数大于8
        {
          D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));
                                                  //求下面这些点连起来的斜率
                                                  //好给无边行做延长线左个基准
          if (D_R > 0) {
            R_Found_T ='T';                       //如果斜率大于0  那么找到了这个基准行  因为梯形畸变
                                                  //所以一般情况都是斜率大于0  小于0的情况也不用延长 没必要
          } else {
            R_Found_T = 'F';                      //没有找到这个基准行
            if (D_R < 0)
              ExtenRFlag = 'F';                   //这个标志位用于十字角点补线  防止图像误补用的
          }
        }
      }
      if (R_Found_T == 'T')
        ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //如果找到了 那么以基准行做延长线

      LimitL(ImageDeal[Ysite].RightBorder);  //限幅
      LimitH(ImageDeal[Ysite].RightBorder);  //限幅
    }

    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 &&
        ImageStatus.Road_type != Barn_in)    //下面同理  左边界
    {
      if (Get_L_line == 'F') {
        Get_L_line = 'T';
        ytemp_W_L = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsLeftFind == 'T')
            L_found_point++;
        }
        if (L_found_point > 8)              //找到基准斜率边  做延长线重新确定无边
        {
          D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -ImageDeal[Ysite + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
          if (D_L > 0) {
            L_Found_T = 'T';

          } else {
            L_Found_T = 'F';
            if (D_L < 0)
              ExtenLFlag = 'F';
          }
        }
      }

      if (L_Found_T == 'T')
        ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);

      LimitL(ImageDeal[Ysite].LeftBorder);  //限幅
      LimitH(ImageDeal[Ysite].LeftBorder);  //限幅
    }
}
    if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
         {
             ImageStatus.WhiteLine++;  //要是左右都无边，丢边数+1
         }
        if (ImageDeal[Ysite].IsLeftFind == 'W'&&Ysite<55)
        {
             ImageStatus.Left_Line++;
        }
        if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite<55)
        {
             ImageStatus.Right_Line++;
        }


      LimitL(ImageDeal[Ysite].LeftBorder);   //限幅
      LimitH(ImageDeal[Ysite].LeftBorder);   //限幅
      LimitL(ImageDeal[Ysite].RightBorder);  //限幅
      LimitH(ImageDeal[Ysite].RightBorder);  //限幅

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 7)         //重新确定可视距离
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (  ImageDeal[Ysite].RightBorder <= 10
             ||ImageDeal[Ysite].LeftBorder >= 70) {
              ImageStatus.OFFLine = Ysite + 1;
              break;
    }                                        //当图像宽度小于0或者左右边达到一定的限制时，则终止巡边
  }


  return;
}


//延长线绘制，理论上来说是很准确的
static void DrawExtensionLine(void)        //绘制延长线并重新确定中线 ，把补线补成斜线
{
  if (
        (Fork_dowm == 0
        &&ImageStatus.CirquePass == 'F'
        &&ImageStatus.IsCinqueOutIn == 'F'
        &&ImageStatus.CirqueOut == 'F'
//        &&ImageStatus.Road_type != Forkin
//        &&ImageStatus.Road_type != Forkout
        &&ImageStatus.Road_type != Barn_in
        &&ImageStatus.Road_type != Ramp)
//        &&ImageStatus.pansancha_Lenth* OX==0
        &&ImageStatus.Road_type !=Cross_ture
        ||ImageStatus.CirqueOff == 'T'
        )                                  // g5.22  6.22调试注释  记得改回来
  {
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == LeftCirque)
      TFSite = 55;
    if (ExtenLFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)                       //从第五行开始网上扫扫到顶边下面两行   多段补线
                                          //不仅仅只有一段
      {
        PicTemp = Pixle[Ysite];           //存当前行
        if (ImageDeal[Ysite].IsLeftFind =='W')                          //如果本行左边界没扫到但扫到的是白色，说明本行没有左边界点
        {
          //**************************************************//**************************************************
          if (ImageDeal[Ysite + 1].LeftBorder >= 70)                    //如果左边界实在是太右边
          {
            ImageStatus.OFFLine = Ysite + 1;
            break;                        //直接跳出（极端情况）
          }
          //************************************************//*************************************************

          while (Ysite >= (ImageStatus.OFFLine + 4))                    //此时还没扫到顶边
          {
            Ysite--;                      //继续往上扫
            if (  ImageDeal[Ysite].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 1].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].LeftBorder > 0
                &&ImageDeal[Ysite - 2].LeftBorder <70
                )                                                       //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
            {
              FTSite = Ysite - 2;          //把本行上面的第二行存入FTsite
              break;
            }
          }

          DetL =
              ((float)(ImageDeal[FTSite].LeftBorder -
                       ImageDeal[TFSite].LeftBorder)) /
              ((float)(FTSite - TFSite));  //左边界的斜率：列的坐标差/行的坐标差
          if (FTSite > ImageStatus.OFFLine)
            for (
                ytemp = TFSite; ytemp >= FTSite; ytemp--)               //从第一次扫到的左边界的下面第二行的坐标开始往上扫直到空白上方的左边界的行坐标值
            {
              ImageDeal[ytemp].LeftBorder =
                  (int)(DetL * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite]
                      .LeftBorder;                                      //将这期间的空白处补线（补斜线），目的是方便图像处理
            }
        } else
          TFSite = Ysite + 2;                                           //如果扫到了本行的左边界，该行存在这里面，（算斜率）
      }

    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    // g5.22
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55;
    if (ExtenRFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)               //从第五行开始网上扫扫到顶边下面两行
      {
        PicTemp = Pixle[Ysite];  //存当前行

        if (ImageDeal[Ysite].IsRightFind =='W')                       //如果本行右边界没扫到但扫到的是白色，说明本行没有右边界点，但是处于赛道内的
        {
          if (ImageDeal[Ysite + 1].RightBorder <= 10)                 //如果右边界实在是太左边
          {
            ImageStatus.OFFLine =Ysite + 1;                           //直接跳出，说明这种情况赛道就尼玛离谱
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4))                  //此时还没扫到顶边下面两行
          {
            Ysite--;
            if (  ImageDeal[Ysite].IsRightFind == 'T'
                &&ImageDeal[Ysite - 1].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].RightBorder < 70
                &&ImageDeal[Ysite - 2].RightBorder > 10
                )                                                      //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
            {
              FTSite = Ysite - 2;                                      // 把本行上面的第二行存入FTsite
              break;
            }
          }

          DetR =((float)(ImageDeal[FTSite].RightBorder -ImageDeal[TFSite].RightBorder)) /((float)(FTSite - TFSite));         //右边界的斜率：列的坐标差/行的坐标差
          if (FTSite > ImageStatus.OFFLine)
            for (ytemp = TFSite; ytemp >= FTSite;ytemp--)              //从第一次扫到的右边界的下面第二行的坐标开始往上扫直到空白上方的右边界的行坐标值
            {
              ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].RightBorder;          //将这期间的空白处补线（补斜线），目的是方便图像处理
            }
        } else
          TFSite =Ysite +2;                                           //如果本行的右边界找到了，则把该行下面第二行坐标送个TFsite
      }
  }
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /2;                                //扫描结束，把这一块经优化之后的中间值存入
    ImageDeal[Ysite].Wide =-ImageDeal[Ysite].LeftBorder +ImageDeal[Ysite].RightBorder;                                       //把优化之后的宽度存入
  }
}
/*上交大左右手法则扫线，作为处理圆环等判断元素的第二依据*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Bottom_Line_OTSU
//  @brief          获取底层左右边线
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @return         Bottonline                              底边行选择
//  @time           2022年10月9日
//  @Author
//  Sample usage:   Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{

    //寻找左边边界
    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;//获取底边左边线
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < LCDW-1; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;//获取底边右边线
            break;
        }
    }

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Left_and_Right_Lines
//  @brief          通过sobel提取左右边线
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         无
//  @time           2022年10月7日
//  @Author
//  Sample usage:   Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    //定义小人的当前行走状态位置为 上 左 下 右 一次要求 上：左边为黑色 左：上边为褐色 下：右边为色  右：下面有黑色
/*  前进方向定义：
                *   0
                * 3   1
                *   2
*/
/*寻左线坐标规则*/
    uint8 Left_Rule[2][8] = {
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
    };
    /*寻右线坐标规则*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
    };
      int num=0;
    uint8 Left_Ysite = Bottonline;
    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    uint8 Left_Rirection = 0;//左边方向
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;

    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;//右边方向
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;
    ImageStatus.OFFLineBoundary = 5;
    while (1)
    {
            num++;
            if(num>400)
            {
                 ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {
            if (Ysite < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;
            }
        }
        /*********左边巡线*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//前方是黑色
            {
                //顺时针旋转90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//前方是白色
            {
                /*计算左前方坐标*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0)
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                        ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Left_Rirection  逆时针90度
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }

            }
        }
        /*********右边巡线*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//前方是黑色
            {
                //逆时针旋转90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//前方是白色
            {
                /*计算右前方坐标*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79 )
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Right_Rirection  逆时针90度
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }

            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80是为了放在底部是斑马线扫描结束  3 && Ysite < 30
        {

            ImageStatus.OFFLineBoundary = Ysite;
            break;
        }

    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Border_OTSU
//  @brief          通过OTSU获取边线 和信息
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        传入的图像数组
//  @param          Row                                     图像的Ysite
//  @param          Col                                     图像的Xsite
//  @param          Bottonline                              底边行选择
//  @return         无
//  @time           2022年10月7日
//  @Author
//  Sample usage:   Search_Border_OTSU(mt9v03x_image, IMAGE_ROW, IMAGE_COL, IMAGE_ROW-8);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /*封上下边界处理*/
    for (int Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*封左右边界处理*/
    for (int Ysite = 0; Ysite < LCDH; Ysite++)
    {
            ImageDeal[Ysite].LeftBoundary_First = 0;
            ImageDeal[Ysite].RightBoundary_First = 79;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][LCDW - 1] = 0;
    }
    /********获取底部边线*********/
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    /********获取左右边线*********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);



    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if (ImageDeal[Ysite].RightBoundary > LCDW - 3)
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}

//出现丢边的时候  重新确定无边行的中线
static void RouteFilter(void) {
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5);
       Ysite--)                                     //从开始位到停止位
  {
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W'
         &&Ysite <= 45
         &&ImageDeal[Ysite - 1].IsLeftFind == 'W'
         &&ImageDeal[Ysite - 1].IsRightFind =='W')  //当前行左右都无边，而且在前45行   滤波
    {
      ytemp = Ysite;
      while (ytemp >= (ImageStatus.OFFLine +5))     // 改改试试，-6效果好一些
      {
        ytemp--;
        if (  ImageDeal[ytemp].IsLeftFind == 'T'
            &&ImageDeal[ytemp].IsRightFind == 'T')  //寻找两边都正常的，找到离本行最近的就不找了
        {
          DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /(float)(ytemp - 1 - Ysite - 2);          //算斜率
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) {
            ImageDeal[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp));                                     //用斜率补
            Ysite--;
          }
          break;
        }
      }
    }
    ImageDeal[Ysite].Center =(ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) /3;                                  //求平均，应该会比较滑  本来是上下两点平均
  }
}

int icm_start_test_cross = 0;  //开启icm积分标志位

/****十字检测*****/  //上面的算法已经滤除了十字  相当于不要处理
void Cross_Test() {
  if (ImageStatus.OFFLine > 15)                       //当可视距离比较近的时候 开启积分处理  用于特殊处理270°弯道之后的十字
    icm_start_test_cross = 1;
  else
    icm_start_test_cross = 0;

  if (abs(tuoluoyijifen) > 100)
    ImageStatus.Road_type = Cross;
  if (ImageStatus.Road_type == Cross && ImageStatus.Cross_Lenth * OX > 100)
    ImageStatus.Road_type = 0;
}

//获取单行黑点数量  最后没有用到
int Black_Point = 0;
int Black_Point_ALL = 0;
void GetBlack_Point()
{
  for (Ysite = 59; Ysite > 0; Ysite--) {
    PicTemp = Pixle[Ysite];
    for (Xsite = 0; Xsite < 80; Xsite++) {
      if (*(PicTemp + Xsite) == 0)
        Black_Point++;
    }
    Black_Point_ALL = Black_Point_ALL + Black_Point;
    ImageDeal[Ysite].Black_Point = Black_Point;
    Black_Point = 0;
  }
  ImageStatus.Black_Pro_ALL = Black_Point_ALL / 48;
  Black_Point_ALL = 0;
}

/****圆环检测***/
uint8 Pass_flag = 'F';
//uint8 Left_Less_Num = 0;
//环岛检测
//--------------------------------------------------------------
//  @name           Element_Judgment_Left_Rings()
//  @brief          整个图像判断的子函数，用来判断左圆环类型.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Left_Rings();
//--------------------------------------------------------------
void Element_Judgment_Left_Rings()
{

    int ring_ysite = 3;
  //  uint8 Left_Less_Num = 0;
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
 //   ceshi_flag = 1;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;

        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {

        if (
                ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder
            )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0 )
    {
        if(ImageStatus.Left_Line > 6)//13
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag ==0)
    {

        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;
        ImageStatus.Road_type = LeftCirque;
        //gpio_set_level(P20_8, 0);
        //wireless_uart_send_byte(9);
    }
    Ring_Help_Flag = 0;
}

//--------------------------------------------------------------
//  @name           Element_Judgment_Right_Rings()
//  @brief          整个图像判断的子函数，用来判断右圆环类型.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Right_Rings();
//--------------------------------------------------------------
void Element_Judgment_Right_Rings()
{
//    if (   ImageStatus.Left_Line > 0
//        || ImageStatus.Right_Line < 20 //13
//        || ImageStatus.OFFLine > 5
//        )
//        {return;}
    int ring_ysite = 3;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {
//        if (ImageDeal[Ysite + 3].RightBoundary_First > ImageDeal[Ysite].RightBoundary_First
//            && ImageDeal[Ysite + 2].RightBoundary_First > ImageDeal[Ysite].RightBoundary_First
//            && ImageDeal[Ysite].RightBoundary_First < ImageDeal[Ysite - 1].RightBoundary_First
//            && ImageDeal[Ysite].RightBoundary_First < ImageDeal[Ysite - 2].RightBoundary_First
//           )
        if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder
           )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.Right_Line>7)
            Ring_Help_Flag = 1;
    }
    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0)
    {

        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;     //小环
        ImageStatus.Road_type = RightCirque;
//        gpio_set_level(Beep, 1);
    }
    Ring_Help_Flag = 0;
}

//--------------------------------------------------------------
//  @name           Element_Handle_Left_Rings()
//  @brief          整个图像处理的子函数，用来处理左圆环类型.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Left_Rings();
//-------------------------------------------------------------

void Element_Handle_Left_Rings_1()
{
    /****************大小圆环判断*****************/
    if(ImageFlag.ring_big_small == 0)
    {
        Black = 0;
        for (int Ysite = 30; Ysite > 0; Ysite--)
        {
            if(Pixle[Ysite][5]==0)
            {
                Black++;
            }
        }
        if(Black > 10)
            ImageFlag.ring_big_small = 1;//大圆环
        else
            ImageFlag.ring_big_small = 2;//小圆环
    }
    /***************************************判断**************************************/
    int num = 0;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].LeftBoundary_First == 1)
        {
            num++;
        }
        else
            break;
    }
//    ips114_show_int(180,20,num,5);
        //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>20)
    {
        ImageFlag.image_element_rings_flag = 2;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);

    }
    if (ImageFlag.image_element_rings_flag == 2 && num<7)
    {

        ImageFlag.image_element_rings_flag = 5;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }
//    if(ImageFlag.image_element_rings_flag == 3 && num<3)//
//    {
//        ImageFlag.image_element_rings_flag = 4;
//    }
        //刚到圆弧
    if(ImageFlag.image_element_rings_flag == 4)
    {
        //Stop= 1;
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 40; Ysite > ImageStatus.OFFLine + 2; Ysite--)
        {
            if(ImageDeal[Ysite].IsLeftFind == 'W' && ImageDeal[Ysite-1].IsLeftFind == 'T')
            {
                Point_Ysite = Ysite;
                Point_Xsite = 0;
                break;
            }
        }
        if(Point_Ysite > 6 && ImageFlag.ring_big_small == 1)
        {
            ImageFlag.image_element_rings_flag=5;
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        }
        if(Point_Ysite > 11 && ImageFlag.ring_big_small == 2)
        {
            ImageFlag.image_element_rings_flag=5;
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        }
    }
        //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.WhiteLine_R>30)
    {
        ImageFlag.image_element_rings_flag = 6;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.WhiteLine_R<8)
    {
        //Stop = 1;
        ImageFlag.image_element_rings_flag = 7;
        icm_x = 0;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }

    if (ImageFlag.image_element_rings_flag == 7)
    {

        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary + 3; Ysite--)
        {
            if (    ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite + 1].RightBoundary
                 && ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite - 1].RightBoundary
               )
            {
                Point_Xsite = ImageDeal[Ysite].RightBoundary;
                Point_Ysite = Ysite;
                break;
            }
        }
        if
//           (Point_Ysite > 40)
            (icm_x>gypo_left&&ImageStatus.Right_Line > 20)
//            if(icm_x>220)                   //陀螺仪判断出环岛
//            if(ImageStatus.Right_Line < 16)
        {
            ImageFlag.image_element_rings_flag = 8;
//            SystemData.Stop = 1; //停车标志位
        }
    }
        //出环后
    if (ImageFlag.image_element_rings_flag == 8)
    {
        if (
                 ImageStatus.Right_Line < 3
             && ImageStatus.OFFLine < 10
             )    //右边为直线且截止行（前瞻值）很小

            ImageFlag.image_element_rings_flag = 9;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }

    if (ImageFlag.image_element_rings_flag == 9)
    {
        int num=0;
        for (int Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].LeftBoundary_First < 3 )
            {
                num++;
            }
        }
        if(num < 3)
        {
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
         //   gpio_set_level(B0, 0);
        }
    }



    /***************************************处理**************************************/
        //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite]-5;
        }
    }

    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6
        )
    {
        int  flag_Xsite_1=0;
        int flag_Ysite_1=0;
        float Slope_Rings=0;
//        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
//            {
//                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite]-10;
//            }
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)//下面弧点
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(  Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                 {
                   flag_Ysite_1 = Ysite;
                   flag_Xsite_1 = Xsite;
                   Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                   break;
                 }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
            {
                if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite+1].IsLeftFind=='T'&&ImageDeal[Ysite+2].IsLeftFind=='W'
                    &&abs(ImageDeal[Ysite].LeftBorder-ImageDeal[Ysite+2].LeftBorder)>10
                  )
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=ImageDeal[flag_Ysite_1].LeftBorder;
                    ImageStatus.OFFLine=Ysite;
                    Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }

            }
        }
        //补线
        if(flag_Ysite_1 != 0)
        {
            for(Ysite=flag_Ysite_1;Ysite<59;Ysite++)
            {
                ImageDeal[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);//标记点加上斜率乘以坐标
                //if(ImageFlag.ring_big_small==1)//大圆环不减半宽
//                    ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2;
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite]-8;
                //else//小圆环减半宽
                //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                //if(ImageDeal[Ysite].Center<0)
                   // ImageDeal[Ysite].Center = 0;

            }
            ImageDeal[flag_Ysite_1].RightBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
            {
                for(Xsite=ImageDeal[Ysite+1].RightBorder-10;Xsite<ImageDeal[Ysite+1].RightBorder+2;Xsite++)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                    {
                        ImageDeal[Ysite].RightBorder=Xsite;
                        //if(ImageFlag.ring_big_small==1)//大圆环不减半宽
                            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2;






                        //else//小圆环减半宽
                          //  ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                       // if(ImageDeal[Ysite].Center<0)
                            //ImageDeal[Ysite].Center = 0;
                        //ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                        break;
                    }
                }

                if(ImageDeal[Ysite].Wide>8 &&ImageDeal[Ysite].RightBorder< ImageDeal[Ysite+2].RightBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        //环内 小环弯道减半宽 大环不减
    if (ImageFlag.image_element_rings_flag == 7)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite]-10;
                    }
//        for (int Ysite = 57; Ysite > ImageStatus.OFFLine+1; Ysite--)
//        {
//            if(ImageFlag.ring_big_small==2)
//                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
//            if(ImageDeal[Ysite].Center<=0)
//            {
//                ImageDeal[Ysite].Center = 0;
//                ImageStatus.OFFLine=Ysite-1;
//                break;
//            }
//        }
    }
    if (ImageFlag.image_element_rings_flag == 8)    //小圆环
    {
        Repair_Point_Xsite = 0;
        Repair_Point_Ysite = 0;
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                            {
                                ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite]-10;
                            }
        for (int Ysite = 55; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][15] == 1 && Pixle[Ysite-1][15] == 0)
            {
                Repair_Point_Xsite = 15;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
        {
            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
//            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite]-15;
        }
    }
        //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite]-10;
        }
    }
}

void Element_Handle_Right_Rings_1()
{
    /****************大小圆环判断*****************/
    if(ImageFlag.ring_big_small == 0)
    {
       // Less_Big_Small_Num = 0;
        for (int Ysite = 30; Ysite > 0; Ysite--)
        {
            if(Pixle[Ysite][75]==0)
            {
                Black++;
            }
        }
        if(Black > 10) //大圆环
            ImageFlag.ring_big_small = 1;
        else          //小圆环
            ImageFlag.ring_big_small = 2;
    }
    /****************判断*****************/
    int num =0 ;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].RightBoundary_First == 78)
        {
            num++;
            ips114_show_int(165,117,num,1);
        }
        else
            break;
    }
        //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>20)
    {
        ImageFlag.image_element_rings_flag = 2;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<7
//            && ImageStatus.Right_Line>23
            )     //这个地方要修改
    {
        ImageFlag.image_element_rings_flag = 5;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        //Stop = 1;
    }
        //刚到圆弧
//    if (ImageFlag.image_element_rings_flag == 4)
//    {
//        Point_Ysite = 0;
//        Point_Xsite = 0;
//        for (int Ysite = 30; Ysite > ImageStatus.OFFLine + 2; Ysite--)
//        {
//            if(ImageDeal[Ysite].IsRightFind == 'W' && ImageDeal[Ysite-1].IsRightFind == 'T')
//            {
//                Point_Ysite = Ysite;
//                Point_Xsite = 0;
//                break;
//            }
//        }
//        if(Point_Ysite > 6 && ImageFlag.ring_big_small == 1)
//        {
//         //   Stop =1;
//            ImageFlag.image_element_rings_flag=5;
//            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
//        }
//        if(Point_Ysite > 11 && ImageFlag.ring_big_small == 2)
//        {
//         //   Stop =1;
//            ImageFlag.image_element_rings_flag=5;
//            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
//        }
//
//    }
        //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line>30)
    {
      //  Stop = 1;
        ImageFlag.image_element_rings_flag = 6;
//        keep_rings = 1;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line<10)
    {
        ImageFlag.image_element_rings_flag = 7;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
       // Stop=1;


        //代码目前直接复制大圆环 大小圆环判断有问题



//        ImageFlag.ring_big_small = 1 ;
    }
        //环内 大圆环判断
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary+3; Ysite--)
        {
            if (    ImageDeal[Ysite].LeftBoundary > ImageDeal[Ysite + 1].LeftBoundary
                 && ImageDeal[Ysite].LeftBoundary > ImageDeal[Ysite - 1].LeftBoundary
                )

            {
                        Point_Xsite = ImageDeal[Ysite].LeftBoundary;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 40)
        {
            ImageFlag.image_element_rings_flag = 8;
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        }
    }
        //环内 小圆环判断
//    if (ImageFlag.ring_big_small == 2 && ImageFlag.image_element_rings_flag == 7 && ImageStatus.WhiteLine_R > 37)
//    {
//        Point_Xsite = 0;
//        Point_Ysite = 0;
//        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary+3; Ysite--)
//        {
//            if (  ImageDeal[Ysite].LeftBoundary > ImageDeal[Ysite + 2].LeftBoundary
//               && ImageDeal[Ysite].LeftBoundary > ImageDeal[Ysite - 2].LeftBoundary
//              )
//
//            {
//                      Point_Xsite = ImageDeal[Ysite].LeftBoundary;
//                      Point_Ysite = Ysite;
//                      break;
//            }
//        }
//        if (Point_Ysite > 20)
//        {
//            ImageFlag.image_element_rings_flag = 8;
//        }
//    }
        //出环后
    if (ImageFlag.image_element_rings_flag == 8)
    {

         if (
//                 Straight_Judge(1, ImageStatus.OFFLine+10, 50) < 1
//             && ImageStatus.Left_Line < 5
//             && ImageStatus.OFFLine < 7
                  ImageStatus.Left_Line < 9
                              && ImageStatus.OFFLine < 20
             )    //右边为直线且截止行（前瞻值）很小
            {
             ImageFlag.image_element_rings_flag = 9;
//             keep_rings = 0;
             ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
            }

    }

    //结束圆环进程
    if (ImageFlag.image_element_rings_flag == 9)
    {
        int num=0;
        for (int Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].RightBoundary_First > 76 )
            {
                num++;
            }
        }
        if(num < 3)
        {
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;

            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        //    gpio_set_level(B0, 0);
        }
    }
    /***************************************处理**************************************/
         //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        //|| ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }

    }
//    if(
//    //                        (
//                                    ImageStatus.Left_Line > 15 &&25>ImageStatus.Left_Line &&
//                             30>ImageStatus.Right_Line &&ImageStatus.Right_Line > 15
//    //                         )
//    //                         ||
//    //                        (ImageStatus.Left_Line < 3 && ImageStatus.Right_Line < 3)
//
//                            )
//                    {
//                        ImageStatus.Road_type = 0;   //出环处理完道路类型清0
//                        ImageFlag.image_element_rings_flag = 0;
//                        ImageFlag.image_element_rings = 0;
//                        ImageFlag.ring_big_small = 0;
//                        ImageStatus.Road_type = Normol;
//                    }
        //进环  补线
    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6
       )
    {
//        system_start();

//        if(system_getval_ms()>500)
//        {
//            keep_rings = 1;
//        }
        int flag_Xsite_1=0;
        int  flag_Ysite_1=0;
        float Slope_Right_Rings = 0;

        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=Xsite;
                    Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
              break;
            }
        }
        if(flag_Ysite_1==0)
        {
        for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
        {
         if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite+1].IsRightFind=='T'&&ImageDeal[Ysite+2].IsRightFind=='W'
               &&abs(ImageDeal[Ysite].RightBorder-ImageDeal[Ysite+2].RightBorder)>10
         )
         {
             flag_Ysite_1=Ysite;
             flag_Xsite_1=ImageDeal[flag_Ysite_1].RightBorder;
             ImageStatus.OFFLine=Ysite;
             Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
             break;
         }

        }

        }
        //补线
        if(flag_Ysite_1!=0)
        {
            for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
            {
                ImageDeal[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);


                //if(ImageFlag.ring_big_small==2)//小圆环加半宽
                //    ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
//              else//大圆环不加半宽
//                    ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//板块
                ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite]+5;
                //if(ImageDeal[Ysite].Center>79)
                //    ImageDeal[Ysite].Center=79;
            }
            ImageDeal[flag_Ysite_1].LeftBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
            {
                for(Xsite=ImageDeal[Ysite+1].LeftBorder+8;Xsite>ImageDeal[Ysite+1].LeftBorder-4;Xsite--)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite-1]==0)
                    {
                     ImageDeal[Ysite].LeftBorder=Xsite;
                     ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                  //   if(ImageFlag.ring_big_small==2)//小圆环加半宽
                  //       ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
                   //  else//大圆环不加半宽
                         ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//板块
                   //  if(ImageDeal[Ysite].Center>79)
                   //      ImageDeal[Ysite].Center=79;
                     break;
                    }
                }
                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder>  ImageDeal[Ysite+2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }


    }

        //环内不处理
    if (ImageFlag.image_element_rings_flag == 7)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
              {
                  ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite]+5;
              }

    }

        //大圆环出环 补线
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)  //大圆环
    {
        Repair_Point_Xsite = 60;
        Repair_Point_Ysite = 0;
        for (int Ysite = 50; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][55] == 1 && Pixle[Ysite-1][55] == 0)
            {
                Repair_Point_Xsite = 55;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
                        //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
        {
            ImageDeal[Ysite].LeftBorder = (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //小圆环出环 补线
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 2)  //小圆环
    {
        Repair_Point_Xsite = 79;
        Repair_Point_Ysite = 0;
        for (int Ysite = 50; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][65] == 1 && Pixle[Ysite-1][65] == 0)
            {
                Repair_Point_Xsite = 74;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
                        //  ips200_show_uint(200,200,Repair_Point_Ysite,2);
                break;
            }
        }
        for (int Ysite = 55; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
        {
            ImageDeal[Ysite].LeftBorder = (ImageDeal[56].LeftBorder - Repair_Point_Xsite) * (Ysite - 56) / (56 - Repair_Point_Ysite)  + ImageDeal[56].LeftBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}

//左圆环判断
void Element_Handle_Left_Rings()
{
    /***************************************判断**************************************/
     int num = 0;
     for (int Ysite = 55; Ysite > 30; Ysite--)
     {
         if(ImageDeal[Ysite].IsLeftFind == 'W')
             num++;
         if(    ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
             && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T')
             break;
     }
 //    tft180_show_int(60,125,num,3);
 //    int ring_ysite = 30;
 //    for (int Ysite = 5; Ysite < ring_ysite; Ysite++)
 //    {
 //        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
 //        {
 //            Right_RingsFlag_Point1_Ysite = Ysite;
 //            break;
 //        }
 //    }
 //    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
 //    {
 //        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
 //        {
 //            Right_RingsFlag_Point2_Ysite = Ysite;
 //            break;
 //        }
 //    }
         //准备进环
     if (ImageFlag.image_element_rings_flag == 1 && num>10)
     {
         ImageFlag.image_element_rings_flag = 2;
         ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
         //wireless_uart_send_byte(2);
     }
     if (ImageFlag.image_element_rings_flag == 2 && num<8)
     {

         ImageFlag.image_element_rings_flag = 5;
         ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
         //wireless_uart_send_byte(5);
     }
         //进环
     if(ImageFlag.image_element_rings_flag == 5 && /*num>15)*/ImageStatus.Right_Line>15)
     {
         ImageFlag.image_element_rings_flag = 6;
         ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
      //   ImageStatus.Road_type = LeftCirque;
         //wireless_uart_send_byte(6);
     }
         //进环小圆环
     if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line<4)
     {
         //Stop = 1;
         ImageFlag.image_element_rings_flag = 7;
         ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
         //wireless_uart_send_byte(8);
     }
         //环内 大圆环判断
     if (ImageFlag.image_element_rings_flag == 7)
     {
         Point_Ysite = 0;
         Point_Xsite = 0;
         for (int Ysite = 45; Ysite > ImageStatus.OFFLine + 3; Ysite--)
         {
             if (
//             ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder
//                     && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder
//                     &&
                     ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder
                     && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder
//                     && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 4].RightBorder
//                     && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 4].RightBorder
                )
             {
                 Point_Xsite = ImageDeal[Ysite].RightBorder;
                 Point_Ysite = Ysite;
                 break;
             }
         }
         if (Point_Ysite > 22)
         {
             ImageFlag.image_element_rings_flag = 8;
             ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
            // wireless_uart_send_byte(8);
             //Stop = 1;
         }
     }
         if (ImageFlag.image_element_rings_flag == 8)
         {
              if (
                  ImageStatus.Right_Line < 9
                  && ImageStatus.OFFLine < 10)    //右边为直线且截止行（前瞻值）很小
                {ImageFlag.image_element_rings_flag = 9;
                ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
                 //wireless_uart_send_byte(9);
                }
         }

         //结束圆环进程
         if (ImageFlag.image_element_rings_flag == 9)
         {
             int num=0;
             for (int Ysite = 40; Ysite > 10; Ysite--)
             {
                 if(ImageDeal[Ysite].IsLeftFind == 'W' )
                     num++;
             }
             if(num < 5)
             {
                 ImageStatus.Road_type = 0;   //出环处理完道路类型清0
                 ImageFlag.image_element_rings_flag = 0;
                 ImageFlag.image_element_rings = 0;
                 ImageFlag.ring_big_small = 0;
                 ImageStatus.Road_type = Normol;
                 //wireless_uart_send_byte(0);
 //                gpio_set_level(Beep, 0);
             }
     }



     /***************************************处理**************************************/
         //准备进环  半宽处理
     if (   ImageFlag.image_element_rings_flag == 1
         || ImageFlag.image_element_rings_flag == 2
         || ImageFlag.image_element_rings_flag == 3
         || ImageFlag.image_element_rings_flag == 4)
     {
         for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
         {
             ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
         }
     }
         //进环  补线
     if  ( ImageFlag.image_element_rings_flag == 5
         ||ImageFlag.image_element_rings_flag == 6
         )
     {
         int  flag_Xsite_1=0;
         int  flag_Ysite_1=0;
         float Slope_Rings=0;
         for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)//下面弧点
         {
             for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
             {
                 if(  Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                  {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = Xsite;
                    Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                  }
             }
             if(flag_Ysite_1 != 0)
             {
                 break;
             }
         }
         if(flag_Ysite_1 == 0)
         {
             for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
             {
                 if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite+1].IsLeftFind=='T'&&ImageDeal[Ysite+2].IsLeftFind=='W'
                     &&abs(ImageDeal[Ysite].LeftBorder-ImageDeal[Ysite+2].LeftBorder)>10
                   )
                 {
                     flag_Ysite_1=Ysite;
                     flag_Xsite_1=ImageDeal[flag_Ysite_1].LeftBorder;
                     ImageStatus.OFFLine=Ysite;
                     Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                     break;
                 }

             }
         }
         //补线
         if(flag_Ysite_1 != 0)
         {
             for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
             {
                 ImageDeal[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);
                 //if(ImageFlag.ring_big_small==1)//大圆环不减半宽
                     ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                 //else//小圆环减半宽
                 //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                 if(ImageDeal[Ysite].Center<4)
                     ImageDeal[Ysite].Center = 4;
             }
             ImageDeal[flag_Ysite_1].RightBorder=flag_Xsite_1;
             for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
             {
                 for(Xsite=ImageDeal[Ysite+1].RightBorder-10;Xsite<ImageDeal[Ysite+1].RightBorder+2;Xsite++)
                 {
                     if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                     {
                         ImageDeal[Ysite].RightBorder=Xsite;
                         //if(ImageFlag.ring_big_small==1)//大圆环不减半宽
                             ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
//                             ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite]-5;
                         //else//小圆环减半宽
                         //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                         if(ImageDeal[Ysite].Center<4)
                             ImageDeal[Ysite].Center = 4;
                         ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                         break;
                     }
                 }

                 if(ImageDeal[Ysite].Wide>8 &&ImageDeal[Ysite].RightBorder< ImageDeal[Ysite+2].RightBorder)
                 {
                     continue;
                 }
                 else
                 {
                     ImageStatus.OFFLine=Ysite+2;
                     break;
                 }
             }
         }
     }
     if (ImageFlag.image_element_rings_flag == 6)
          {

         for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                  {
                      ImageDeal[Ysite].Center = 15;
                  }
          }
         //环内 小环弯道减半宽 大环不减
     if (ImageFlag.image_element_rings_flag == 7)
     {
//         for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
//                          {
//                             ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2)-7;
//                          }
     }
         //大圆环出环 补线
     if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)    //大圆环
     {
         Repair_Point_Xsite = 20;
         Repair_Point_Ysite = 7;
         for (int Ysite = 40; Ysite > 5; Ysite--)
         {
             if (Pixle[Ysite][28] == 1 && Pixle[Ysite-1][28] == 0)//28
             {
                 Repair_Point_Xsite = 28;
                 Repair_Point_Ysite = Ysite-1;
                 ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
                 break;
             }
         }
         for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
         {
             ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
             ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
         }
     }
 //        //小圆环出环 补线
 //    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 2)    //小圆环
 //    {
 //        Repair_Point_Xsite = 0;
 //        Repair_Point_Ysite = 0;
 //        for (int Ysite = 55; Ysite > 5; Ysite--)
 //        {
 //            if (Pixle[Ysite][15] == 1 && Pixle[Ysite-1][15] == 0)
 //            {
 //                Repair_Point_Xsite = 15;
 //                Repair_Point_Ysite = Ysite-1;
 //                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
 //                break;
 //            }
 //        }
 //        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
 //        {
 //            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
 //            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
 //        }
 //    }
         //已出环 半宽处理
//     if (ImageFlag.image_element_rings_flag == 8)
//               {
//
//              for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
//                       {
//                           ImageDeal[Ysite].Center = 15;
//                       }
//               }
     if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
     {
         for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
         {
             ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
         }
     }
}
//--------------------------------------------------------------
//  @name           Element_Handle_Right_Rings()
//  @brief          整个图像处理的子函数，用来处理右圆环类型.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Right_Rings();
//-------------------------------------------------------------
void Element_Handle_Right_Rings()
{

    /****************判断*****************/
    int num =0 ;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
        }
        if(    ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T' )
            break;
    }
        //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>10)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<8)
    {
        ImageFlag.image_element_rings_flag = 5;
    }
        //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line>15)
    {
        ImageFlag.image_element_rings_flag = 6;
       // ImageStatus.Road_type = RightCirque;
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line<4)
    {
        ImageFlag.image_element_rings_flag = 7;
        //Stop=1;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 45; Ysite > ImageStatus.OFFLine+3; Ysite--)
        {
            if (
//                    ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
//                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
//                 &&
                 ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder

//                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder
//                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder
                )

            {
                        Point_Xsite = ImageDeal[Ysite].LeftBorder;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 22)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
    if (ImageFlag.image_element_rings_flag == 8)
    {
         if (   Straight_Judge(1, ImageStatus.OFFLine+10, 45) < 1
             && ImageStatus.Left_Line < 9
             && ImageStatus.OFFLine < 20)    //右边为直线且截止行（前瞻值）很小
            {ImageFlag.image_element_rings_flag = 9;}

    }
    if(ImageFlag.image_element_rings_flag == 9 )
    {
        int num=0;
        for (int Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = 0;   //出环处理完道路类型清0
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
//            Front_Ring_Continue_Count++;
//            gpio_set_level(Beep, 0);
        }
    }
    /***************************************处理**************************************/
         //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }

        //进环  补线
    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6
       )
    {
        int flag_Xsite_1=0;
        int  flag_Ysite_1=0;
        float Slope_Right_Rings = 0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=Xsite;
                    Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
              break;
            }
        }
        if(flag_Ysite_1==0)
        {
        for(Ysite=ImageStatus.OFFLine+5;Ysite<30;Ysite++)
        {
         if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite+1].IsRightFind=='T'&&ImageDeal[Ysite+2].IsRightFind=='W'
               &&abs(ImageDeal[Ysite].RightBorder-ImageDeal[Ysite+2].RightBorder)>10
         )
         {
             flag_Ysite_1=Ysite;
             flag_Xsite_1=ImageDeal[flag_Ysite_1].RightBorder;
             ImageStatus.OFFLine=Ysite;
             Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
             break;
         }

        }

        }
        //补线
        if(flag_Ysite_1!=0)
        {
            for(Ysite=flag_Ysite_1;Ysite<58;Ysite++)
            {
                ImageDeal[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);
//                if(ImageFlag.ring_big_small==2)//小圆环加半宽
//                    ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
//                else//大圆环不加半宽
                    ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//板块
                if(ImageDeal[Ysite].Center>79)
                    ImageDeal[Ysite].Center=79;
            }
            ImageDeal[flag_Ysite_1].LeftBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A点上方进行扫线
            {
                for(Xsite=ImageDeal[Ysite+1].LeftBorder+8;Xsite>ImageDeal[Ysite+1].LeftBorder-4;Xsite--)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite-1]==0)
                    {
                     ImageDeal[Ysite].LeftBorder=Xsite;
                     ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
//                     if(ImageFlag.ring_big_small==2)//小圆环加半宽
//                         ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//板块
//                     else//大圆环不加半宽
                         ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//板块
                     if(ImageDeal[Ysite].Center>79)
                         ImageDeal[Ysite].Center=79;
                     if(ImageDeal[Ysite].Center<5)
                         ImageDeal[Ysite].Center=5;
                     break;
                    }
                }
                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder>  ImageDeal[Ysite+2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
    if (ImageFlag.image_element_rings_flag == 6)
             {

            for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                     {
                         ImageDeal[Ysite].Center = 63;
                     }
             }
        //环内不处理
    if (ImageFlag.image_element_rings_flag == 7)
    {
        //         for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        //                          {
        //                             ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2)+3;
        //                          }
    }
        //大圆环出环 补线
    if (ImageFlag.image_element_rings_flag == 8)  //大圆环
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 7;
        for (int Ysite = 40; Ysite > 8; Ysite--)
        {
            if (Pixle[Ysite][28] == 1 && Pixle[Ysite-1][28] == 0)
            {
                Repair_Point_Xsite = 28;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //截止行重新规划
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //补线
        {
            ImageDeal[Ysite].LeftBorder = (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            //if(ImageDeal[Ysite].LeftBorder<3){ImageDeal[Ysite].LeftBorder = 3;}
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}

void leftrings_jump()
{
    /***********************判断****************************/
    ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    if(bianmaqi_rings>5&&ImageFlag.image_element_rings_flag == 1)
    {
        icm_x = 0;
    }
    if(bianmaqi_rings>bianmaqi_ring&&ImageFlag.image_element_rings_flag == 1)
    {

        ImageFlag.image_element_rings_flag = 2;
//        keep_rings = 1;
        bianmaqi_rings = 0;
//        icm_z = 0;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }
    if(icm_x>gypo_left&&ImageFlag.image_element_rings_flag == 2)
        {
//            keep_rings = 1;
            ImageFlag.image_element_rings_flag = 3;
//            SystemData.Stop = 1; //停车标志位
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        }
    if(bianmaqi_rings>(bianmaqi_ring-1000)&&ImageFlag.image_element_rings_flag == 3)
        {
            ImageFlag.image_element_rings_flag = 4;
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        }
//    if(ImageFlag.image_element_rings_flag == 4)
//        {
//            ImageFlag.image_element_rings_flag = 5;
//            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
//        }

    /***********************执行****************************/
    if(ImageFlag.image_element_rings_flag == 1)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite]-5;
                }
    }
    if(ImageFlag.image_element_rings_flag == 2)   //如果编码器累加满足标准
    {

        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                 {
                       ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite]-7;
                 }
    }
    if(ImageFlag.image_element_rings_flag == 3)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
                    }
    }
    if(ImageFlag.image_element_rings_flag == 4)
    {
        ImageFlag.image_element_rings_flag = 0;
        bianmaqi_rings = 0;
        ImageFlag.image_element_rings = 0;
        ImageFlag.ring_big_small = 0;
        ImageStatus.Road_type = Normol;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }

}
void rightrings_jump()
{
    /***********************判断****************************/
    ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    if(bianmaqi_rings>5&&ImageFlag.image_element_rings_flag == 1)
    {
        icm_x = 0;
    }
    if(bianmaqi_rings>bianmaqi_ring&&ImageFlag.image_element_rings_flag == 1)
    {

        ImageFlag.image_element_rings_flag = 2;
//        keep_rings = 1;
        bianmaqi_rings = 0;
//        icm_x = 0;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }
    if(icm_x>gypo_right&&ImageFlag.image_element_rings_flag == 2)
        {
//            keep_rings = 1;
            ImageFlag.image_element_rings_flag = 3;
//            SystemData.Stop = 1; //停车标志位
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        }
    if(bianmaqi_rings>(bianmaqi_ring-1000)&&ImageFlag.image_element_rings_flag == 3)
        {
            ImageFlag.image_element_rings_flag = 4;
            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
        }
//    if(ImageFlag.image_element_rings_flag == 4)
//        {
//            ImageFlag.image_element_rings_flag = 5;
//            ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
//        }

    /***********************执行****************************/
    if(ImageFlag.image_element_rings_flag == 1)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite]+5;
                }
    }
    if(ImageFlag.image_element_rings_flag == 2)   //如果编码器累加满足标准
    {

        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                 {
                       ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite]+7;
                 }
    }
    if(ImageFlag.image_element_rings_flag == 3)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
                    }
    }
    if(ImageFlag.image_element_rings_flag == 4)
    {
        ImageFlag.image_element_rings_flag = 0;
        bianmaqi_rings = 0;
        ImageFlag.image_element_rings = 0;
        ImageFlag.ring_big_small = 0;
        ImageStatus.Road_type = Normol;
        ips114_show_int(140,117,ImageFlag.image_element_rings_flag,1);
    }
}
int Fork_dowm = 0;

//用于加速的直道检测
float variance, variance_acc;  //方差
void Straightacc_Test(void) {
  int sum = 0;
  for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--)
  {
    sum += (ImageDeal[Ysite].Center - ImageStatus.MiddleLine) *(ImageDeal[Ysite].Center - ImageStatus.MiddleLine);
  }
  variance_acc = (float)sum / (54 - ImageStatus.OFFLine);
  if ( variance_acc < ImageStatus.variance_acc
    && ImageStatus.OFFLine <= 7
    && ImageStatus.Left_Line < 1             //4
    && ImageStatus.Right_Line < 1
    )
  {
      ImageStatus.straight_acc = 1;
  } else
      ImageStatus.straight_acc = 0;
}

void Element_Test(void) {
  if (
      ImageStatus.Road_type != Cross  //赛道类型清0
      &&ImageStatus.Road_type != LeftCirque
      &&ImageStatus.Road_type != RightCirque
      )
    Straightacc_Test();//直道检测


  if (                                        //圆环检测
       Element_on ==1
      &&ImageStatus.OFFLine < 5
      && ImageStatus.WhiteLine<3
      && ImageDeal[52].IsLeftFind != 'W'
      && ImageDeal[53].IsLeftFind != 'W'
      && ImageDeal[54].IsLeftFind != 'W'
      && ImageDeal[55].IsLeftFind != 'W'
      &&ImageDeal[56].IsLeftFind != 'W'
      && ImageDeal[57].IsLeftFind != 'W'
      && ImageDeal[58].IsLeftFind != 'W'

      )
  {
      if (
                    ImageStatus.Right_Line < 2
                  &&ImageStatus.Left_Line > 13 // 13
                  &&Cirque_or_Cross(1,ImageStatus.Left_Line)>70   //白点数量
              )
      {
          Element_Judgment_Left_Rings();           //左圆环检测
          ips114_show_string(140,100,"1");
      }
      else
      {
//          ips114_show_string(140,100,"0");
      }
      if (
              ImageStatus.Left_Line <  2
          && ImageStatus.Right_Line > 17 //13
          &&Cirque_or_Cross(2,ImageStatus.Right_Line)>120        //白点数量
          )
      Element_Judgment_Right_Rings();          //右圆环检测
      ips114_show_int(0,117,Ring_Help_Flag,1);
  }
  ips114_show_int(100,117,ImageStatus.Road_type,2);
}



void Element_Judgment_Zebra()//斑马线判断
{
    if(
            ImageStatus.Road_type == LeftCirque  //防止出环标志位没加导致一直占用标志位
            ||ImageStatus.Road_type == RightCirque
            ||ImageStatus.Road_type == Ramp
            ||banmaxian_on == 0
)
        return;
    int NUM = 0,net = 0;

    for (int Ysite = 45; Ysite <55 ; Ysite++)          // 25   35
    {
        net = 0;
        for (int Xsite =ImageDeal[Ysite].LeftBoundary-5 ; Xsite < ImageDeal[Ysite].RightBoundary+5; Xsite++)
        {
            if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 1)
            {
                net++;
                if (net > 4)
                {
                    NUM++;
                    ImageStatus.Zebra_read = 1;
                    ImageStatus.Road_type != Zebra_Flag;
                }
            }
        }
    }
}
void Element_Handle()   //环岛方案选择
{

   if (ImageFlag.image_element_rings == 1)
   {
//       ips114_show_string(205,90,"huan_l");

//       if(
//               rings_choose == 0||
//               rings_choose == 2
//               )
           Element_Handle_Left_Rings();
//           Element_Handle_Left_Rings_1();
//       else
//       {
//           leftrings_jump();   //固定判断左环
//       }
//      ImageStatus.Zebra_read = 1;
//      menu_flag_--;
   }
   else if(ImageFlag.image_element_rings == 2)
   {
//
//       if(
//               rings_choose == 0||
//               rings_choose == 1
//       )
//           Element_Handle_Right_Rings_1();     //右圆环判断
             Element_Handle_Right_Rings();
//       else
//       {
//           rightrings_jump();   //固定判断右环
//       }

//      ImageStatus.Zebra_read = 1;
//      menu_flag_--;

   }
//         Cross_Handle();
}

void straight_speed()
{
    if(ImageStatus.straight_acc == 1)               //直道
    {
       Speed_Goal = Speed_Goal_add;
    }
    if(ImageStatus.straight_acc == 0)
    {
       Speed_Goal = Speed_Goal_;         //弯道
//       speed_=0;
    }
}
/*****************误差按权重重新整定**********************/
void GetDet() {
  float DetTemp = 0;
  int TowPoint = 0;
  float SpeedGain = 0;
  float UnitAll = 0;

  SpeedGain=(SystemData.SpeedData.nowspeed - SystemData.SpeedData.MinSpeed) * 0.2 +0.5 ;//根据速度调整前瞻的因子   速度高于80  增益为负数  前瞻加长

    if (SpeedGain >= 3)
      SpeedGain = 3;
    else if (SpeedGain <= -1)
      SpeedGain = -1;

    if ((ImageStatus.Road_type == RightCirque ||ImageStatus.Road_type == LeftCirque)&&ImageStatus.CirqueOff == 'F')
    TowPoint = 15;                                                                      //圆环前瞻

    else if (ImageStatus.Road_type == Straight)
    TowPoint = SystemData.straighet_towpoint;

  else if(ImageStatus.Road_type ==Cross_ture)
  {
      TowPoint=22;
  }
else
    TowPoint = ImageStatus.TowPoint-SpeedGain;                                          //速度越快前瞻越长

  if (TowPoint < ImageStatus.OFFLine)
    TowPoint = ImageStatus.OFFLine + 1;

  if (TowPoint >= 49)
    TowPoint = 49;

  if ((TowPoint - 5) >= ImageStatus.OFFLine) {                                          //前瞻取设定前瞻还是可视距离  需要分情况讨论
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
      DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

  } else if (TowPoint > ImageStatus.OFFLine) {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
      DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  } else if (ImageStatus.OFFLine < 49) {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

  } else
    DetTemp =ImageStatus.Det_True;                                                     //如果是出现OFFLine>50情况，保持上一次的偏差值

  ImageStatus.Det_True = DetTemp;                                                      //此时的解算出来的平均图像偏差

  ImageStatus.TowPoint_True = TowPoint;                                                //此时的前瞻
}

//void bianmaqi_jump()
//        {
//            if(
//                    ImageFlag.image_element_rings_flag == 1||
//                    ImageFlag.image_element_rings_flag == 3
//                    )
//                bianmaqi_rings += abs((speed1 + speed2)/40);
//        }

//float huandaopanduan(int x,int dian1,int dian2,int dian3)          //判断环岛   没用
//{
//    if(x==1)
//    {
//           int p1[2] = {dian1, ImageDeal[dian1].LeftBorder};
//           int p2[2] = {dian2, ImageDeal[dian2].LeftBorder};
//           int p3[2] = {dian3, ImageDeal[dian3].LeftBorder};
//
//           int a[2] = {p2[0] - p1[0], p2[1] - p1[1]};
//           int b[2] = {p3[0] - p2[0], p3[1] - p2[1]};
//
//           float cos_value = (a[0] * b[0] + a[1] * b[1]) / (sqrt(a[0] * a[0] + a[1] * a[1]) * sqrt(b[0] * b[0] + b[1] * b[1]));
//           float angle = acos(cos_value);
//
//           float curvature = 2 * sin(angle) / sqrt(pow(p3[0]-p1[0], 2) + pow(p3[1]-p1[1], 2));
//
//           return curvature;
//
////          ImageDeal[dian2].LeftBorder  ImageDeal[dian3].LeftBorder
//    }
//    else if(x==2)
//    {
//                   int p1[2] = {dian1, ImageDeal[dian1].RightBorder};
//                   int p2[2] = {dian2, ImageDeal[dian2].RightBorder};
//                   int p3[2] = {dian3, ImageDeal[dian3].RightBorder};
//
//                   int a[2] = {p2[0] - p1[0], p2[1] - p1[1]};
//                   int b[2] = {p3[0] - p2[0], p3[1] - p2[1]};
//
//                   float cos_value = (a[0] * b[0] + a[1] * b[1]) / (sqrt(a[0] * a[0] + a[1] * a[1]) * sqrt(b[0] * b[0] + b[1] * b[1]));
//                   float angle = acos(cos_value);
//                   // 计算曲率
//                   float curvature = 2 * sin(angle) / sqrt(pow(p3[0]-p1[0], 2) + pow(p3[1]-p1[1], 2));
//
//                   return curvature;
//
//    }
//}



float Det = 0;
void ImageProcess(void)
{
    compressimage();          //对图像进行压缩
    ImageStatus.OFFLine = 2;  //限制图像顶端
    ImageStatus.WhiteLine = 0;
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)//从下往上搜线（因为第60行是最上面）
  {
    ImageDeal[Ysite].IsLeftFind = 'F';
    ImageDeal[Ysite].IsRightFind = 'F';
    ImageDeal[Ysite].LeftBorder = 0;
    ImageDeal[Ysite].RightBorder = 79;
    ImageDeal[Ysite].LeftTemp = 0;
    ImageDeal[Ysite].RightTemp = 79;
    ImageDeal[Ysite].close_LeftBorder = 0;
    ImageDeal[Ysite].close_RightBorder = 79;
  }                     //边界与标志位初始化

  Get01change_dajin();  //图像二值化
  DrawLinesFirst();     //绘制底边
  DrawLinesProcess();   //搜边线

  Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);//58行位底行

  Element_Test();                //元素判断
  DrawExtensionLine();           //绘制延长线，补线
  RouteFilter();                 //中线滤波平滑


  /***元素处理*****/
  if(Zebras<320)    //斑马线判断延时
      Zebras++;
  if(Zebras>300)
  {
      Element_Judgment_Zebra();  //斑马线判断
      if(speed1<20 || speed2<20)          //如果俩同时小于20，堵转停车
          {
              Zebras_++;
              if(Zebras_>100)
              {
                  ImageStatus.Zebra_read = 1;
                  ImageStatus.Road_type != Zebra_Flag;
              }
          }
  }

  Element_Handle();  //环岛执行
//  bianmaqi_jump();  //编码器累加
  straight_speed();   //直道加速处理
  GetDet();               //获取动态前瞻  并且计算图像偏差 3us

  ImageStatus.Foresight = ((((ImageDeal[ImageStatus.OFFLine + 1].Center) +
                             (ImageDeal[ImageStatus.OFFLine + 2].Center) +
                             (ImageDeal[ImageStatus.OFFLine + 3].Center)) /3) -40);

  ImageStatus.Det_all = (ImageStatus.Foresight + 40) - ImageDeal[54].Center;
  ImageStatus.Det_all_k =(float)(ImageStatus.Det_all) / (ImageStatus.OFFLine + 2 - 54) * 30;
  ImageStatus.Foresight = abs(ImageStatus.Foresight);
}



void mt9v03x_set_exposure_time_set(exposure_time)   //摄像头曝光度设置
{
    mt9v03x_set_exposure_time(exposure_time);
}

int half_wide_count(int count_y)    //计算赛道半宽，用于在调整摄像头角度之后调整半宽数组
{
    int count_x;
    count_x = 39 - ImageDeal[count_y].LeftBorder+ImageStatus.Det_True-39;
    return count_x;
}

