/*
 * image.c
 *
 *  Created on: 2024年5月26日
 *      Author: point
 */
#include "headfile.h"
#include "image.h"

int ImageScanInterval = 5;                         //扫边范围    上一行的边界+-ImageScanInterval
int ImageScanInterval_Cross;                   //270°的弯道后十字的扫线范围

uint8 Image_Use[LCDH][LCDW];      //灰度图像
uint8 Pixle[LCDH][LCDW];          //用于处理的二值化图像
uint8 ExtenLFlag = 0;  //是否左延长标志
uint8 ExtenRFlag = 0;  //是否右延长标志
static int Ysite = 0, x = 0;               //Y坐标=列
static uint8* pixtemp;                         //保存单行图像
static int IntervalLow = 0, IntervalHigh = 0;  //定义高低扫描区间
static int ytemp = 0;                          //存放行
static int TFSite = 0, FTSite = 0;             //存放行
static float DetR = 0, DetL = 0;               //存放斜率
static int bottomright = 79,             //59行右边界
bottomleft = 0,                          //59行左边界
BottomCenter = 0;                              //59行中点
ImageDealDatatypedef ImageDeal[60];            //记录单行的信息
ImageStatustypedef ImageStatus;
Barriertypedef RoadBarrier;
Flagtypedef RoadFlag;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      图像压缩
//  @return     void
//  @since      v2.0
//  Sample usage:   void compressimage();
//-------------------------------------------------------------------------------------------------------------------
void compressimage(void)
{
  int i, j, row, line;
  const float div_h = MT9V03X_DVP_H / LCDH, div_w = MT9V03X_DVP_W / LCDW;
  for (i = 0; i < LCDH; i++)
  {
    row = i * div_h + 0.5;
    for (j = 0; j < LCDW; j++)
    {
      line = j * div_w + 0.5;
      Image_Use[i][j] = mt9v03x_image_dvp[row][line];
    }
  }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     快速大津求阈值，来自山威
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//山威快速大津
  @note      据说比传统大津法快一点，实测使用效果差不多
-------------------------------------------------------------------------------------------------------------------*/
uint8_t my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    uint8_t threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return threshold;
}

/***************************************************************
* 函数名称：bin_block(uint8_t Threshold)
* 函数输入：无
* 函数输出：无
* 功能说明：二值化处理图像像素点
***************************************************************/
//二值化
void Get01change() {
  uint8 thre;
  uint8 i, j;
  thre = my_adapt_threshold(Image_Use[0], LCDH, LCDW);
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
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
    if (deltaTmp < deltaMax) {
      break;
    }
  }
  return threshold;
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
  pixtemp = Pixle[59];
  if (*(pixtemp + ImageSensorMid) == 0)                 //如果底边图像中点为黑，异常情况
  {
    for (x = 0; x < ImageSensorMid; x++)    //找左右边线
    {
      if (*(pixtemp + ImageSensorMid - x) != 0)     //一旦找到左或右赛道到中心距离，就break
        break;                                          //并且记录x
      if (*(pixtemp + ImageSensorMid + x) != 0)
        break;
    }

    if (*(pixtemp + ImageSensorMid - x) != 0)       //赛道如果在左边的话
    {
      bottomright = ImageSensorMid - x + 1;   // 59行右边线有啦
      for (x = bottomright; x > 0; x--)  //开始找59行左边线
      {
        if (*(pixtemp + x) == 0 &&
            *(pixtemp + x - 1) == 0)                //连续两个黑点，滤波
        {
          bottomleft = x;                     //左边线找到
          break;
        } else if (x == 1) {
          bottomleft = 0;                         //搜索到最后了，看不到左边线，左边线认为是0
          break;
        }
      }
    } else if (*(pixtemp + ImageSensorMid + x) != 0)  //赛道如果在右边的话
    {
      bottomleft = ImageSensorMid + x - 1;    // 59行左边线有啦
      for (x = bottomleft; x < 79; x++)  //开始找59行左边线
      {
        if (  *(pixtemp + x) == 0
            &&*(pixtemp + x + 1) == 0)              //连续两个黑点，滤波
        {
          bottomright = x;                    //右边线找到
          break;
        } else if (x == 78) {
          bottomright = 79;                       //搜索到最后了，看不到右边线，左边线认为是79
          break;
        }
      }
    }
  }
  else                                                //左边线中点是白的，比较正常的情况
  {
    for (x = 79; x >ImageSensorMid; x--)   //一个点一个点地搜索右边线
    {
      if (  *(pixtemp + x) == 1
          &&*(pixtemp + x - 1) == 1)                //连续两个黑点，滤波     //两个白点
      {
        bottomright = x;                      //找到就记录
        break;
      } else if (x == 40) {
        bottomright = 39;                         //找不到认为79
        break;
      }
    }
    for (x = 0; x < ImageSensorMid; x++)    //一个点一个点地搜索左边线
    {
      if (  *(pixtemp + x) == 1
          &&*(pixtemp + x + 1) == 1)                //连续两个黑点，滤波
      {
        bottomleft = x;                       //找到就记录
        break;
      } else if (x == 38) {
        bottomleft = 39;                           //找不到认为0
        break;
      }
    }
  }
  BottomCenter =(bottomleft + bottomright) / 2;   // 59行中点直接取平均
  ImageDeal[59].LeftBorder = bottomleft;                //在数组里面记录一下信息，第一行特殊一点而已
  ImageDeal[59].RightBorder = bottomright;
  ImageDeal[59].Center = BottomCenter;                        //确定最底边
  ImageDeal[59].Wide = bottomright - bottomleft;  //存储宽度信息
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';



  for (Ysite = 58; Ysite > 54; Ysite--)                       //由中间向两边确定底边五行
  {
    pixtemp = Pixle[Ysite];
    for (x = 79; x > ImageDeal[Ysite + 1].Center;
         x--)                                             //和前面一样的搜索
    {
      if (*(pixtemp + x) == 1 && *(pixtemp + x - 1) == 1) {
        ImageDeal[Ysite].RightBorder = x;
        break;
      } else if (x == (ImageDeal[Ysite + 1].Center+1)) {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    for (x = 0; x < ImageDeal[Ysite + 1].Center;
         x++)                                             //和前面一样的搜索
    {
      if (*(pixtemp + x) == 1 && *(pixtemp + x  +1) == 1) {
        ImageDeal[Ysite].LeftBorder = x;
        break;
      } else if (x == (ImageDeal[Ysite + 1].Center-1)) {
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
}

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
  ImageStatus.OFFLine = 5;
  ImageStatus.WhiteLine = 0;
  ImageStatus.RWLine = 0;
  ImageStatus.LWLine = 0;

  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)            //前5行处理过了，下面从55行到（设定的不处理的行OFFLine）
  {                        //太远的图像不稳定，OFFLine以后的不处理
    pixtemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                          // 0左1右

      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;             //从上一行右边线-Interval的点开始（确定扫描开始点）
      IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           //到上一行右边线+Interval的点结束（确定扫描结束点）

//      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval_Cross;       //从上一行右边线-Interval_Cross的点开始（确定扫描开始点）
//      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;    //到上一行右边线+Interval_Cross的点开始（确定扫描开始点）


    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(pixtemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     //扫右边线

    IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                //从上一行左边线-5的点开始（确定扫描开始点）
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               //到上一行左边线+5的点结束（确定扫描结束点）

    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(pixtemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);

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
    if ( ImageDeal[Ysite].IsLeftFind == 'H'
         ||ImageDeal[Ysite].IsRightFind == 'H') {
      if (ImageDeal[Ysite].IsLeftFind == 'H')
      //如果左边线大跳变
        for (x = (ImageDeal[Ysite].LeftBorder + 1);
             x <= (ImageDeal[Ysite].RightBorder - 1);
             x++)                                                           //左右边线之间重新扫描
        {
          if ((*(pixtemp + x) == 0) && (*(pixtemp + x + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder =x;                                 //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(pixtemp + x) != 0)                                   //一旦出现白点则直接跳出
            break;
          else if (x ==(ImageDeal[Ysite].RightBorder - 1))
          {
             ImageDeal[Ysite].IsLeftFind = 'T';
             ImageDeal[Ysite].LeftBorder =x;
            break;
          }
        }

      if (ImageDeal[Ysite].IsRightFind == 'H')
        for (x = (ImageDeal[Ysite].RightBorder - 1);
             x >= (ImageDeal[Ysite].LeftBorder + 1); x--) {
          if ((*(pixtemp + x) == 0) && (*(pixtemp + x - 1) != 0)) {
            ImageDeal[Ysite].RightBorder =
                x;                    //如果右边线的左边还有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } else if (*(pixtemp + x) != 0)
            break;
          else if (x == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = x;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
    }

 /***********重新确定无边行************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;

    if (    ImageDeal[Ysite].IsRightFind == 'W'
          &&Ysite > 10
          &&Ysite < 50
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

    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 )    //下面同理  左边界
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

    /*   丢边数量统计               */
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W')
          ImageStatus.WhiteLine++;  //要是左右都无边，丢边数+1
    if(ImageDeal[Ysite].IsLeftFind == 'W' && Ysite <45 && Ysite > 15 )
        ImageStatus.LWLine++;
    if(ImageDeal[Ysite].IsRightFind == 'W' && Ysite <45 && Ysite > 15 )
        ImageStatus.RWLine++;

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
}

void DrawExtensionLine(void)        //绘制延长线并重新确定中线 ，把补线补成斜线
{

    if (ImageStatus.WhiteLine >= 8)
      TFSite = 55;
    if (ExtenLFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)                       //从第五行开始网上扫扫到顶边下面两行   多段补线
                                          //不仅仅只有一段
      {
        pixtemp = Pixle[Ysite];           //存当前行
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

    if (ImageStatus.WhiteLine >= 8)
      TFSite = 55;
    if (ExtenRFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)               //从第五行开始网上扫扫到顶边下面两行
      {
        pixtemp = Pixle[Ysite];  //存当前行

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

  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /2;                                //扫描结束，把这一块经优化之后的中间值存入
    ImageDeal[Ysite].Wide =-ImageDeal[Ysite].LeftBorder +ImageDeal[Ysite].RightBorder;                                       //把优化之后的宽度存入
      }

}

void roadblock_test(void)
{
    if(RoadFlag.zebra_flag == 1 || RoadFlag.fork_flag == 1)
        return;

    RoadBarrier.barrier = 'F';
    RoadBarrier.left_barrier = 'F';
    RoadBarrier.right_barrier = 'F';
    RoadBarrier.flag = 0;

    RoadBarrier.left_bridge = 'F';
    RoadBarrier.right_bridge = 'F';
    RoadBarrier.rbridge = 'F';
    RoadBarrier.lbridge = 'F';

    RoadFlag.block_flag = 0;
    RoadFlag.bridge_flag = 0;
//    RoadFlag.bridge_flag2 = 0;

    uint8_t net = 0;

    uint8_t rtemp = 39, ltemp = 39, wid = 0;
//    uint8_t rerr = 0, lerr = 0, werr = 0;

if(ImageStatus.Road_type != Block)
{
    pixtemp = Pixle[barrier_line];//障碍行检测

    RoadBarrier.bridgeborder = ImageDeal[barrier_line].Center;

    for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder - 4); x++)
    {
        if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 1 && *(pixtemp + x - 1) == 0)
        {
            rtemp = x;

            RoadBarrier.right_barrier = 'T';         //障碍标志位
            RoadBarrier.right_bridge = 'T';
            break;
        }
        else if (x == (ImageDeal[barrier_line].RightBorder - 4))//9
        {
            RoadBarrier.right_barrier = 'F';
            RoadBarrier.right_bridge = 'F';
            break;
        }
    }

    for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder); x++)
    {
        if(*(pixtemp + x) == 1 && *(pixtemp + x + 1) == 0 && *(pixtemp + x - 1) == 1)
        {
            RoadBarrier.right_bridge = 'T';
            RoadBarrier.bridgeborder = x;
            break;
        }
        else if (x == (ImageDeal[barrier_line].RightBorder - 1))//9
        {
            RoadBarrier.right_bridge = 'F';
            break;
        }
    }

    for (x = ImageSensorMid; x > (ImageDeal[barrier_line].LeftBorder + 5); x--)
    {
        if(*(pixtemp + x) == 1 && *(pixtemp + x + 1) == 0 && *(pixtemp + x - 1) == 1)
        {
            ltemp = x;
            RoadBarrier.left_barrier = 'T';         //障碍标志位
            RoadBarrier.left_bridge = 'T';
            break;
        }
        else if (x == (ImageDeal[barrier_line].LeftBorder + 9))
        {
            RoadBarrier.left_barrier = 'F';
            RoadBarrier.left_bridge = 'F';
            break;
        }
    }


    if(RoadBarrier.right_barrier == 'T' && RoadBarrier.left_barrier == 'T')
    {
        wid = rtemp - ltemp;        //计算障碍的宽度
//        lerr = ltemp - ImageDeal[barrier_line].LeftBorder;
//        rerr = rtemp - ImageDeal[barrier_line].RightBorder;
//        werr = my_abs(rerr - lerr);
    }

     for (int Ysite = 24; Ysite < 28; Ysite++)
     {
         for (int Xsite =ImageDeal[Ysite].LeftBorder + 2; Xsite < ImageDeal[Ysite].Center - 2; Xsite++)
         {
             if (Pixle[Ysite][Xsite] == 0)
             {
                 net++;
             }
         }
     }

    //是不是还可以再加上检测此时路道的状态，'T'之类的
    if((RoadBarrier.right_barrier == 'T')
        && (RoadBarrier.left_barrier == 'T')
        &&(wid > 5)                                     //宽度限制
        && (ImageDeal[barrier_line].IsLeftFind == 'T')
        && (ImageDeal[barrier_line].IsRightFind == 'T')
        && (wid < ImageDeal[barrier_line].Wide)
        && (ImageDeal[barrier_line - 5].IsLeftFind == 'T')
        && (ImageDeal[barrier_line - 5].IsRightFind == 'T')
        && (ImageDeal[barrier_line + 5].IsLeftFind == 'T')
        && (ImageDeal[barrier_line + 5].IsRightFind == 'T')
//        && (Straight_Judge(1, 25, 50) < 1)
//        && (Straight_Judge(2, 25, 50) < 1)
        )
    {
        RoadBarrier.barrier = 'T';
        RoadBarrier.flag = 1;
    }

    else if(RoadBarrier.right_bridge == 'T'
      && RoadBarrier.left_bridge == 'F'
      && net < 4
    )
    {
        RoadBarrier.rbridge = 'T';
        RoadFlag.bridge_flag = 1;
        ImageStatus.Road_type = Block;
    }
}

    if(ImageStatus.Road_type == Block)
    {
        pixtemp = Pixle[barrier_line];//障碍行检测

        for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder - 4); x++)
        {
            if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 1 && *(pixtemp + x - 1) == 0)
            {
                rtemp = x;
                RoadBarrier.right_barrier = 'T';         //障碍标志位
                break;
            }
            else if (x == (ImageDeal[barrier_line].RightBorder - 4))//9
            {
                RoadBarrier.right_barrier = 'F';
                break;
            }
        }

        for (x = ImageSensorMid; x > (ImageDeal[barrier_line].LeftBorder + 5); x--)
            {
                if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 0 && *(pixtemp + x - 1) == 1)
                {
                    ltemp = x;
                    RoadBarrier.left_barrier = 'T';         //障碍标志位
                    break;
                }
                else if (x == (ImageDeal[barrier_line].LeftBorder + 9))
                {
                    RoadBarrier.left_barrier = 'F';
                    break;
                }
            }

        if(RoadBarrier.right_barrier == 'T' && RoadBarrier.left_barrier == 'T')
        {
            wid = rtemp - ltemp;        //计算障碍的宽度
        }

        if((RoadBarrier.right_barrier == 'T')
            && (RoadBarrier.left_barrier == 'T')
            && (ImageDeal[barrier_line].IsLeftFind == 'T')
            && (ImageDeal[barrier_line].IsRightFind == 'T')
//            && (wid < ImageDeal[ImageStatus.OFFLine + 12].Wide)
//            && (ImageDeal[ImageStatus.OFFLine + 12 + 5].IsLeftFind == 'T')
//            && (ImageDeal[ImageStatus.OFFLine + 12 + 5].IsRightFind == 'T')
            )
        {
            RoadFlag.block_flag = 1;
            ImageStatus.Road_type = Normol;
        }
    }

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           barrier_detection
//  @brief          检测障碍
//  @parameter      void
//  Sample usage:   barrier_detection();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void barrier_detection(void)
{
    if(RoadFlag.zebra_flag == 1 || RoadFlag.fork_flag == 1)
        return;

    RoadBarrier.barrier = 'F';
    RoadBarrier.left_barrier = 'F';
    RoadBarrier.right_barrier = 'F';
    RoadBarrier.flag = 0;

    RoadBarrier.left_bridge = 'F';
    RoadBarrier.right_bridge = 'F';
    RoadBarrier.rbridge = 'F';
    RoadBarrier.lbridge = 'F';

    RoadFlag.bridge_flag = 0;
    RoadFlag.bridge_flag2 = 0;

    uint8_t net = 0, NET = 0;

    uint8_t rtemp = 39, ltemp = 39, wid = 0;
//    uint8_t rerr = 0, lerr = 0, werr = 0;

    pixtemp = Pixle[barrier_line];//障碍行检测

    RoadBarrier.bridgeborder = ImageDeal[barrier_line].Center;

    for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder - 4); x++)
    {
        if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 1 && *(pixtemp + x - 1) == 0)
        {
            rtemp = x;

            RoadBarrier.right_barrier = 'T';         //障碍标志位
            break;
        }
        else if (x == (ImageDeal[barrier_line].RightBorder - 4))//9
        {
            RoadBarrier.right_barrier = 'F';
            break;
        }
    }

    for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder); x++)
    {
        if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 0 && *(pixtemp + x - 1) == 1)
        {
            RoadBarrier.right_bridge = 'T';
            RoadBarrier.bridgeborder = x;
            break;
        }
        else if (x == (ImageDeal[barrier_line].RightBorder - 1))//9
        {
            RoadBarrier.right_bridge = 'F';
            break;
        }
    }

    for (x = ImageSensorMid; x > (ImageDeal[barrier_line].LeftBorder + 5); x--)
    {
        if(*(pixtemp + x) == 1 && *(pixtemp + x + 1) == 0 && *(pixtemp + x - 1) == 1)
        {
            ltemp = x;
            RoadBarrier.left_barrier = 'T';         //障碍标志位
            RoadBarrier.left_bridge = 'T';
            break;
        }
        else if (x == (ImageDeal[barrier_line].LeftBorder + 3))
        {
            RoadBarrier.left_barrier = 'F';
            RoadBarrier.left_bridge = 'F';
            break;
        }
    }

//    for (x = ImageSensorMid; x > (ImageDeal[barrier_line].LeftBorder + 5); x--)
//    {
//        if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 1 && *(pixtemp + x - 1) == 0)
//        {
//            RoadBarrier.left_bridge = 'T';
//            break;
//        }
//        else if (x == (ImageDeal[barrier_line].LeftBorder + 3))
//        {
//            RoadBarrier.left_bridge = 'F';
//            break;
//        }
//    }

    if(RoadBarrier.right_barrier == 'T' && RoadBarrier.left_barrier == 'T')
    {
        wid = rtemp - ltemp;        //计算障碍的宽度
    }

     for (int Ysite = 24; Ysite < 28; Ysite++)
     {
         for (int Xsite =ImageDeal[Ysite].LeftBorder + 2; Xsite < ImageDeal[Ysite].Center - 2; Xsite++)
         {
             if (Pixle[Ysite][Xsite] == 0)
             {
                 net++;
             }
         }
     }

     for (int Ysite = 24; Ysite < 28; Ysite++)
     {
         for (int Xsite =ImageDeal[Ysite].Center + 2; Xsite < ImageDeal[Ysite].RightBorder - 2; Xsite++)
         {
             if (Pixle[Ysite][Xsite] == 0)
             {
                 NET++;
             }
         }
     }


    //是不是还可以再加上检测此时路道的状态，'T'之类的
    if((RoadBarrier.right_barrier == 'T')
        && (RoadBarrier.left_barrier == 'T')
        &&(wid > 5)                                     //宽度限制
        && (ImageDeal[barrier_line].IsLeftFind == 'T')
        && (ImageDeal[barrier_line].IsRightFind == 'T')
        && (wid < ImageDeal[barrier_line].Wide)
        && (ImageDeal[barrier_line - 5].IsLeftFind == 'T')
        && (ImageDeal[barrier_line - 5].IsRightFind == 'T')
        && (ImageDeal[barrier_line + 5].IsLeftFind == 'T')
        && (ImageDeal[barrier_line + 5].IsRightFind == 'T')
        && (Straight_Judge(1, 25, 50) < 1)
        && (Straight_Judge(2, 25, 50) < 1)
        )
    {
        RoadBarrier.barrier = 'T';
        RoadBarrier.flag = 1;
    }

    else if(RoadBarrier.right_bridge == 'T'
      && RoadBarrier.left_bridge == 'F'
      && net < 4
    )
    {
        RoadBarrier.rbridge = 'T';
        RoadFlag.bridge_flag = 1;
    }

    else if(RoadBarrier.right_bridge == 'F'
      && RoadBarrier.left_bridge == 'T'
      && NET < 4
    )
    {
        RoadBarrier.lbridge = 'T';
        RoadFlag.bridge_flag2 = 1;
    }

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

//--------------------------------------------------------------
//  @name           Element_Judgment_Zebra()
//  @brief          整个图像判断的子函数，用来判断斑马线
//  @parameter      void
//  @time
//  @Author
//  Sample usage:   Element_Judgment_Zebra();
//--------------------------------------------------------------
void Element_Judgment_Zebra()//斑马线判断
{
//    if(RoadFlag.fork_flag == 1 || RoadFlag.barn_in_flag == 1 || RoadBarrier.flag == 1)
//        return;
    int NUM = 0, net = 0;
    RoadFlag.zebra_flag = 0;

    if(ImageStatus.OFFLine<20)
    {
        for (int Ysite = 38; Ysite < 45; Ysite++)
        {
            net = 0;
            for (int Xsite =ImageDeal[Ysite].LeftBorder + 2; Xsite < ImageDeal[Ysite].RightBorder - 2; Xsite++)
            {
                if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                {
                    net++;
                    if (net > 4)
                        NUM++;
                }
            }
        }
    }

    if (NUM >= 4)
    {
        RoadFlag.zebra_flag = 1;
    }
}

//--------------------------------------------------------------
//  @name           LRoad
//  @brief          整个图像判断的子函数
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   LRoad();
//--------------------------------------------------------------
void zhijiao_test(void)
{
    if(RoadFlag.zebra_flag == 1 && RoadFlag.bridge_flag == 1 && RoadFlag.fork_flag == 1 && RoadBarrier.barrier == 1)
        return ;
    RoadFlag.Lroad_flag = 0;
    RoadFlag.Lroad_flag2 = 0;

    if(ImageStatus.RWLine - ImageStatus.LWLine > 15
    && ImageStatus.OFFLine > 20
    && ImageStatus.WhiteLine <15)
    {
        if(Straight_Judge(1, ImageStatus.OFFLine + 5, 50) < 1
        && Straight_Judge(2, ImageStatus.OFFLine + 5, 50) < 1
           )

            RoadFlag.Lroad_flag = 1;

    }

    if(ImageStatus.LWLine - ImageStatus.RWLine > 11
    && ImageStatus.OFFLine > 20
    && ImageStatus.WhiteLine <15)
    {
        if(Straight_Judge(1, ImageStatus.OFFLine + 5, 50) < 1
        && Straight_Judge(2, ImageStatus.OFFLine + 5, 50) < 1
           )

            RoadFlag.Lroad_flag2 = 1;

    }
}

/*三岔路检测*/
uint8 f1 = 0;  //没用
uint8 f2 = 0;  //没用
uint8 f3 = 0;  //没用
uint8 Fork_in_1 = 0;//近端特征点
uint8 Fork_in_2 = 0;//远端特征点
uint8 Fork_in = 0;  //入三叉标志
uint8 ForkLinePointx_l = 0;//三叉倒三角左边列
uint8 ForkLinePointx_r = 0;//三叉倒三角右边列
int ForkLinePointy = 0;    //三叉倒三角底边行
int Fork1_Y = 0;           //三叉近端特征行
int Fork2_Y = 0;           //三叉远端特征行


void ForkTest() {
  int wide = 0;   //临时变量

  RoadFlag.fork_flag = 0;

    for (Ysite = 53; Ysite > (ImageStatus.OFFLine + 6);Ysite--)   //防止Ysite溢出

    {
      if ((  ImageDeal[Ysite].IsRightFind == 'T'
           &&ImageDeal[Ysite + 1].IsRightFind == 'T')
          ||(ImageDeal[Ysite].IsLeftFind == 'T'
           &&ImageDeal[Ysite + 1].IsLeftFind =='T'))            //进三叉的时候一般会看见左右两边120*的圆角
      {
        if (((ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 6].LeftBorder) >2
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 6].LeftBorder) <8
            &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 6].LeftBorder) >2
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 6].LeftBorder) <8)        //左边的角
            ||((ImageDeal[Ysite - 6].RightBorder - ImageDeal[Ysite].RightBorder) >2
             &&(ImageDeal[Ysite - 6].RightBorder - ImageDeal[Ysite].RightBorder) <8
//            &&(ImageDeal[Ysite + 6].RightBorder - ImageDeal[Ysite].RightBorder) >2       //右边的角 看到一个就可以  因为看到两个经常漏判
//             &&(ImageDeal[Ysite + 6].RightBorder -ImageDeal[Ysite].RightBorder) < 8
             ))     //阈值需要调整
        {
          Fork_in_1 = 'T';  //表示近端的角点特征找到
          Fork1_Y = Ysite;  //记录第一特征点的行数
          break;
        }

        else {
          Fork_in_1 = 'F';  //没找到GG
        }
      }
    }

    //第二特征  找黑色三角块  并运算得到相关图像信息
    for (Ysite = Fork1_Y; Ysite > (ImageStatus.OFFLine);Ysite--)                                          //从第一特征点开始往上搜索
    {
      pixtemp = Pixle[Ysite];
      for (x = ImageDeal[Ysite].LeftBorder; x < 50;x++)                                       //找三叉口黑色三角块
      {
        if ((*(pixtemp + x) != 0) && (*(pixtemp + x + 1) == 0) &&(*(pixtemp + x + 2) == 0))   //找到黑色角快的左边
        {
          ImageDeal[Ysite].Black_Wide_L = x + 1;                                                       //记录此时的左黑边界
          break;
        } else
          ImageDeal[Ysite].Black_Wide_L =ImageDeal[Ysite].Center;                                          //没找到就在中点
      }

      for (x = ImageDeal[Ysite].RightBorder; x > 30;x--)                                       //找三叉口黑色三角块
      {
        if (  (*(pixtemp + x) == 0)
           && (*(pixtemp + x - 1) == 0)
            &&(*(pixtemp + x + 1) != 0))      //找到黑色角快的右边
        {
          ImageDeal[Ysite].Black_Wide_R = x;  //记录此时的右黑边界
          break;
        } else
          ImageDeal[Ysite].Black_Wide_R = ImageDeal[Ysite].Center;                                          //没找到就在中点
      }

      for (x = ImageDeal[Ysite].Black_Wide_L;
           x <= ImageDeal[Ysite].Black_Wide_R; x++) {
        if (ImageDeal[Ysite].Black_Wide_L == ImageDeal[Ysite].Center
            ||ImageDeal[Ysite].Black_Wide_R ==ImageDeal[Ysite].Center)                                       //如果是中点值那么GG因为这是没找到
          break;
        else if ((*(pixtemp + x) == 0))     //数数左黑和右黑之间的黑点数
        {
          wide++;                               //计算找到三角块的本行黑点数
        }
      }

      ImageDeal[Ysite].BlackWide = wide;        //记录这个宽度
      ImageDeal[Ysite].Black_Pro =ImageDeal[Ysite].BlackWide / ImageDeal[Ysite].Wide;                         //图像黑点比例
      wide = 0;                                 //清0
    }

    //判断是否为三叉的黑色三角块
    for (Ysite = Fork1_Y; Ysite >= (ImageStatus.OFFLine + 1); Ysite--)  // g
    {
      if (( ImageDeal[Ysite].BlackWide - ImageDeal[Ysite + 3].BlackWide) >=2 //如果这个左黑和右黑之间黑点数比较多 并且满足三角形的形状
          &&ImageDeal[Ysite].BlackWide > 21
          &&ImageDeal[Ysite + 1].BlackWide > 17
          &&ImageDeal[Ysite - 1].BlackWide > 17
          &&(39 - ImageDeal[Ysite].Black_Wide_L) > 0                        //滤除斜十字
          &&(ImageDeal[Ysite].Black_Wide_R - 39) > 0) {
        ForkLinePointx_r = ImageDeal[Ysite].Black_Wide_R;                   //用于补线的点
        ForkLinePointx_l = ImageDeal[Ysite].Black_Wide_L;
        ForkLinePointy = Ysite;
        Fork2_Y = Ysite;                                                    //记录第二特征点的行数
        if (Fork1_Y - Fork2_Y > 6)
        {
          Fork_in_2 ='T';                                                   //当两特征点行数大于10   才判断为入环特征点  用于防误判
          break;
        }

      } else
        Fork_in_2 = 'F';
    }

    if ((Fork_in_1 == 'T') && (Fork_in_2 == 'T'))
      Fork_in = 'T';
    else if ((Fork_in == 'T') && (Fork_in_2 == 'T'))
      Fork_in = 'T';
    else
      Fork_in = 'F';

    if (Fork_in == 'T')
      ImageStatus.Road_type = Forkin;
    else {
        ImageStatus.Road_type = Normol;
    }


  if (ImageStatus.Road_type == Forkin)
  {
    RoadFlag.fork_flag = 1;
    RoadTask.task2 += 1;
    if(RoadTask.task2 >= 17)
    {
        RoadTask.task2 = 17;
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Fork_Handle
//  @brief          三岔路处理
//  @parameter      void
//  Sample usage:   Fork_Handle();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Fork_Handle() {
  float Det_Fork_L;
  float Det_Fork_R;
  int yuansanshu;
  yuansanshu = RoadTask.fork_rl;         //左走右走
//  yuansanshu = 0;
  if (  (ImageStatus.Road_type == Forkin)
      &&(yuansanshu == 1))                                           //第一圈右补线
  {
    Det_Fork_R = 1.0*(79 - ForkLinePointx_l) / (55 - ForkLinePointy);
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      int temp = (int)(ImageDeal[55].RightBorder - Det_Fork_R * (55 - Ysite));
      if (temp < 0) {
        temp = 0;
      }
      if (temp < ImageDeal[Ysite].RightBorder) {
        ImageDeal[Ysite].RightBorder = temp;
        if (Pixle[Ysite][ImageDeal[Ysite].RightBorder] == 0) {
          for (x = ImageDeal[Ysite].RightBorder;x > ImageDeal[Ysite].LeftBorder; x--) {
            if (Pixle[Ysite][x] == 1
             && Pixle[Ysite][x - 1] == 1) {
              ImageDeal[Ysite].RightBorder = x;
            }
          }
        }
      }
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
      ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder -ImageDeal[Ysite].LeftBorder;  //宽度更新
    }
  }
  if ((ImageStatus.Road_type == Forkin ) &&
         ( yuansanshu == 0))   //第二圈左补线
  {
    Det_Fork_L = 1.1*ForkLinePointx_r / (55 - ForkLinePointy);
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      int temp = (int)(ImageDeal[55].LeftBorder + Det_Fork_L * (55 - Ysite));
      if (temp > 79) {
        temp = 79;
      }
      if (temp > ImageDeal[Ysite].LeftBorder) {
        ImageDeal[Ysite].LeftBorder = temp;
        if (Pixle[Ysite][ImageDeal[Ysite].LeftBorder] == 0) {
          for (x = ImageDeal[Ysite].LeftBorder;x < ImageDeal[Ysite].RightBorder; x++) {
            if (Pixle[Ysite][x] == 1 && Pixle[Ysite][x + 1] == 1) {
              ImageDeal[Ysite].LeftBorder = x;
            }
          }
        }
      }
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
      ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder -ImageDeal[Ysite].LeftBorder;  //宽度更新
    }
  }
}

////---------------------------------------------------------------------------------------------------------------------------------------------------------------
////  @name           barn_in_test
////  @brief          元素检测
////  @parameter      void
////  Sample usage:   barn_in_test();
////---------------------------------------------------------------------------------------------------------------------------------------------------------------
//void barn_in_test(void)
//{
//
//    if((ImageStatus.LWLine - ImageStatus.RWLine > 10)  && Straight_Judge(2,10,50)<1
//            && ImageDeal[30].IsRightFind == 'T'
//            && ImageDeal[35].IsRightFind == 'T'
//            && ImageDeal[25].IsRightFind == 'T'
//            && ImageDeal[30].IsLeftFind == 'W'
//            && ImageDeal[35].IsLeftFind == 'W' )
//    {
//        RoadFlag.barn_in_flag = 1;
//        ImageStatus.Road_type = Barn_in;
//    }
//    else {
//        RoadFlag.barn_in_flag = 0;
//        ImageStatus.Road_type = Normol;
//    }
//}

////---------------------------------------------------------------------------------------------------------------------------------------------------------------
////  @name           barn_out_test
////  @brief          元素检测
////  @parameter      void
////  Sample usage:   barn_out_test();
////---------------------------------------------------------------------------------------------------------------------------------------------------------------
//void bran_out_test(void)
//{
//    if(RoadFlag.zebra_flag == 1 && ImageStatus.WhiteLine > 10
//            && ImageDeal[50].IsLeftFind == 'T'
//            && ImageDeal[50].IsRightFind == 'T'
//            && ImageDeal[30].IsLeftFind == 'W'
//            && ImageDeal[35].IsLeftFind == 'W'
//            && ImageDeal[30].IsRightFind == 'W'
//            && ImageDeal[35].IsRightFind == 'W' )
//    {
//        RoadFlag.barn_out_flag = 1;
//        ImageStatus.Road_type = Barn_out;
//    }
//    else {
//        RoadFlag.barn_out_flag = 0;
//        ImageStatus.Road_type = Normol;
//    }
//}



//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           element_detection
//  @brief          元素检测
//  @parameter      void
//  Sample usage:   element_detection();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void element_detection(void)    //元素检测
{
//    barrier_detection();    //障碍检测
//    roadblock_test();
    zhijiao_test();
    Element_Judgment_Zebra();
    ForkTest();             //三岔路检测
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           RouteFilter
//  @brief
//  @parameter      void
//  Sample usage:   RouteFilter();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void RouteFilter(void) {
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


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           image_process
//  @brief          图像处理
//  @parameter      void
//  Sample usage:   image_process();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void image_process(void)
{
    compressimage();
    Get01change();
    DrawLinesFirst();
    DrawLinesProcess();

    /*元素识别与处理*/
    element_detection();

    if(RoadBarrier.flag != 1
    && RoadFlag.barn_in_flag2 !=1
    && RoadFlag.barn_in_flag != 1
    && RoadFlag.bridge_flag2 != 1
    && RoadFlag.zebra_flag != 1
    && RoadFlag.Lroad_flag != 1
    && RoadFlag.Lroad_flag2 != 1
    && RoadFlag.fork_flag != 1  //
    && (ImageStatus.WhiteLine >= 10)
    )
    {
        DrawExtensionLine();        //补线用了就卡死，想想加条件吧
    }
    RouteFilter();              //中线滤波
}
