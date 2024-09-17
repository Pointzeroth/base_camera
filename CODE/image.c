/*
 * image.c
 *
 *  Created on: 2024��5��26��
 *      Author: point
 */
#include "headfile.h"
#include "image.h"

int ImageScanInterval = 5;                         //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval
int ImageScanInterval_Cross;                   //270��������ʮ�ֵ�ɨ�߷�Χ

uint8 Image_Use[LCDH][LCDW];      //�Ҷ�ͼ��
uint8 Pixle[LCDH][LCDW];          //���ڴ���Ķ�ֵ��ͼ��
uint8 ExtenLFlag = 0;  //�Ƿ����ӳ���־
uint8 ExtenRFlag = 0;  //�Ƿ����ӳ���־
static int Ysite = 0, x = 0;               //Y����=��
static uint8* pixtemp;                         //���浥��ͼ��
static int IntervalLow = 0, IntervalHigh = 0;  //����ߵ�ɨ������
static int ytemp = 0;                          //�����
static int TFSite = 0, FTSite = 0;             //�����
static float DetR = 0, DetL = 0;               //���б��
static int bottomright = 79,             //59���ұ߽�
bottomleft = 0,                          //59����߽�
BottomCenter = 0;                              //59���е�
ImageDealDatatypedef ImageDeal[60];            //��¼���е���Ϣ
ImageStatustypedef ImageStatus;
Barriertypedef RoadBarrier;
Flagtypedef RoadFlag;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ͼ��ѹ��
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
  @brief     ���ٴ������ֵ������ɽ��
  @param     image       ͼ������
             col         �� �����
             row         �У�����
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//ɽ�����ٴ��
  @note      ��˵�ȴ�ͳ��򷨿�һ�㣬ʵ��ʹ��Ч�����
-------------------------------------------------------------------------------------------------------------------*/
uint8_t my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //ע�������ֵ��һ��Ҫ��ԭͼ��
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    uint8_t threshold = 0;
    uint8* data = image;  //ָ���������ݵ�ָ��
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
        }
    }
    //����ÿ������ֵ�ĵ�������ͼ���еı���
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    //�����Ҷȼ�[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
        u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //����ƽ���Ҷ�
        u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
        u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
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
* �������ƣ�bin_block(uint8_t Threshold)
* �������룺��
* �����������
* ����˵������ֵ������ͼ�����ص�
***************************************************************/
//��ֵ��
void Get01change() {
  uint8 thre;
  uint8 i, j;
  thre = my_adapt_threshold(Image_Use[0], LCDH, LCDW);
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (Image_Use[i][j] >
          (thre))  //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
        Pixle[i][j] = 1;  //��
      else
        Pixle[i][j] = 0;  //��
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �Ż��Ĵ��
//  @param      image  ͼ������
//  @param      clo    ��
//  @param      row    ��
//  @param      pixel_threshold ��ֵ����
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
  uint8* data = image;  //ָ���������ݵ�ָ��
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
      //{
      pixelCount[(
          int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
      gray_sum += (int)data[i * width + j];  //�Ҷ�ֵ�ܺ�
      //}
    }
  }

  //����ÿ������ֵ�ĵ�������ͼ���еı���
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }


  //�����Ҷȼ�[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < pixel_threshold; j++) {
    w0 +=
        pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮�� ���������ֵı���
    u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    //����ƽ���Ҷ�
    u1 = u1tmp / w1;    //ǰ��ƽ���Ҷ�
    u = u0tmp + u1tmp;  //ȫ��ƽ���Ҷ�
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

void GetJumpPointFromDet(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)  //��һ��������Ҫ���ҵ����飨80���㣩
                                                                               //�ڶ���ɨ����߻���ɨ�ұ���
{                                                                              //�����ǿ�ʼ�ͽ�����
  int i = 0;
  if (type == 'L')                              //ɨ�������
  {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //�ɺڱ��
      {
        Q->point = i;                           //��¼�����
        Q->type = 'T';                          //��ȷ����
        break;
      } else if (i == (L + 1))                  //����ɨ�����Ҳû�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����м��ǰ׵�
        {
          Q->point = (L + H) / 2;               //��Ϊ��������е�
          Q->type = 'W';                        //����ȷ�������м�Ϊ�ף���Ϊû�б�
          break;
        } else                                  //����ȷ�������м�Ϊ��
        {
          Q->point = H;                         //����м��Ǻڵ�
          Q->type = 'H';                        //�����ֱ�����ֵ����Ϊ�Ǵ�����
          break;
        }
      }
    }
  } else if (type == 'R')                       //ɨ���ұ���
  {
    for (i = L; i <= H; i++)                    //��������ɨ
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //���ɺڵ��׵�����
      {
        Q->point = i;                           //��¼
        Q->type = 'T';
        break;
      } else if (i == (H - 1))                  //����ɨ�����Ҳû�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����м��ǰ׵�
        {
          Q->point = (L + H) / 2;               //�ұ������е�
          Q->type = 'W';
          break;
        } else                                  //����е��Ǻڵ�
        {
          Q->point = L;                         //�����ֱ�����ֵ
          Q->type = 'H';
          break;
        }
      }
    }
  }
}

static uint8 DrawLinesFirst(void) {
  pixtemp = Pixle[59];
  if (*(pixtemp + ImageSensorMid) == 0)                 //����ױ�ͼ���е�Ϊ�ڣ��쳣���
  {
    for (x = 0; x < ImageSensorMid; x++)    //�����ұ���
    {
      if (*(pixtemp + ImageSensorMid - x) != 0)     //һ���ҵ���������������ľ��룬��break
        break;                                          //���Ҽ�¼x
      if (*(pixtemp + ImageSensorMid + x) != 0)
        break;
    }

    if (*(pixtemp + ImageSensorMid - x) != 0)       //�����������ߵĻ�
    {
      bottomright = ImageSensorMid - x + 1;   // 59���ұ�������
      for (x = bottomright; x > 0; x--)  //��ʼ��59�������
      {
        if (*(pixtemp + x) == 0 &&
            *(pixtemp + x - 1) == 0)                //���������ڵ㣬�˲�
        {
          bottomleft = x;                     //������ҵ�
          break;
        } else if (x == 1) {
          bottomleft = 0;                         //����������ˣ�����������ߣ��������Ϊ��0
          break;
        }
      }
    } else if (*(pixtemp + ImageSensorMid + x) != 0)  //����������ұߵĻ�
    {
      bottomleft = ImageSensorMid + x - 1;    // 59�����������
      for (x = bottomleft; x < 79; x++)  //��ʼ��59�������
      {
        if (  *(pixtemp + x) == 0
            &&*(pixtemp + x + 1) == 0)              //���������ڵ㣬�˲�
        {
          bottomright = x;                    //�ұ����ҵ�
          break;
        } else if (x == 78) {
          bottomright = 79;                       //����������ˣ��������ұ��ߣ��������Ϊ��79
          break;
        }
      }
    }
  }
  else                                                //������е��ǰ׵ģ��Ƚ����������
  {
    for (x = 79; x >ImageSensorMid; x--)   //һ����һ����������ұ���
    {
      if (  *(pixtemp + x) == 1
          &&*(pixtemp + x - 1) == 1)                //���������ڵ㣬�˲�     //�����׵�
      {
        bottomright = x;                      //�ҵ��ͼ�¼
        break;
      } else if (x == 40) {
        bottomright = 39;                         //�Ҳ�����Ϊ79
        break;
      }
    }
    for (x = 0; x < ImageSensorMid; x++)    //һ����һ��������������
    {
      if (  *(pixtemp + x) == 1
          &&*(pixtemp + x + 1) == 1)                //���������ڵ㣬�˲�
      {
        bottomleft = x;                       //�ҵ��ͼ�¼
        break;
      } else if (x == 38) {
        bottomleft = 39;                           //�Ҳ�����Ϊ0
        break;
      }
    }
  }
  BottomCenter =(bottomleft + bottomright) / 2;   // 59���е�ֱ��ȡƽ��
  ImageDeal[59].LeftBorder = bottomleft;                //�����������¼һ����Ϣ����һ������һ�����
  ImageDeal[59].RightBorder = bottomright;
  ImageDeal[59].Center = BottomCenter;                        //ȷ����ױ�
  ImageDeal[59].Wide = bottomright - bottomleft;  //�洢�����Ϣ
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';



  for (Ysite = 58; Ysite > 54; Ysite--)                       //���м�������ȷ���ױ�����
  {
    pixtemp = Pixle[Ysite];
    for (x = 79; x > ImageDeal[Ysite + 1].Center;
         x--)                                             //��ǰ��һ��������
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
         x++)                                             //��ǰ��һ��������
    {
      if (*(pixtemp + x) == 1 && *(pixtemp + x  +1) == 1) {
        ImageDeal[Ysite].LeftBorder = x;
        break;
      } else if (x == (ImageDeal[Ysite + 1].Center-1)) {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T';                        //��Щ��Ϣ�洢��������
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /2; //�洢�е�
    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;      //�洢���
  }
  return 'T';
}

/*����׷����µõ�ȫ������*/
static void DrawLinesProcess(void)  //////���ø���
{
  uint8 L_Found_T = 'F';  //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_L_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
  uint8 R_Found_T = 'F';  //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_R_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
  float D_L = 0;           //�ӳ��������б��
  float D_R = 0;           //�ӳ����ұ���б��
  int ytemp_W_L;           //��ס�״��󶪱���
  int ytemp_W_R;           //��ס�״��Ҷ�����
  ExtenRFlag = 0;          //��־λ��0
  ExtenLFlag = 0;
  ImageStatus.OFFLine = 5;
  ImageStatus.WhiteLine = 0;
  ImageStatus.RWLine = 0;
  ImageStatus.LWLine = 0;

  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)            //ǰ5�д�����ˣ������55�е����趨�Ĳ��������OFFLine��
  {                        //̫Զ��ͼ���ȶ���OFFLine�Ժ�Ĳ�����
    pixtemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                          // 0��1��

      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;             //����һ���ұ���-Interval�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
      IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           //����һ���ұ���+Interval�ĵ������ȷ��ɨ������㣩

//      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval_Cross;       //����һ���ұ���-Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
//      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;    //����һ���ұ���+Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩


    LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
    LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
    GetJumpPointFromDet(pixtemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     //ɨ�ұ���

    IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                //����һ�������-5�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               //����һ�������+5�ĵ������ȷ��ɨ������㣩

    LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
    LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
    GetJumpPointFromDet(pixtemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);

    if (JumpPoint[0].type =='W')                                                    //�����������߲��������䣬����10���㶼�ǰ׵�
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                 //�������������һ�е���ֵ
    } else                                                                          //���������
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                             //��¼������
    }

    if (JumpPoint[1].type == 'W')                                                   //��������ұ��߲���������
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;               //�����ұ�������һ�е���ֵ
    } else                                                                          //�ұ�������
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                            //��¼������
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                 //��¼�����Ƿ��ҵ����ߣ�����������
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    //����ȷ����Щ������ı�Ե
    if ( ImageDeal[Ysite].IsLeftFind == 'H'
         ||ImageDeal[Ysite].IsRightFind == 'H') {
      if (ImageDeal[Ysite].IsLeftFind == 'H')
      //�������ߴ�����
        for (x = (ImageDeal[Ysite].LeftBorder + 1);
             x <= (ImageDeal[Ysite].RightBorder - 1);
             x++)                                                           //���ұ���֮������ɨ��
        {
          if ((*(pixtemp + x) == 0) && (*(pixtemp + x + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder =x;                                 //�����һ������ߵ��ұ��кڰ�������Ϊ���Ա���ֱ��ȡ��
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(pixtemp + x) != 0)                                   //һ�����ְ׵���ֱ������
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
                x;                    //����ұ��ߵ���߻��кڰ�������Ϊ���Ա���ֱ��ȡ��
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

 /***********����ȷ���ޱ���************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;

    if (    ImageDeal[Ysite].IsRightFind == 'W'
          &&Ysite > 10
          &&Ysite < 50
          )                     //������ֵ��ޱ���
    {
      if (Get_R_line == 'F')    //��һ֡ͼ��û���ܹ�����һ�׼�ߵĴ���β�����
      {
        Get_R_line = 'T';       //����  һ֡ͼ��ֻ��һ�� ��ΪT
        ytemp_W_R = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsRightFind =='T')  //���ޱ�����������  һ�㶼���бߵ�
            R_found_point++;
        }
        if (R_found_point >8)                      //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�   ���бߵĵ�������8
        {
          D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));
                                                  //��������Щ����������б��
                                                  //�ø��ޱ������ӳ��������׼
          if (D_R > 0) {
            R_Found_T ='T';                       //���б�ʴ���0  ��ô�ҵ��������׼��  ��Ϊ���λ���
                                                  //����һ���������б�ʴ���0  С��0�����Ҳ�����ӳ� û��Ҫ
          } else {
            R_Found_T = 'F';                      //û���ҵ������׼��
            if (D_R < 0)
              ExtenRFlag = 'F';                   //�����־λ����ʮ�ֽǵ㲹��  ��ֹͼ�����õ�
          }
        }
      }
      if (R_Found_T == 'T')
        ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //����ҵ��� ��ô�Ի�׼�����ӳ���

      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�
    }

    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 )    //����ͬ��  ��߽�
    {
      if (Get_L_line == 'F') {
        Get_L_line = 'T';
        ytemp_W_L = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsLeftFind == 'T')
            L_found_point++;
        }
        if (L_found_point > 8)              //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�
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

      LimitL(ImageDeal[Ysite].LeftBorder);  //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);  //�޷�
    }

    /*   ��������ͳ��               */
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W')
          ImageStatus.WhiteLine++;  //Ҫ�����Ҷ��ޱߣ�������+1
    if(ImageDeal[Ysite].IsLeftFind == 'W' && Ysite <45 && Ysite > 15 )
        ImageStatus.LWLine++;
    if(ImageDeal[Ysite].IsRightFind == 'W' && Ysite <45 && Ysite > 15 )
        ImageStatus.RWLine++;

      LimitL(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 7)         //����ȷ�����Ӿ���
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (  ImageDeal[Ysite].RightBorder <= 10
             ||ImageDeal[Ysite].LeftBorder >= 70) {
              ImageStatus.OFFLine = Ysite + 1;
              break;
    }                                        //��ͼ����С��0�������ұߴﵽһ��������ʱ������ֹѲ��
  }
}

void DrawExtensionLine(void)        //�����ӳ��߲�����ȷ������ ���Ѳ��߲���б��
{

    if (ImageStatus.WhiteLine >= 8)
      TFSite = 55;
    if (ExtenLFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)                       //�ӵ����п�ʼ����ɨɨ��������������   ��β���
                                          //������ֻ��һ��
      {
        pixtemp = Pixle[Ysite];           //�浱ǰ��
        if (ImageDeal[Ysite].IsLeftFind =='W')                          //���������߽�ûɨ����ɨ�����ǰ�ɫ��˵������û����߽��
        {
          //**************************************************//**************************************************
          if (ImageDeal[Ysite + 1].LeftBorder >= 70)                    //�����߽�ʵ����̫�ұ�
          {
            ImageStatus.OFFLine = Ysite + 1;
            break;                        //ֱ�����������������
          }
          //************************************************//*************************************************

          while (Ysite >= (ImageStatus.OFFLine + 4))                    //��ʱ��ûɨ������
          {
            Ysite--;                      //��������ɨ
            if (  ImageDeal[Ysite].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 1].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].LeftBorder > 0
                &&ImageDeal[Ysite - 2].LeftBorder <70
                )                                                       //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
            {
              FTSite = Ysite - 2;          //�ѱ�������ĵڶ��д���FTsite
              break;
            }
          }

          DetL =
              ((float)(ImageDeal[FTSite].LeftBorder -
                       ImageDeal[TFSite].LeftBorder)) /
              ((float)(FTSite - TFSite));  //��߽��б�ʣ��е������/�е������
          if (FTSite > ImageStatus.OFFLine)
            for (
                ytemp = TFSite; ytemp >= FTSite; ytemp--)               //�ӵ�һ��ɨ������߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ�����߽��������ֵ
            {
              ImageDeal[ytemp].LeftBorder =
                  (int)(DetL * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite]
                      .LeftBorder;                                      //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
            }
        } else
          TFSite = Ysite + 2;                                           //���ɨ���˱��е���߽磬���д��������棬����б�ʣ�
      }

    if (ImageStatus.WhiteLine >= 8)
      TFSite = 55;
    if (ExtenRFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)               //�ӵ����п�ʼ����ɨɨ��������������
      {
        pixtemp = Pixle[Ysite];  //�浱ǰ��

        if (ImageDeal[Ysite].IsRightFind =='W')                       //��������ұ߽�ûɨ����ɨ�����ǰ�ɫ��˵������û���ұ߽�㣬���Ǵ��������ڵ�
        {
          if (ImageDeal[Ysite + 1].RightBorder <= 10)                 //����ұ߽�ʵ����̫���
          {
            ImageStatus.OFFLine =Ysite + 1;                           //ֱ��������˵�����������������������
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4))                  //��ʱ��ûɨ��������������
          {
            Ysite--;
            if (  ImageDeal[Ysite].IsRightFind == 'T'
                &&ImageDeal[Ysite - 1].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].RightBorder < 70
                &&ImageDeal[Ysite - 2].RightBorder > 10
                )                                                      //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
            {
              FTSite = Ysite - 2;                                      // �ѱ�������ĵڶ��д���FTsite
              break;
            }
          }

          DetR =((float)(ImageDeal[FTSite].RightBorder -ImageDeal[TFSite].RightBorder)) /((float)(FTSite - TFSite));         //�ұ߽��б�ʣ��е������/�е������
          if (FTSite > ImageStatus.OFFLine)
            for (ytemp = TFSite; ytemp >= FTSite;ytemp--)              //�ӵ�һ��ɨ�����ұ߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ����ұ߽��������ֵ
            {
              ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].RightBorder;          //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
            }
        } else
          TFSite =Ysite +2;                                           //������е��ұ߽��ҵ��ˣ���Ѹ�������ڶ��������͸�TFsite
      }

  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /2;                                //ɨ�����������һ�龭�Ż�֮����м�ֵ����
    ImageDeal[Ysite].Wide =-ImageDeal[Ysite].LeftBorder +ImageDeal[Ysite].RightBorder;                                       //���Ż�֮��Ŀ�ȴ���
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
    pixtemp = Pixle[barrier_line];//�ϰ��м��

    RoadBarrier.bridgeborder = ImageDeal[barrier_line].Center;

    for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder - 4); x++)
    {
        if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 1 && *(pixtemp + x - 1) == 0)
        {
            rtemp = x;

            RoadBarrier.right_barrier = 'T';         //�ϰ���־λ
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
            RoadBarrier.left_barrier = 'T';         //�ϰ���־λ
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
        wid = rtemp - ltemp;        //�����ϰ��Ŀ��
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

    //�ǲ��ǻ������ټ��ϼ���ʱ·����״̬��'T'֮���
    if((RoadBarrier.right_barrier == 'T')
        && (RoadBarrier.left_barrier == 'T')
        &&(wid > 5)                                     //�������
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
        pixtemp = Pixle[barrier_line];//�ϰ��м��

        for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder - 4); x++)
        {
            if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 1 && *(pixtemp + x - 1) == 0)
            {
                rtemp = x;
                RoadBarrier.right_barrier = 'T';         //�ϰ���־λ
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
                    RoadBarrier.left_barrier = 'T';         //�ϰ���־λ
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
            wid = rtemp - ltemp;        //�����ϰ��Ŀ��
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
//  @brief          ����ϰ�
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

    pixtemp = Pixle[barrier_line];//�ϰ��м��

    RoadBarrier.bridgeborder = ImageDeal[barrier_line].Center;

    for (x = ImageSensorMid; x < (ImageDeal[barrier_line].RightBorder - 4); x++)
    {
        if(*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 1 && *(pixtemp + x - 1) == 0)
        {
            rtemp = x;

            RoadBarrier.right_barrier = 'T';         //�ϰ���־λ
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
            RoadBarrier.left_barrier = 'T';         //�ϰ���־λ
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
        wid = rtemp - ltemp;        //�����ϰ��Ŀ��
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


    //�ǲ��ǻ������ټ��ϼ���ʱ·����״̬��'T'֮���
    if((RoadBarrier.right_barrier == 'T')
        && (RoadBarrier.left_barrier == 'T')
        &&(wid > 5)                                     //�������
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

/*****************ֱ���ж�******************/
float Straight_Judge(uint8 dir, uint8 start, uint8 end)     //���ؽ��С��1��Ϊֱ��
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
//  @brief          ����ͼ���жϵ��Ӻ����������жϰ�����
//  @parameter      void
//  @time
//  @Author
//  Sample usage:   Element_Judgment_Zebra();
//--------------------------------------------------------------
void Element_Judgment_Zebra()//�������ж�
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
//  @brief          ����ͼ���жϵ��Ӻ���
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

/*����·���*/
uint8 f1 = 0;  //û��
uint8 f2 = 0;  //û��
uint8 f3 = 0;  //û��
uint8 Fork_in_1 = 0;//����������
uint8 Fork_in_2 = 0;//Զ��������
uint8 Fork_in = 0;  //�������־
uint8 ForkLinePointx_l = 0;//���浹���������
uint8 ForkLinePointx_r = 0;//���浹�����ұ���
int ForkLinePointy = 0;    //���浹���ǵױ���
int Fork1_Y = 0;           //�������������
int Fork2_Y = 0;           //����Զ��������


void ForkTest() {
  int wide = 0;   //��ʱ����

  RoadFlag.fork_flag = 0;

    for (Ysite = 53; Ysite > (ImageStatus.OFFLine + 6);Ysite--)   //��ֹYsite���

    {
      if ((  ImageDeal[Ysite].IsRightFind == 'T'
           &&ImageDeal[Ysite + 1].IsRightFind == 'T')
          ||(ImageDeal[Ysite].IsLeftFind == 'T'
           &&ImageDeal[Ysite + 1].IsLeftFind =='T'))            //�������ʱ��һ��ῴ����������120*��Բ��
      {
        if (((ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 6].LeftBorder) >2
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite - 6].LeftBorder) <8
            &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 6].LeftBorder) >2
             &&(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 6].LeftBorder) <8)        //��ߵĽ�
            ||((ImageDeal[Ysite - 6].RightBorder - ImageDeal[Ysite].RightBorder) >2
             &&(ImageDeal[Ysite - 6].RightBorder - ImageDeal[Ysite].RightBorder) <8
//            &&(ImageDeal[Ysite + 6].RightBorder - ImageDeal[Ysite].RightBorder) >2       //�ұߵĽ� ����һ���Ϳ���  ��Ϊ������������©��
//             &&(ImageDeal[Ysite + 6].RightBorder -ImageDeal[Ysite].RightBorder) < 8
             ))     //��ֵ��Ҫ����
        {
          Fork_in_1 = 'T';  //��ʾ���˵Ľǵ������ҵ�
          Fork1_Y = Ysite;  //��¼��һ�����������
          break;
        }

        else {
          Fork_in_1 = 'F';  //û�ҵ�GG
        }
      }
    }

    //�ڶ�����  �Һ�ɫ���ǿ�  ������õ����ͼ����Ϣ
    for (Ysite = Fork1_Y; Ysite > (ImageStatus.OFFLine);Ysite--)                                          //�ӵ�һ�����㿪ʼ��������
    {
      pixtemp = Pixle[Ysite];
      for (x = ImageDeal[Ysite].LeftBorder; x < 50;x++)                                       //������ں�ɫ���ǿ�
      {
        if ((*(pixtemp + x) != 0) && (*(pixtemp + x + 1) == 0) &&(*(pixtemp + x + 2) == 0))   //�ҵ���ɫ�ǿ�����
        {
          ImageDeal[Ysite].Black_Wide_L = x + 1;                                                       //��¼��ʱ����ڱ߽�
          break;
        } else
          ImageDeal[Ysite].Black_Wide_L =ImageDeal[Ysite].Center;                                          //û�ҵ������е�
      }

      for (x = ImageDeal[Ysite].RightBorder; x > 30;x--)                                       //������ں�ɫ���ǿ�
      {
        if (  (*(pixtemp + x) == 0)
           && (*(pixtemp + x - 1) == 0)
            &&(*(pixtemp + x + 1) != 0))      //�ҵ���ɫ�ǿ���ұ�
        {
          ImageDeal[Ysite].Black_Wide_R = x;  //��¼��ʱ���Һڱ߽�
          break;
        } else
          ImageDeal[Ysite].Black_Wide_R = ImageDeal[Ysite].Center;                                          //û�ҵ������е�
      }

      for (x = ImageDeal[Ysite].Black_Wide_L;
           x <= ImageDeal[Ysite].Black_Wide_R; x++) {
        if (ImageDeal[Ysite].Black_Wide_L == ImageDeal[Ysite].Center
            ||ImageDeal[Ysite].Black_Wide_R ==ImageDeal[Ysite].Center)                                       //������е�ֵ��ôGG��Ϊ����û�ҵ�
          break;
        else if ((*(pixtemp + x) == 0))     //������ں��Һ�֮��ĺڵ���
        {
          wide++;                               //�����ҵ����ǿ�ı��кڵ���
        }
      }

      ImageDeal[Ysite].BlackWide = wide;        //��¼������
      ImageDeal[Ysite].Black_Pro =ImageDeal[Ysite].BlackWide / ImageDeal[Ysite].Wide;                         //ͼ��ڵ����
      wide = 0;                                 //��0
    }

    //�ж��Ƿ�Ϊ����ĺ�ɫ���ǿ�
    for (Ysite = Fork1_Y; Ysite >= (ImageStatus.OFFLine + 1); Ysite--)  // g
    {
      if (( ImageDeal[Ysite].BlackWide - ImageDeal[Ysite + 3].BlackWide) >=2 //��������ں��Һ�֮��ڵ����Ƚ϶� �������������ε���״
          &&ImageDeal[Ysite].BlackWide > 21
          &&ImageDeal[Ysite + 1].BlackWide > 17
          &&ImageDeal[Ysite - 1].BlackWide > 17
          &&(39 - ImageDeal[Ysite].Black_Wide_L) > 0                        //�˳�бʮ��
          &&(ImageDeal[Ysite].Black_Wide_R - 39) > 0) {
        ForkLinePointx_r = ImageDeal[Ysite].Black_Wide_R;                   //���ڲ��ߵĵ�
        ForkLinePointx_l = ImageDeal[Ysite].Black_Wide_L;
        ForkLinePointy = Ysite;
        Fork2_Y = Ysite;                                                    //��¼�ڶ������������
        if (Fork1_Y - Fork2_Y > 6)
        {
          Fork_in_2 ='T';                                                   //������������������10   ���ж�Ϊ�뻷������  ���ڷ�����
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
//  @brief          ����·����
//  @parameter      void
//  Sample usage:   Fork_Handle();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void Fork_Handle() {
  float Det_Fork_L;
  float Det_Fork_R;
  int yuansanshu;
  yuansanshu = RoadTask.fork_rl;         //��������
//  yuansanshu = 0;
  if (  (ImageStatus.Road_type == Forkin)
      &&(yuansanshu == 1))                                           //��һȦ�Ҳ���
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
      ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder -ImageDeal[Ysite].LeftBorder;  //��ȸ���
    }
  }
  if ((ImageStatus.Road_type == Forkin ) &&
         ( yuansanshu == 0))   //�ڶ�Ȧ����
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
      ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder -ImageDeal[Ysite].LeftBorder;  //��ȸ���
    }
  }
}

////---------------------------------------------------------------------------------------------------------------------------------------------------------------
////  @name           barn_in_test
////  @brief          Ԫ�ؼ��
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
////  @brief          Ԫ�ؼ��
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
//  @brief          Ԫ�ؼ��
//  @parameter      void
//  Sample usage:   element_detection();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void element_detection(void)    //Ԫ�ؼ��
{
//    barrier_detection();    //�ϰ����
//    roadblock_test();
    zhijiao_test();
    Element_Judgment_Zebra();
    ForkTest();             //����·���
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           RouteFilter
//  @brief
//  @parameter      void
//  Sample usage:   RouteFilter();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void RouteFilter(void) {
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5);
       Ysite--)                                     //�ӿ�ʼλ��ֹͣλ
  {
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W'
         &&Ysite <= 45
         &&ImageDeal[Ysite - 1].IsLeftFind == 'W'
         &&ImageDeal[Ysite - 1].IsRightFind =='W')  //��ǰ�����Ҷ��ޱߣ�������ǰ45��   �˲�
    {
      ytemp = Ysite;
      while (ytemp >= (ImageStatus.OFFLine +5))     // �ĸ����ԣ�-6Ч����һЩ
      {
        ytemp--;
        if (  ImageDeal[ytemp].IsLeftFind == 'T'
            &&ImageDeal[ytemp].IsRightFind == 'T')  //Ѱ�����߶������ģ��ҵ��뱾������ľͲ�����
        {
          DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /(float)(ytemp - 1 - Ysite - 2);          //��б��
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) {
            ImageDeal[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp));                                     //��б�ʲ�
            Ysite--;
          }
          break;
        }
      }
    }
    ImageDeal[Ysite].Center =(ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) /3;                                  //��ƽ����Ӧ�û�Ƚϻ�  ��������������ƽ��
  }
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           image_process
//  @brief          ͼ����
//  @parameter      void
//  Sample usage:   image_process();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void image_process(void)
{
    compressimage();
    Get01change();
    DrawLinesFirst();
    DrawLinesProcess();

    /*Ԫ��ʶ���봦��*/
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
        DrawExtensionLine();        //�������˾Ϳ����������������
    }
    RouteFilter();              //�����˲�
}
