/*
 * image.h
 *
 *  Created on: 2024年5月26日
 *      Author: point
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#define LCDH 60                              //用于图像处理图像的高度
#define LCDW 80                              //用于图像处理图像的宽度
#define uLCDH 120                            //用于图像显示的图像高度
#define uLCDW 160                            //用于图像显示的图像宽度
#define LimitL(L) (L = ((L < 1) ? 1 : L))    //限制幅度
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //限制幅度
#define ImageSensorMid 39                    //图像屏幕中点
#define WHITE 255
#define BLACK 0
#define barrier_line    26
#define bridge_line     28

extern int ImageScanInterval;  //扫边范围    上一行的边界+-ImageScanInterval
extern int ImageScanInterval_Cross;  //十字扫边范围

typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;

typedef struct {
  /*左右边边界标志    T为正常跳变边    W为无边   P为障碍类多跳边的内边*/
  uint8 IsRightFind;      //右边有边标志
  uint8 IsLeftFind;       //左边有边标志
  int Wide;               //边界宽度
  int LeftBorder;         //左边界
  int RightBorder;        //右边界
  int Center;             //中线

  // fork   下面的变量用于岔路口的检测
  uint8 isBlackFind;      //三叉边
    int BlackWide;     //三叉口黑色宽度
    int Black_Wide_L;  //三叉口黑色左边
    int Black_Wide_R;  //三叉口黑色右边
    int Black_Pro;     //三叉口黑点比例
    int mid_temp;      //中线临时值
} ImageDealDatatypedef;

//元素类型
typedef enum {
  Normol,       //无任何特征
  Straight,     ////直道
  Cross,        ////十字
  Forkin,       //岔路进口
  Forkout,      //岔路出口
  Barn_out,     //出库
  Barn_in,      //入库
  Block,
} RoadType_e;


typedef enum
{
    FlASH_192_SRAM_128 = 0,
    FLASH_224_SRAM_96,
    FLASH_256_SRAM_64,
    FLASH_288_RAM_32
} FLASH_SRAM_DEFIN;

typedef struct {
  /*以下关于全局图像正常参数*/

  //图像信息
  uint8 OFFLine;            /////图像顶边
  uint8 WhiteLine;          ////双边丢边数
  uint8_t RWLine;           //右丢线数量
  uint8_t LWLine;           //左丢线数量

  RoadType_e Road_type;  //元素类型

} ImageStatustypedef;

typedef struct
{
  uint8_t right_barrier;
  uint8 left_barrier;
  uint8_t barrier;
  uint8_t flag;
  uint8_t right_bridge;
  uint8_t left_bridge;
  uint8_t lbridge;
  uint8_t rbridge;
  uint8_t bridgeborder;
} Barriertypedef;       //障碍

typedef struct
{
  uint8_t zebra_flag;
  uint8_t zebra_flag2;
  uint8_t fork_flag;
  uint8_t barn_in_flag;
  uint8_t barn_in_flag2;
  uint8_t barn_out_flag;
  uint8_t barn_out_flag2;
  uint8_t bridge_flag;
  uint8_t bridge_flag2;
  uint8_t bridge_flag3;
  uint8_t Lroad_flag;
  uint8_t Lroad_flag2;
  uint8_t block_flag;

} Flagtypedef;       //标志位结构体

uint8 Threshold_deal(uint8* image,
                     uint16 col,
                     uint16 row,
                     uint32 pixel_threshold);

extern uint8 Image_Use[LCDH][LCDW];
extern Barriertypedef RoadBarrier;
extern Flagtypedef RoadFlag;
extern ImageStatustypedef ImageStatus;
ImageDealDatatypedef ImageDeal[60];


float Straight_Judge(uint8 dir, uint8 start, uint8 end);
uint8_t my_adapt_threshold(uint8 *image, uint16 col, uint16 row);
void barn_in_test(void);
void bran_out_test(void);
void bridge_test(void);
void Element_Judgment_Zebra();
void barrier_detection(void);
void roadblock_test(void);
void element_detection(void);
void ForkTest(void);
void Fork_Handle(void);
void image_process(void);
void RouteFilter(void);


#endif /* IMAGE_H_ */
