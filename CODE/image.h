/*
 * image.h
 *
 *  Created on: 2024��5��26��
 *      Author: point
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#define LCDH 60                              //����ͼ����ͼ��ĸ߶�
#define LCDW 80                              //����ͼ����ͼ��Ŀ��
#define uLCDH 120                            //����ͼ����ʾ��ͼ��߶�
#define uLCDW 160                            //����ͼ����ʾ��ͼ����
#define LimitL(L) (L = ((L < 1) ? 1 : L))    //���Ʒ���
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //���Ʒ���
#define ImageSensorMid 39                    //ͼ����Ļ�е�
#define WHITE 255
#define BLACK 0
#define barrier_line    26
#define bridge_line     28

extern int ImageScanInterval;  //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval
extern int ImageScanInterval_Cross;  //ʮ��ɨ�߷�Χ

typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;

typedef struct {
  /*���ұ߽߱��־    TΪ���������    WΪ�ޱ�   PΪ�ϰ�������ߵ��ڱ�*/
  uint8 IsRightFind;      //�ұ��б߱�־
  uint8 IsLeftFind;       //����б߱�־
  int Wide;               //�߽���
  int LeftBorder;         //��߽�
  int RightBorder;        //�ұ߽�
  int Center;             //����

  // fork   ����ı������ڲ�·�ڵļ��
  uint8 isBlackFind;      //�����
    int BlackWide;     //����ں�ɫ���
    int Black_Wide_L;  //����ں�ɫ���
    int Black_Wide_R;  //����ں�ɫ�ұ�
    int Black_Pro;     //����ںڵ����
    int mid_temp;      //������ʱֵ
} ImageDealDatatypedef;

//Ԫ������
typedef enum {
  Normol,       //���κ�����
  Straight,     ////ֱ��
  Cross,        ////ʮ��
  Forkin,       //��·����
  Forkout,      //��·����
  Barn_out,     //����
  Barn_in,      //���
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
  /*���¹���ȫ��ͼ����������*/

  //ͼ����Ϣ
  uint8 OFFLine;            /////ͼ�񶥱�
  uint8 WhiteLine;          ////˫�߶�����
  uint8_t RWLine;           //�Ҷ�������
  uint8_t LWLine;           //��������

  RoadType_e Road_type;  //Ԫ������

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
} Barriertypedef;       //�ϰ�

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

} Flagtypedef;       //��־λ�ṹ��

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
