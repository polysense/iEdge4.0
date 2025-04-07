/**
 * @file  mtepd.c
 * @brief MT EPD液晶面板帧缓冲显示驱动
 * @author Runby F.
 * @date 2022-4-1
 * @copyright Polysense
 */

#include "drv_api.h"
#include "drv_mtepd.h"

#define MTEPD_FBUF_POS_ID(x,y)          ((x)/8+(y)*(MB_EPD_WIDTH/8))

typedef struct {
  char name[4];
  char unit[5];
  pos_u16_t precision;
  pos_u16_t signed_flag;
} vs_type_t;

/**
 * @brief 16x16点阵字库
 * 制曲车间班组酵堆窖池粮酒请刷卡成功未扫码上中下注
 */
const static pos_u8_t _fonts_hz16x16[]= {
  /*--  文字:  制  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x04,0x04,0x24,0x04,0x24,0x04,0x3F,0xA4,0x44,0x24,0x04,0x24,0xFF,0xE4,0x04,0x24,
  0x04,0x24,0x3F,0xA4,0x24,0xA4,0x24,0xA4,0x26,0x84,0x25,0x04,0x04,0x14,0x04,0x08,
  
  /*--  文字:  曲  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x04,0x40,0x04,0x40,0x04,0x40,0x04,0x40,0x7F,0xFC,0x44,0x44,0x44,0x44,0x44,0x44,
  0x44,0x44,0x7F,0xFC,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x7F,0xFC,0x40,0x04,
  
  /*--  文字:  车  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x02,0x00,0x02,0x00,0x02,0x00,0x7F,0xFC,0x04,0x00,0x09,0x00,0x11,0x00,0x21,0x00,
  0x3F,0xF8,0x01,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,
  
  /*--  文字:  间  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x20,0x00,0x13,0xFC,0x10,0x04,0x40,0x04,0x47,0xC4,0x44,0x44,0x44,0x44,0x44,0x44,
  0x47,0xC4,0x44,0x44,0x44,0x44,0x44,0x44,0x47,0xC4,0x40,0x04,0x40,0x14,0x40,0x08,
  
  /*--  文字:  班  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x80,0x00,0x80,0xF8,0xBE,0x22,0x88,0x22,0x88,0x22,0x88,0x22,0x88,0xFA,0xBE,
  0x22,0x88,0x24,0x88,0x20,0x88,0x39,0x08,0xE1,0x08,0x42,0x3E,0x04,0x00,0x08,0x00,
  
  /*--  文字:  组  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x10,0x00,0x11,0xF8,0x21,0x08,0x21,0x08,0x49,0x08,0xF9,0xF8,0x11,0x08,0x21,0x08,
  0x41,0x08,0xF9,0xF8,0x41,0x08,0x01,0x08,0x19,0x08,0xE1,0x08,0x47,0xFE,0x00,0x00,
  
  /*--  文字:  酵  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x20,0xFE,0x22,0x29,0xFA,0x28,0x24,0xFE,0x24,0xAB,0xFE,0xAA,0x10,0xAA,0x20,
  0xAF,0xFC,0xC2,0x88,0x83,0x10,0xFE,0xFE,0x82,0x10,0x82,0x10,0xFE,0x50,0x82,0x20,
  
  /*--  文字:  堆  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x21,0x40,0x21,0x20,0x21,0x20,0x23,0xFE,0x22,0x20,0xFE,0x20,0x2B,0xFC,0x22,0x20,
  0x22,0x20,0x23,0xFC,0x22,0x20,0x3A,0x20,0xE2,0x20,0x43,0xFE,0x02,0x00,0x02,0x00,
  
  /*--  文字:  窖  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x02,0x00,0x01,0x00,0x7F,0xFE,0x48,0x22,0x91,0x14,0x09,0x00,0x0F,0xF0,0x11,0x00,
  0x01,0x00,0xFF,0xFE,0x00,0x00,0x1F,0xF0,0x10,0x10,0x10,0x10,0x1F,0xF0,0x10,0x10,
  
  /*--  文字:  池  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x40,0x20,0x40,0x12,0x40,0x12,0x58,0x82,0x68,0x42,0xC8,0x53,0x48,0x1E,0x48,
  0x12,0x48,0x22,0x48,0xE2,0x58,0x22,0x42,0x22,0x02,0x22,0x02,0x21,0xFE,0x00,0x00,
  
  /*--  文字:  粮  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x10,0x40,0x10,0x20,0x95,0xFC,0x55,0x04,0x59,0x04,0x11,0xFC,0xFD,0x04,0x31,0x04,
  0x39,0xFC,0x55,0x20,0x55,0x22,0x91,0x14,0x11,0x08,0x11,0x44,0x11,0x82,0x11,0x00,
  
  /*--  文字:  酒  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x00,0x4F,0xFE,0x20,0xA0,0x20,0xA0,0x07,0xFC,0x84,0xA4,0x44,0xA4,0x54,0xA4,
  0x15,0x1C,0x26,0x04,0xE4,0x04,0x27,0xFC,0x24,0x04,0x24,0x04,0x27,0xFC,0x04,0x04,
  
  /*--  文字:  请  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x40,0x40,0x40,0x27,0xFC,0x20,0x40,0x03,0xF8,0x00,0x40,0xE7,0xFE,0x20,0x00,
  0x23,0xF8,0x22,0x08,0x23,0xF8,0x22,0x08,0x2B,0xF8,0x32,0x08,0x22,0x28,0x02,0x10,
  
  /*--  文字:  刷  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x02,0x3F,0xC2,0x20,0x42,0x20,0x42,0x3F,0xD2,0x22,0x12,0x22,0x12,0x22,0x12,
  0x3F,0xD2,0x52,0x52,0x52,0x52,0x52,0x52,0x93,0x42,0x12,0x82,0x02,0x0A,0x02,0x04,
  
  /*--  文字:  卡  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x02,0x00,0x02,0x00,0x03,0xF8,0x02,0x00,0x02,0x00,0x02,0x00,0xFF,0xFE,0x02,0x00,
  0x02,0x00,0x02,0x40,0x02,0x20,0x02,0x10,0x02,0x08,0x02,0x00,0x02,0x00,0x02,0x00,
  
  /*--  文字:  成  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x50,0x00,0x48,0x00,0x40,0x3F,0xFE,0x20,0x40,0x20,0x40,0x20,0x44,0x3E,0x44,
  0x22,0x44,0x22,0x28,0x22,0x28,0x22,0x12,0x2A,0x32,0x44,0x4A,0x40,0x86,0x81,0x02,
  
  /*--  文字:  功  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x40,0x00,0x40,0x00,0x40,0xFE,0x40,0x11,0xFC,0x10,0x44,0x10,0x44,0x10,0x44,
  0x10,0x44,0x10,0x84,0x10,0x84,0x1E,0x84,0xF1,0x04,0x41,0x04,0x02,0x28,0x04,0x10,
  
  /*--  文字:  未  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x01,0x00,0x01,0x00,0x01,0x00,0x3F,0xF8,0x01,0x00,0x01,0x00,0x01,0x00,0xFF,0xFE,
  0x03,0x80,0x05,0x40,0x09,0x20,0x11,0x10,0x21,0x08,0xC1,0x06,0x01,0x00,0x01,0x00,
  
  /*--  文字:  扫  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x10,0x00,0x10,0x00,0x13,0xFC,0x10,0x04,0xFC,0x04,0x10,0x04,0x10,0x04,0x15,0xFC,
  0x18,0x04,0x30,0x04,0xD0,0x04,0x10,0x04,0x10,0x04,0x13,0xFC,0x50,0x04,0x20,0x00,
  
  /*--  文字:  码  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x00,0x01,0xF8,0xFC,0x08,0x10,0x08,0x10,0x88,0x20,0x88,0x3C,0x88,0x64,0xFE,
  0x64,0x02,0xA4,0x02,0x24,0x02,0x25,0xFA,0x3C,0x02,0x24,0x02,0x20,0x14,0x00,0x08,

  /*--  文字:  上  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x03,0xF8,0x02,0x00,
  0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0xFF,0xFE,0x00,0x00,

  /*--  文字:  中  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x3F,0xF8,0x21,0x08,0x21,0x08,0x21,0x08,
  0x21,0x08,0x21,0x08,0x3F,0xF8,0x21,0x08,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,

  /*--  文字:  下  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x00,0xFF,0xFE,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x40,0x02,0x20,
  0x02,0x10,0x02,0x08,0x02,0x08,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,

  /*--  文字:  注  --*/
  /*--  宋体12;  此字体下对应的点阵为：宽x高=16x16   --*/
  0x00,0x80,0x20,0x40,0x10,0x00,0x17,0xFC,0x80,0x40,0x40,0x40,0x40,0x40,0x10,0x40,
  0x13,0xFC,0x20,0x40,0xE0,0x40,0x20,0x40,0x20,0x40,0x20,0x40,0x2F,0xFE,0x00,0x00,  
};


/**
 * @brief 8x16 ASCII点阵字库
 *  !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
 */
const static pos_u8_t _fonts8x16[] = {
  /*--  文字:     --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  !  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x18,0x3C,0x3C,0x3C,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00,
  
  /*--  文字:  "  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x66,0x66,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  #  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x36,0x36,0x7F,0x36,0x36,0x36,0x7F,0x36,0x36,0x00,0x00,0x00,0x00,
  
  /*--  文字:  $  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x18,0x18,0x3C,0x66,0x60,0x30,0x18,0x0C,0x06,0x66,0x3C,0x18,0x18,0x00,0x00,
  
  /*--  文字:  %  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x22,0x52,0x54,0x54,0x54,0x2a,0x0d,0x15,0x15,0x15,0x22,0x00,0x00,
  
  /*--  文字:  &  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x38,0x6C,0x6C,0x38,0x60,0x6F,0x66,0x66,0x3B,0x00,0x00,0x00,0x00,
  
  /*--  文字:  '  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  (  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x0C,0x18,0x18,0x30,0x30,0x30,0x30,0x30,0x18,0x18,0x0C,0x00,0x00,
  
  /*--  文字:  )  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x30,0x18,0x18,0x0C,0x0C,0x0C,0x0C,0x0C,0x18,0x18,0x30,0x00,0x00,
  
  /*--  文字:  *  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x36,0x1C,0x7F,0x1C,0x36,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  +  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  ,  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x0C,0x18,0x00,0x00,
  
  /*--  文字:  -  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  .  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  /  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x06,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30,0x60,0x60,0x00,0x00,0x00,
  
  /*--  文字:  0  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x1E,0x33,0x37,0x37,0x33,0x3B,0x3B,0x33,0x1E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  1  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x0C,0x1C,0x7C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  2  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  3  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x06,0x1C,0x06,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  4  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x30,0x30,0x36,0x36,0x36,0x66,0x7F,0x06,0x06,0x00,0x00,0x00,0x00,
  
  /*--  文字:  5  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7E,0x60,0x60,0x60,0x7C,0x06,0x06,0x0C,0x78,0x00,0x00,0x00,0x00,
  
  /*--  文字:  6  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x1C,0x18,0x30,0x7C,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  7  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7E,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30,0x30,0x00,0x00,0x00,0x00,
  
  /*--  文字:  8  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x76,0x3C,0x6E,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  9  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x3E,0x0C,0x18,0x38,0x00,0x00,0x00,0x00,
  
  /*--  文字:  :  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  ;  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x1C,0x1C,0x0C,0x18,0x00,0x00,
  
  /*--  文字:  <  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00,0x00,0x00,
  
  /*--  文字:  =  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  >  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x60,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00,
  
  /*--  文字:  ?  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x0C,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00,
  
  /*--  文字:  @  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7E,0xC3,0xC3,0xCF,0xDB,0xDB,0xCF,0xC0,0x7F,0x00,0x00,0x00,0x00,
  
  /*--  文字:  A  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x18,0x3C,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  B  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  C  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x60,0x60,0x60,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  D  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x78,0x6C,0x66,0x66,0x66,0x66,0x66,0x6C,0x78,0x00,0x00,0x00,0x00,
  
  /*--  文字:  E  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7E,0x60,0x60,0x60,0x7C,0x60,0x60,0x60,0x7E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  F  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7E,0x60,0x60,0x60,0x7C,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00,
  
  /*--  文字:  G  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x60,0x60,0x6E,0x66,0x66,0x3E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  H  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  I  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  J  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x06,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  K  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x66,0x66,0x6C,0x6C,0x78,0x6C,0x6C,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  L  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  M  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x63,0x63,0x77,0x6B,0x6B,0x6B,0x63,0x63,0x63,0x00,0x00,0x00,0x00,
  
  /*--  文字:  N  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x63,0x63,0x73,0x7B,0x6F,0x67,0x63,0x63,0x63,0x00,0x00,0x00,0x00,
  
  /*--  文字:  O  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  P  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00,
  
  /*--  文字:  Q  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x0C,0x06,0x00,0x00,
  
  /*--  文字:  R  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x6C,0x66,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  S  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x66,0x60,0x30,0x18,0x0C,0x06,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  T  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00,
  
  /*--  文字:  U  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  V  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00,0x00,0x00,0x00,
  
  /*--  文字:  W  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x63,0x63,0x63,0x6B,0x6B,0x6B,0x36,0x36,0x36,0x00,0x00,0x00,0x00,
  
  /*--  文字:  X  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x66,0x66,0x34,0x18,0x18,0x2C,0x66,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  Y  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00,
  
  /*--  文字:  Z  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x7E,0x06,0x06,0x0C,0x18,0x30,0x60,0x60,0x7E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  [  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,
  
  /*--  文字:  \  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x60,0x60,0x30,0x30,0x18,0x18,0x0C,0x0C,0x06,0x06,0x00,0x00,0x00,
  
  /*--  文字:  ]  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,
  
  /*--  文字:  ^  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x18,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  /*--  文字:  _  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,

#if 0  
  /*--  文字:  `  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x38,0x18,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
#else
  /*--  文字:  ` 替换为上。用于摄氏度显示比如`C将显示为。C摄氏度 --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x06,0x09,0x09,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
#endif

  /*--  文字:  a  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x3C,0x06,0x06,0x3E,0x66,0x66,0x3E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  b  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  c  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  d  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  e  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x66,0x7E,0x60,0x60,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  f  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x1E,0x30,0x30,0x30,0x7E,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00,
  
  /*--  文字:  g  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x7C,0x00,
  
  /*--  文字:  h  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  i  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x18,0x18,0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  j  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x0C,0x0C,0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x78,0x00,
  
  /*--  文字:  k  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x60,0x60,0x66,0x66,0x6C,0x78,0x6C,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  l  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  m  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x7E,0x6B,0x6B,0x6B,0x6B,0x6B,0x63,0x00,0x00,0x00,0x00,
  
  /*--  文字:  n  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  o  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  p  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x00,
  
  /*--  文字:  q  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x06,0x00,
  
  /*--  文字:  r  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x66,0x6E,0x70,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00,
  
  /*--  文字:  s  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x3E,0x60,0x60,0x3C,0x06,0x06,0x7C,0x00,0x00,0x00,0x00,
  
  /*--  文字:  t  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x30,0x30,0x7E,0x30,0x30,0x30,0x30,0x30,0x1E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  u  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  v  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00,0x00,0x00,0x00,
  
  /*--  文字:  w  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x63,0x6B,0x6B,0x6B,0x6B,0x36,0x36,0x00,0x00,0x00,0x00,
  
  /*--  文字:  x  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00,0x00,0x00,0x00,
  
  /*--  文字:  y  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x0C,0x18,0xF0,0x00,
  
  /*--  文字:  z  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x00,0x00,0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00,0x00,
  
  /*--  文字:  {  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x0C,0x18,0x18,0x18,0x30,0x60,0x30,0x18,0x18,0x18,0x0C,0x00,0x00,
  
  /*--  文字:  |  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,
  
  /*--  文字:  }  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x30,0x18,0x18,0x18,0x0C,0x06,0x0C,0x18,0x18,0x18,0x30,0x00,0x00,
  
  /*--  文字:  ~  --*/
  /*--  Arial Narrow18;  此字体下对应的点阵为：宽x高=8x16   --*/
  0x00,0x00,0x00,0x71,0xDB,0x8E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

  
};

/**
 * @brief 清除帧缓存并重置首字符位置
 */
void mtepd_clrscr(mtepd_param_t *p) {
  /* init all zero by default */
  p->drv.os->memset(p->fbuf, 0, MTEPD_FBUF_SIZE);
}

/**
 * @brief 把当前全局帧缓存输出到EPD显示 
 */
void mtepd_show(mtepd_param_t *p) {
  mtepd_drv_on(p);
  mtepd_drv_display(p, p->fbuf);
  mtepd_drv_off(p);  
}

void mtepd_fbuf_putpixel(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, pos_u8_t value, pos_u8_t set) {
  pos_u16_t i, b;
  i = MTEPD_FBUF_POS_ID(x,y);
  b = 1 << (7- (x&7 ));
  if( value ) {
    value = b;
    p->fbuf[i] |= value;
//    p->drv.os->printf("#%u.b%x %u x %u\n", i, (7-(x&7)), x, y);
  } else {
    /* mask off when value == 0 and SET is required */
    if( set )
      p->fbuf[i] &= ~b;
  }
}

pos_u8_t mted_fbuf_getpixel(const pos_u8_t *bmp, pos_u16_t x, pos_u16_t y, pos_u16_t width, pos_u16_t height, pos_u8_t reverse) {
  pos_u8_t v;
  if( y >= height )
    y = height - 1;
  if( reverse )
    y = height-1-y;
  v = bmp[x/8+y*(width/8)];
  v = (v >> (7-(x&7))) & 0x1;
  return v;
}

void mtepd_fbuf_putbmp_xyr_align8(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, const pos_u8_t *bmp, pos_u16_t width, pos_u16_t height, pos_u8_t reverse, pos_u8_t set) {
  pos_u16_t i, b, j, w;
//  pos_u32_t v;
  w = (width+7)/8;
  if( reverse )
    i = MTEPD_FBUF_POS_ID(x,y+height-1);
  else
    i = MTEPD_FBUF_POS_ID(x,y);
  for( j = 0; j < height; j++ ) {
    for( b = 0; b < w; b++) {
      if( set )
        p->fbuf[i+b] = *bmp++;
      else
        p->fbuf[i+b] |= *bmp++;
    }
    if( reverse )
      i -= (MB_EPD_WIDTH/8);  
    else
      i += (MB_EPD_WIDTH/8);  
  }
}

void mtepd_fbuf_putbmp_xyr(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, const pos_u8_t *bmp, pos_u16_t width, pos_u16_t height, pos_u8_t reverse, pos_u8_t set) {
  pos_u16_t w, h;
  /* fast copy when aligned */
  if( (x & 7) == 0 ) {
    mtepd_fbuf_putbmp_xyr_align8(p, x, y, bmp, width, height, reverse, set);
    return;
  }

#if 1
  /* slow p2p copy when !aligned */
  for( h = 0; h < height; h++ ) {
  for( w = 0; w < width; w++ ) {
    mtepd_fbuf_putpixel(p, x+w, y+h, mted_fbuf_getpixel(bmp, w, h, width, height, reverse), set);
  }
  }
#else
{
  pos_u16_t i;
  if( reverse )
    i = MTEPD_FBUF_POS_ID(x,y+height-1);
  else
    i = MTEPD_FBUF_POS_ID(x,y);
  for( h = 0; h < height; h++ ) {
    const pos_u8_t ofs_bits_clr[8] = {0, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe,};
    pos_u8_t v, pb, ob;
    pos_u16_t j;
    ob = x & 7;
    pb = (8-ob); /* first byte valid bits to be SET or OR */
    v = bmp[0] >> ob; /* only keep the 8-v valid btis */
    if( set )
      p->fbuf[i] = (p->fbuf[i] & ofs_bits_clr[pb]) + v;
    else
      p->fbuf[i] |= v;
    j = 1;
    for( w = pb; w + 8 <= width; w+=8 ) {
      v = (*bmp++) << pb;
      v += *bmp >> ob;
      if( set )
        p->fbuf[i+j] = v;
      else
        p->fbuf[i+j] |= v;
      j++;
    }
    /* copy last byte */
    if( w < width ) {
      v = (*bmp++) << pb;
      if( set )
        p->fbuf[i+j] = v;
      else
        p->fbuf[i+j] |= v;
    }
    if( reverse )
      i -= (MB_EPD_WIDTH/8);
    else
      i += (MB_EPD_WIDTH/8);
  }
}
#endif
}

void mtepd_fbuf_putbmp_xy(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, const pos_u8_t *bmp, pos_u16_t width, pos_u16_t height) {
  mtepd_fbuf_putbmp_xyr(p,x,y,bmp,width,height,0,0);
}

void mtepd_fbuf_putchar8x16_xy(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, pos_u8_t ch, pos_u8_t heavy) {
  pos_u16_t id;
//  pos_u32_t v;
  if( ch < 32 || ch > 126 )
    id = 0;
  else
    id = ch - 32;

  id = id * (1 * 16);

  mtepd_fbuf_putbmp_xy(p, x, y, &_fonts8x16[id], 8, 16);
  if( heavy || ch == '%') {
//    mtepd_fbuf_putbmp_xy(p, x+1, y+1, &_fonts8x16[id], 8, 16);
    mtepd_fbuf_putbmp_xy(p, x, y+1, &_fonts8x16[id], 8, 16);
//    mtepd_fbuf_putbmp_xy(p, x+1, y, &_fonts8x16[id], 8, 16);
  }
}

/**
 * @brief 在指定位置打印16点阵汉字 (只支持16点阵)
 */
void mtepd_fbuf_puthz16x16_xy(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, MHZ_t hz, pos_u8_t heavy) {
  pos_u16_t id;
//  pos_u32_t v;
  if( hz <= MHZ_NULL || hz >= MHZ_MAX )
    return;

  /* remap to hz16 id */

  /* out of screen protection */  
  if( x >= MB_EPD_WIDTH-16  )
    return;
  if( y >= MB_EPD_HEIGHT-16  )
    return;

  id = (hz-1) * 32;
  mtepd_fbuf_putbmp_xy(p, x, y, &_fonts_hz16x16[id], 16, 16);
  if( heavy ) {
    mtepd_fbuf_putbmp_xy(p, x+1, y+1, &_fonts_hz16x16[id], 16, 16); 
    mtepd_fbuf_putbmp_xy(p, x, y+1, &_fonts_hz16x16[id], 16, 16);   
    mtepd_fbuf_putbmp_xy(p, x+1, y, &_fonts_hz16x16[id], 16, 16);     
  }
}

/**
 * @brief 在指定位置打印16点阵汉字 or CHAR (只支持16点阵)
 */
void mtepd_fbuf_puts_mix16_xy(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, char *s) {
  char c;
  while(*s) {
    c = *s++;
    if( c < MHZ_MAX ) {
      mtepd_fbuf_puthz16x16_xy(p, x, y, c, 1); /* use HEAVY for HZ */
      x += 16;
    } else {
      mtepd_fbuf_putchar8x16_xy(p, x, y, c, 0);
      x += 8;
    }
  }
}

/**
 * @brief 画线段
 */
void mtepd_fbuf_line_x(mtepd_param_t *p, pos_u16_t x1, pos_u16_t x2, pos_u16_t y) {
  pos_u16_t i, x;
  for( x = x1; x <= x2;  ) {
    i = MTEPD_FBUF_POS_ID(x, y);
    if( (x&7) == 0 && x + 7 <= x2 ) {
      p->fbuf[i] = 0xff;
      x += 8;
      continue;
    }
    p->fbuf[i] |= (1<<(7-(x&7)));
    x++;
  }
}

void mtepd_fbuf_line_y(mtepd_param_t *p, pos_u16_t x, pos_u16_t y1, pos_u16_t y2) {
  pos_u16_t i, y;
  for( y = y1; y <= y2; y++ ) {
    i = MTEPD_FBUF_POS_ID(x, y);
    p->fbuf[i] |= (1<<(7-(x&7)));
  }
}

/**
 * @brief 画矩形
 */
void mtepd_fbuf_rectangle(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, pos_u16_t x_end, pos_u16_t y_end) {
  mtepd_fbuf_line_x(p, x, x_end, y);
  mtepd_fbuf_line_x(p, x, x_end, y_end);  
  mtepd_fbuf_line_y(p, x, y, y_end);
  mtepd_fbuf_line_y(p, x_end, y, y_end);
}

/**
 * @brief Solid rectangle
 */
void mtepd_fbuf_rectangle_solid(mtepd_param_t *p, pos_u16_t x1, pos_u16_t y1, pos_u16_t x2, pos_u16_t y2) {
  pos_u16_t i, x, y;
  for( y = y1; y <= y2; y++ ) {
    for( x = x1; x <= x2;  ) {
      i = MTEPD_FBUF_POS_ID(x, y);
      if( (x&7) == 0 && x + 7 <= x2 ) {
        p->fbuf[i] = 0xff;
        x += 8;
        continue;
      }
      p->fbuf[i] |= (1<<(7-(x&7)));
      x++;
    }
  }
}

/**
 * @brief Draw circle
 */
void mtepd_fbuf_circle2(mtepd_param_t *p, pos_u16_t xc, pos_u16_t yc, pos_u16_t r, pos_u8_t mode, pos_u8_t thick) {
  pos_u32_t x, y, x2, y2, r2, flag;
  r2 = r*r;
  for( y = 0; y <= r/2; y++ ) {
    y2 = y*y;
    flag = 0;
    for( x = 0; x <= r; x++ ) {
      x2 = x*x;
      if( x2 + y2 >= r2 ) {
        if( flag < thick ) {
          if( mode & 1 )
            mtepd_fbuf_putpixel(p, xc-x, yc-y, 1, 0);
          if( mode & 2 )
            mtepd_fbuf_putpixel(p, xc+x, yc-y, 1, 0);
          if( mode & 4 )
            mtepd_fbuf_putpixel(p, xc+x, yc+y, 1, 0);
          if( mode & 8 )
            mtepd_fbuf_putpixel(p, xc-x, yc+y, 1, 0);
          flag++;
        }
      } else
        flag = 0;
    }
  }
}
void mtepd_fbuf_circle(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, pos_u16_t r, pos_u8_t mode) {
  pos_i16_t a = 0, b = r;
  pos_i16_t d = 3 - 2*r;
  if( mode == 0 )
    mode = 0xf;
  while( a <= b ) {
    if( mode & 1 ) {
      mtepd_fbuf_putpixel(p, x-b, y-a, 1, 0);
      mtepd_fbuf_putpixel(p, x-a, y-b, 1, 0);
    }
    if( mode & 2 ) {
      mtepd_fbuf_putpixel(p, x+b, y-a, 1, 0);
      mtepd_fbuf_putpixel(p, x+a, y-b, 1, 0);
    }
    if( mode & 4 ) {
      mtepd_fbuf_putpixel(p, x+b, y+a, 1, 0);
      mtepd_fbuf_putpixel(p, x+a, y+b, 1, 0);
    }
    if( mode & 8 ) {
      mtepd_fbuf_putpixel(p, x-b, y+a, 1, 0);  
      mtepd_fbuf_putpixel(p, x-a, y+b, 1, 0);
    }
    if(d < 0 )
      d += 4 * a + 6;
    else {
      d += 10 + 4 * (a-b);
      b--;
    }
    a++;
//    mtepd_fbuf_putpixel(p, x+a, y+b);
  }
}

/**
 * @brief 画电池百分比电量
 */
void mtepd_battery(mtepd_param_t *p, pos_u16_t x, pos_u16_t y, pos_u8_t percentage) {
#define MTEPD_FBUG_BAT_WIDTH 28
#define MTEPD_FBUG_BAT_HEIGHT 8
#define MTEPD_FBUF_BAT_LEVEL_D 2
#define MTEPD_FBUG_BAT_LEVEL_HEIGHT (MTEPD_FBUG_BAT_HEIGHT-MTEPD_FBUF_BAT_LEVEL_D)
  
  mtepd_fbuf_rectangle(p, x, y, x+MTEPD_FBUG_BAT_WIDTH, y + MTEPD_FBUG_BAT_HEIGHT);
  mtepd_fbuf_rectangle(p, x+MTEPD_FBUG_BAT_WIDTH, y + 2, x+MTEPD_FBUG_BAT_WIDTH+4, y + 6);  
  if( percentage > 4 )
    mtepd_fbuf_rectangle_solid(p, x + MTEPD_FBUF_BAT_LEVEL_D, y + MTEPD_FBUF_BAT_LEVEL_D, 
      x + MTEPD_FBUF_BAT_LEVEL_D+ percentage/4, y + MTEPD_FBUG_BAT_LEVEL_HEIGHT );  
}

/**
 * @brief 分配并初始化帧缓存
 */
pos_status_t mtepd_init(mtepd_param_t *p) {
  p->fbuf = p->drv.os->malloc(MTEPD_FBUF_SIZE);
  if( !p->fbuf )
    return POS_STATUS_E_MEM;

  mtepd_clrscr(p);
  
  return POS_STATUS_OK;
}

/**
 * @brief 释放帧缓存
 */
void mtepd_done(mtepd_param_t *p) {
  if( p->fbuf )
    p->drv.os->free(p->fbuf);
  p->fbuf = POS_NULL;
}

/**
 * @brief RFID中提取信息
 * @param[in] rfid RFID数值
 * @param[in] item 提取数值(0:车间, 1:班组, 2:操作台位置, 3:车间类型, 4:传感器类型)
 * @return 提取数值
 */
pos_u32_t mtepd_rfid_to_workshop(pos_u32_t rfid, pos_u32_t item) {
  pos_u32_t mod, div;
  switch(item) {
  case 0: /* 车间 */
    mod = 1000000000;
    div = 10000000;
    break;
    
  case 1: /* 班组 */
    mod = 10000000;
    div = 100000;
    break;
    
  case 2: /* 操作台位置 */
    mod = 1000;
    div = 1;
    break;  
    
  case 3: /* 车间类型 */
    return rfid / 1000000000;
    break;        
    
  default: /* 传感器类型 */
    mod = 100000;
    div = 1000;
    break;
  }

  return ( rfid % mod ) / div;
}

static const vs_type_t vs[] = {
  {"O2", "%VOL", 10, 0},
  {"H2S", "PPM", 100, 0},
  {"CO", "PPM", 10, 0},
  {"CH4", "PPM", 1, 0},
  {"CO2", "PPM", 1, 0},
  {"T", "`C", 10, 1},
  {"RH", "%", 10, 0},
};

static const vs_type_t vs_aw = {"AW", " ", 1000, 0};

pos_i32_t mtepd_load_history_data(mtepd_param_t *p, pos_u8_t v_type, pos_u8_t v_id) {
  pos_i32_t v;
  drv_data_history_ext_t *ext;
  ext = p->drv.history->ext;
  if( v_type < 5 )          
    v = ext->u.u16[v_type+v_id*8];
  else {
    if( v_type == 5 )
      v = p->drv.history->temp[v_id]; /* tmp */
    else
      v = p->drv.history->temp[v_id+8]; /* hum */
  }
  if( (vs[v_type].signed_flag & 1) && (v&0x8000)) {
    v |= 0xffff0000;
  }
  return v;
}

/**
 * print data 
 */
void mtepd_print_data_vs(const pos_lib_t *os, char *buf, vs_type_t *p_vs, int32_t r, uint8_t loose) {
  char *_t = buf;
  char _v_buf[16];
  _t = _v_buf;
  if( r < 0 ) {
    *_t++ = '-';
  }
  *_t = 0 ;
  r = os->util->abs(r);
  if( p_vs->precision >= 1000 ) {
    os->sprintf(_t, "%d.%03u", r/p_vs->precision, r%p_vs->precision);
  } else if( p_vs->precision >= 100 ) {
    os->sprintf(_t, "%d.%02u", r/p_vs->precision, r%p_vs->precision);
  } else if( p_vs->precision >= 10 ) {
    os->sprintf(_t, "%d.%u", r/p_vs->precision, r%p_vs->precision);
  } else {
    os->sprintf(_t, "%d", r);
  }
  if( loose )
    os->sprintf(buf, " %3s: %5s  %4s", p_vs->name, _v_buf, p_vs->unit );  
  else
    os->sprintf(buf, "%s:%s%s", p_vs->name, _v_buf, p_vs->unit );
}

/**
 * check empty 
 */
pos_u8_t mtepd_print_data_is_empty(int32_t r, uint8_t vs_id) {
  pos_u8_t empty = 0;
  if( vs_id >5 && r == 0 )
    empty = 1; /* display none for HUM when HUM=0 */
  else if( vs_id==5 && (r <= - 2730 || r >= 9999 ) )
      empty = 1; /* display none for TEMP when TEMP is invalid */
  else if( vs_id < 5 && r == 65535 ) {
    empty = 1;
  }
  return empty;
}

/**
 * @brief EPD屏幕驱动以及字符打印
 * @param[in] p 参数控制块
 * @param[in] disp_mode 显示模式(0:(车间信息或者未扫码提示), 1:成功, 2:请刷卡, 3:系统初始化)
 */
void mtepd_refresh(mtepd_param_t *p, pos_u8_t disp_mode) {
  pos_u8_t i;
  pos_u16_t x, y, v_l, v_h;
  pos_u32_t v, rfid, tick, log;
  char buf[32];
  const pos_lib_t *os;
  drv_data_history_ext_t *ext;
  os = p->drv.os;
  tick = os->tick->get();
  
  /* update drv model/reverse during each refreshing */
  x = p->drv.s->slot->thld.u16[1];
  p->hw_rev = x & 3;  

  log =p->drv.cfg->ctrl & MA_CFG_CTRL_DRV_LOG;

  if( x & 0x100 )
    p->flags |= DRV_MTEPD_FLAG_X_REVERSE;
  else
    p->flags &= ~DRV_MTEPD_FLAG_X_REVERSE;
  if( x & 0x200 )
    p->flags |= DRV_MTEPD_FLAG_Y_REVERSE;
  else
    p->flags &= ~DRV_MTEPD_FLAG_Y_REVERSE;

  /* check no_battery flag and quit */
  x = VBAT_GET(p);
  v_l = p->drv.s->slot->thld.u16[2];
  if( !v_l )
    v_l = 3100;
  v_h = p->drv.s->slot->thld.u16[3];
  if( !v_h )
    v_h = 3200;
  if( x && x < v_h && (p->flags & DRV_MTEPD_FLAG_NO_BATTERY) != 0 ) {
    /* do NOT refresh when NO_BATTERY_FLAG set AND vbat < 3100 */
    if( log ) {
      p->drv.log->data("mtepd low", x); 
    }    
    return;
  }

  /* set NO_BATTERY flag when vbat < v_l */
  if( x && x < v_l )
    p->flags |= DRV_MTEPD_FLAG_NO_BATTERY;
  else if( x >= v_h )
    /* clear NO_BATTERY flag when vbat >= v_h */
    p->flags &= ~DRV_MTEPD_FLAG_NO_BATTERY;


  /* 分配并初始化帧缓存 */
  if( mtepd_init(p) != POS_STATUS_OK ) {
    /* 内存出错直接退出 */
    p->drv.log->data("mtepd fbuf err",0);
    return;
  }

  /* Display NO BATTERY/EUI and quit */
  if( p->flags & DRV_MTEPD_FLAG_NO_BATTERY ) {
    /* display EUI */
    p->drv.os->sprintf(buf, "EUI:%8B", p->drv.cfg->eui);
    mtepd_fbuf_puts_mix16_xy(p, MB_EPD_WIDTH/2-80, MB_EPD_HEIGHT/2-8-16, buf);
    mtepd_fbuf_puts_mix16_xy(p, MB_EPD_WIDTH/2-40, MB_EPD_HEIGHT/2-8, "LOW VOLTAGE");
    
    mtepd_show(p);
    mtepd_done(p);    
    return;
  }

  /* always alarm set off before logic/btn check */
  mtepd_alarm_set(p, 0);

  /* display EUI */
  p->drv.os->sprintf(buf, "EUI:%8B", p->drv.cfg->eui);
  mtepd_fbuf_puts_mix16_xy(p, 8, 3, buf);

  if( DRV_HISOTRY_EXT_IS_READY(&p->drv) )
    ext = p->drv.history->ext;
  else
    ext = POS_NULL;

  if( log ) {
    p->drv.os->flush();
    p->drv.log->data("mtepd disp", (p->hw_rev << 8)+disp_mode);
    if( ext )
      p->drv.log->buf("history", ext->u.u8, ext->u_size);
    p->drv.log->buf("temp", p->drv.history->temp, sizeof(p->drv.history->temp));
    p->drv.os->flush();
  }

  /* history/ext 
   #0 O2
   #1 H2S
   #2 CO
   #3 CH4
   #4 CO2
   #5..#7 rsvd
   */
#if 0
{
  p->drv.history->temp[1] = -381;
  p->drv.history->temp[9] = 131;
  p->drv.history->temp[2] = 1052;
  p->drv.history->temp[10] = 982;
  p->drv.history->temp[3] = -383;
  p->drv.history->temp[11] = 133;
  p->drv.history->temp[4] = 1054;
  p->drv.history->temp[12] = 984;

  ext->u.u16[0] = 210;
  ext->u.u16[1] = 1230;
  ext->u.u16[2] = 150;
  ext->u.u16[3] = 300;
  ext->u.u16[4] = 1200;

  ext->u.u16[0+8] = 211;
  ext->u.u16[1+8] = 1231;
  ext->u.u16[2+8] = 151;
  ext->u.u16[3+8] = 301;
  ext->u.u16[4+8] = 1201;  

  ext->u.u16[0+16] = 212;
  ext->u.u16[1+16] = 1232;
  ext->u.u16[2+16] = 152;
  ext->u.u16[3+16] = 302;
  ext->u.u16[4+16] = 1202;    
}
#endif

  switch(p->drv.s->slot->thld.u16[0] & 0x3) {
  case 0: /* 3 groups */
    if( !ext )
      break;
    x = 16;
    y = 8;
    for( i = 0; i < 3; i++ ) {
      pos_u8_t j, bmp, step;
      pos_i32_t r;
      pos_u8_t empty = 0;
      vs_type_t *p_vs;
      bmp = 0x6f; /* no co2 */
      step = 0;
      for( j = 0; j < POS_ARRAY_CNT(vs); j++  ) {
        if( ((1<<j) & bmp) == 0 )
          continue;
        r = mtepd_load_history_data(p, j, i);
        empty = mtepd_print_data_is_empty(r, j);
        p_vs = (vs_type_t*)&vs[j];
        /* temp is also invalid when hum is invalid */
        if( j == 5 && !empty )
          empty = mtepd_print_data_is_empty(mtepd_load_history_data(p, 6, i), 6);
        if( j >= 6 && (p->drv.s->slot->thld.u16[1] & 0x400) != 0 ) {
          /* swap hr to aw displaying */
          p_vs = (vs_type_t*)&vs_aw;
        }
        if( log ) {
          p->drv.log->data(p_vs->name, r);
        }
        if( empty ) {
          os->sprintf(buf, " %3s:   --   %4s", p_vs->name, p_vs->unit);
        } else {
          mtepd_print_data_vs(os, buf, p_vs, r, 1);
        }
        mtepd_fbuf_puts_mix16_xy( 
          p,
          x + (i&1)*MB_EPD_WIDTH/2, y + ((i>>1)&1)*MB_EPD_HEIGHT/2 + step * 22 + 8,
          buf
        );
        step++;
      }
    }
    mtepd_fbuf_line_x(p, 0, MB_EPD_WIDTH-1, MB_EPD_HEIGHT/2);
    mtepd_fbuf_puthz16x16_xy(p, MB_EPD_WIDTH/2-18, MB_EPD_HEIGHT/2-18, MHZ_SHANG, 1);
    mtepd_fbuf_puthz16x16_xy(p, MB_EPD_WIDTH/2+3, MB_EPD_HEIGHT/2-18, MHZ_ZHONG, 1);
    mtepd_fbuf_puthz16x16_xy(p, MB_EPD_WIDTH/2-18, MB_EPD_HEIGHT/2+2, MHZ_XIA, 1);
    mtepd_fbuf_puthz16x16_xy(p, MB_EPD_WIDTH/2+3, MB_EPD_HEIGHT/2+2, MHZ_ZHU, 1);
    for( i = 0; i < 2; i++ ) {
      mtepd_fbuf_circle(p, MB_EPD_WIDTH/2, MB_EPD_HEIGHT/2, 24+i, 0);
    }
    break;
    
  case 1: /* 5 groups */
  default:
    if( !ext )
      break;
    x = 16;
    y = 32;
    for( i = 0; i < 5; i++ ) {
      pos_u8_t j, bmp, step;
      pos_i32_t r;
      vs_type_t *p_vs;
      pos_u8_t empty = 0;
      /* only T+H */
      p_vs = (vs_type_t*)&vs[6];
      if( (p->drv.s->slot->thld.u16[1] & 0x400) != 0 ) {
        /* swap hr to aw displaying */
        p_vs = (vs_type_t*)&vs_aw;
      }
      
      if( i == 1 || i == 3 ) {
        r = mtepd_load_history_data(p, 5, i); /* load temp */    
        if( log ) {
          p->drv.log->data(vs[5].name, r);
        }

        empty = mtepd_print_data_is_empty(r, 5);
        /* also invalid when hum is invalid */
        if( !empty )
          empty = mtepd_print_data_is_empty(mtepd_load_history_data(p, 6, i), 6);

        /* display -- when invalid */
        if( empty  )
          os->sprintf(buf, "T: -- `C");
        else 
          mtepd_print_data_vs(os, buf, (vs_type_t*)&vs[5], r, 0);

        step = 16;
        if( i == 3 )
          step = 32 + MB_EPD_WIDTH/2;
        mtepd_fbuf_puts_mix16_xy(p, step, MB_EPD_HEIGHT/2-8, buf);
        r = mtepd_load_history_data(p, 6, i); /* load rh */
        if( log ) {
          p->drv.log->data(p_vs->name, r);
        }
       
        /* display -- when invalid */
        if( !r )
          os->sprintf(buf, "%s: -- %s", p_vs->name, p_vs->unit);
        else 
          mtepd_print_data_vs(os, buf, p_vs, r, 0);
        mtepd_fbuf_puts_mix16_xy(p, step + 88, MB_EPD_HEIGHT/2-8, buf);
        continue;
      }
      bmp = 0x71; /* T/h+co2+o2 */
      step = 0;
      for( j = 0; j < POS_ARRAY_CNT(vs); j++  ) {
        if( ((1<<j) & bmp) == 0 )
          continue;
        if( j < 5 ) {
          r = mtepd_load_history_data(p, j, i/2); /* load gas by: #0 blk#0; #2 blk#1,  #4 blk#2 */
        } else {
          r = mtepd_load_history_data(p, j, i); /* load T/H by raw ID */
        }
        empty = mtepd_print_data_is_empty(r, j);
        /* temp is also invalid when hum is invalid */
        if( j == 5 && !empty )
          empty = mtepd_print_data_is_empty(mtepd_load_history_data(p, 6, i), 6);
        p_vs = (vs_type_t*)&vs[j];
        if( j >= 6 && (p->drv.s->slot->thld.u16[1] & 0x400) != 0 ) {
          /* swap hr to aw displaying */
          p_vs = (vs_type_t*)&vs_aw;
        }        
        if( log ) {
          p->drv.log->data(p_vs->name, r);
        }
        if( empty ) {
          os->sprintf(buf, " %3s:   --   %4s", p_vs->name, p_vs->unit);
        } else {
          mtepd_print_data_vs(os, buf, p_vs, r, 1);
        }
        mtepd_fbuf_puts_mix16_xy(
          p,
          x + (i&2)/2*MB_EPD_WIDTH/2, y + (i/4)*(MB_EPD_HEIGHT/2+8) + step * 24,
          buf
        );
        step++;
      }
    }
    mtepd_fbuf_line_x(p, 0, MB_EPD_WIDTH-1, MB_EPD_HEIGHT/2-16);
    mtepd_fbuf_line_x(p, 0, MB_EPD_WIDTH-1, MB_EPD_HEIGHT/2+14);
    mtepd_fbuf_line_y(p, 96, MB_EPD_HEIGHT/2-16, MB_EPD_HEIGHT/2+14);
    mtepd_fbuf_line_y(p, 312, MB_EPD_HEIGHT/2-16, MB_EPD_HEIGHT/2+14);
    mtepd_fbuf_putchar8x16_xy(p, MB_EPD_WIDTH/2-16, MB_EPD_HEIGHT/2-32, '1', 1);
    mtepd_fbuf_putchar8x16_xy(p, MB_EPD_WIDTH/2-16, MB_EPD_HEIGHT/2-8, '2', 1);    
    mtepd_fbuf_putchar8x16_xy(p, MB_EPD_WIDTH/2+8, MB_EPD_HEIGHT/2-32, '3', 1);    
    mtepd_fbuf_putchar8x16_xy(p, MB_EPD_WIDTH/2+8, MB_EPD_HEIGHT/2-8, '4', 1);    
    mtepd_fbuf_putchar8x16_xy(p, MB_EPD_WIDTH/2-16, MB_EPD_HEIGHT/2+16, '5', 1);
    mtepd_fbuf_puthz16x16_xy(p, MB_EPD_WIDTH/2+3, MB_EPD_HEIGHT/2+15, MHZ_ZHU, 1);
    for( i = 0; i < 2; i++ ) {
      mtepd_fbuf_circle(p, MB_EPD_WIDTH/2, MB_EPD_HEIGHT/2-16, 24+i, 0x3);
      mtepd_fbuf_circle(p, MB_EPD_WIDTH/2, MB_EPD_HEIGHT/2+16, 24+i, 0xc);
      mtepd_fbuf_line_y(p, MB_EPD_WIDTH/2 - 24 - i, MB_EPD_HEIGHT/2-16, MB_EPD_HEIGHT/2+16);
      mtepd_fbuf_line_y(p, MB_EPD_WIDTH/2 + 24 + i, MB_EPD_HEIGHT/2-16, MB_EPD_HEIGHT/2+16);
    }
  }
  
  /* 显示RFID数字 */  
  rfid = RFID_GET_U32(p);
  os->sprintf(buf, "[0%09u]", rfid );
  x = 200;
  y = 150;
  mtepd_fbuf_puts_mix16_xy(p, x+32, y+24, buf);
  
  /* 显示最近一次联网状态及信号强度(非实时) */  
#if 0  
  mtepd_network(p, 96, 0, (p->drv.ext->flag & MA_EXT_FLAG_MODULE_READY) == 0 ? 0 : 100);
#endif

  /* 显示电量百分比 */
  mtepd_battery(p, x+164, y+28, VBAT_PER_GET(p));
  
  /* 显示不同模式内容 */
  switch( disp_mode ) {
  case 0: 
    /* RFID为空显示未扫码提示并告警 */
    if( rfid == 0 ) {
      const char wsm[] = {MHZ_WEI, MHZ_SAO, MHZ_MA, 0};
      mtepd_fbuf_puts_mix16_xy(p, x+80, y+72, (char*)wsm);

      /* 无效RFID告警 */
      if( (p->flags & DRV_MTEPD_FLAG_NO_ALARM) == 0 ) {
        mtepd_alarm_set(p, 1);
        p->flags |= DRV_MTEPD_FLAG_NO_ALARM; /* one round of alarm */
      }
      break;
    } else {
      /* RFID详细解析 */
      char h[16];
      h[0] = MHZ_ZHI;
      h[1] = mtepd_rfid_to_workshop(rfid, 3) == 1 ? MHZ_JIU : MHZ_QU;
      h[2] = ':';
      os->sprintf(&h[3], "%02u", 99); /* RFID does NOT have CHEJIAN no, always use special value here */
      mtepd_fbuf_puts_mix16_xy(p, x+32, y+64, h);

      h[0] = MHZ_CHE;
      h[1] = MHZ_JIAN;
      h[2] = ':';
      os->sprintf(&h[3], "%02u", mtepd_rfid_to_workshop(rfid, 0));
      mtepd_fbuf_puts_mix16_xy(p, x+112, y+64, h);

      h[0] = MHZ_BAN;
      h[1] = MHZ_ZU;
      h[2] = ':';
      os->sprintf(&h[3], "%02u", mtepd_rfid_to_workshop(rfid, 1));
      mtepd_fbuf_puts_mix16_xy(p, x+32, y+96, h);

      i = mtepd_rfid_to_workshop(rfid, 4);
      switch( i ) {
      case 1:
        h[0] = MHZ_LIANG;
        h[1] = MHZ_DUI;
        break;
      case 2:
        h[0] = MHZ_XIAO;
        h[1] = MHZ_DUI;  
        break;
      default:
        h[0] = MHZ_JIAO;
        h[1] = MHZ_CHI;  
        break;
      }
      h[2] = ':';
      os->sprintf(&h[3], "%02u", mtepd_rfid_to_workshop(rfid, 3));
      mtepd_fbuf_puts_mix16_xy(p, x+112, y+96, h);
    }    
    break;

  case 1: {
    const char cg[] = {MHZ_CHENG, MHZ_GONG,0};    
    /* 成功 */
    mtepd_fbuf_puts_mix16_xy(p, x+80, y+72, (char*)cg);
    break;
  }

  case 2: {
    const char qsk[] = {MHZ_QING, MHZ_SHUA, MHZ_KA, 0};
    /* 请刷卡 */
    mtepd_fbuf_puts_mix16_xy(p, x+80, y+72, (char*)qsk);
    break;
  }

  case 3: {
    /* Starting... */
    char buf[32];
    os->sprintf(buf, "POS:%i", os->version);
    mtepd_fbuf_puts_mix16_xy(p, x+32, y+32, buf);
    os->sprintf(buf, "Main:%i", p->drv.ma_version);
    mtepd_fbuf_puts_mix16_xy(p, x+32, y+32+16, buf);
    v = p->drv.s->drv.version;
    os->sprintf(buf, "\nS%04x:%04x", (v>>16)&0xffff, v&0xffff);
    mtepd_fbuf_puts_mix16_xy(p, x+32, y+32+32, buf);
    mtepd_fbuf_puts_mix16_xy(p, x+32, y+32+48, "Starting......");
    break;
  }
  
  default:
    break;
  }
  
  /* 绘制边框 */
  mtepd_fbuf_rectangle(p, 0, 0, MB_EPD_WIDTH-1, MB_EPD_HEIGHT-1);
  mtepd_fbuf_rectangle_solid(p, 184, 0, 216, 16);
  mtepd_fbuf_rectangle_solid(p, 176, 24, 224, 26);
  mtepd_fbuf_rectangle_solid(p, MB_EPD_WIDTH/2-1, 0, MB_EPD_WIDTH/2+1, MB_EPD_HEIGHT-8);
  for( i = 8; i > 1; i-- ) {
    mtepd_fbuf_line_x(p, MB_EPD_WIDTH/2-(i-1), MB_EPD_WIDTH/2+(i-1), MB_EPD_HEIGHT-i);
  }

  if( log ) {
    p->drv.os->flush();
    p->drv.log->data("mtepd draw", os->tick->elaps(tick));
  }  

  /* always alarm set off before refresh done */
  mtepd_alarm_set(p, 0);

  /* 刷新帧缓存到屏幕 */
  mtepd_show(p);

  /* 释放帧缓存内存(不影响当前屏幕显示内容) */
  mtepd_done(p);

}

/**
 * @brief EPD屏幕驱动获取时间参数
 * @param[in] p 参数控制块
 */
void mtepd_time_collect(mtepd_param_t *p) {
  pos_u8_t *arg = p->drv.s->slot->arg.u8;
  p->epd_time = 100;
  if( arg[0] )
    p->epd_time = 100 * arg[0];
  p->rfid_time = 2000;
  if( arg[1] )
    p->rfid_time = 100 * arg[1];
  p->erase_time = 10000;  
  if( arg[2] )
    p->erase_time = 100 * arg[2];
  p->keep_time = 10000;
  if( arg[3] )
    p->keep_time = 100 * arg[3];  
}

/**
 * @brief EPD屏幕驱动判断按键按下时长
 * @param[in] p 参数控制块
 * @param[in] max_time 最大按键时间
 */
pos_u32_t mtepd_btn_time_get(mtepd_param_t *p, pos_u32_t max_time) {
  pos_u32_t ticks, v, beep_flag;
  const pos_lib_t *os;
  os = p->drv.os;

  /* 记录中断时刻 */
  ticks = p->irq_ticks;  
  p->irq_ticks = 0;

  /* 如果当前按钮为弹开则忽略本次中断 */
  if( p->pin_btn == POS_PH31 || os->gpio->get(p->pin_btn) != 0 ) {
    return 0;
  }

  /* 如果从未中断则从当前时刻判断时长 */
  if( !ticks )
    ticks = os->tick->get();
  
  /* 如果中断距今已经很久则最多认为EPD亮屏时间 */
  if( os->tick->elaps(ticks) > p->epd_time )
    ticks = os->tick->get() - p->epd_time;

  /* 计算超时 */
  max_time = ticks + max_time;
  
  /* 检查直到最大时长超时 */
  beep_flag = 0;
  while(!os->tick->is_timeout(os->tick->get(), max_time)) {
    /* 按钮松开立即退出 */
    if( os->gpio->get(p->pin_btn) != 0 ) {
      break;
    }
    /* 休眠并直到按钮松开或者超时 */
    os->tick->sleep(20);

    v = os->tick->elaps(ticks);
    /* 如果超过亮屏时长则检查是否告警关闭 */
    if( v >= p->epd_time )
      mtepd_alarm_set(p, 0);

    /* 达到扫码时长后启动提示音 */
    if( !beep_flag && v >= p->rfid_time && v < p->rfid_time + 500 ) {
      p->drv.board->beep_set(2700);
      beep_flag = 1;
    /* 扫码提示音超时关闭 */
    } else if( (p->flags & 2) && v >= p->rfid_time + 500 ) {
      p->drv.board->beep_set(0);
      beep_flag = 0;
    }
    
  }

  /* 关闭扫码提示 */
  if( beep_flag ) {
    p->drv.board->beep_set(0);
  }
  
  /* 返回按钮时长 */
  return os->tick->elaps(ticks);
}


/** 
 * @brief 设置告警状态
 * @param[in] p 参数控制块
 * @param[in] mode 告警状态
 */
void mtepd_alarm_set(mtepd_param_t *p, pos_u32_t mode) {
  if( mode == 0 ) {
    /* 忽略: 已经关闭 */
    if( (p->flags & DRV_MTEPD_FLAG_IN_ALARM) == 0 ) 
      return;
    
    /* 关闭灯光 */
    MB_PIN_LED_SET_R(p->drv.os, 0);
    
    /* 关闭PWM告警 */
    p->drv.board->beep_set(0);

    /* 清除标志 */
    p->flags &= ~1;
  } else {
    /* 忽略: 已经设置 */  
    if( p->flags & DRV_MTEPD_FLAG_IN_ALARM )
      return;

    /* 打开灯光 */
    MB_PIN_LED_SET_R(p->drv.os, 1);
    
    /* 打开PWM告警 */
    p->drv.board->beep_set(2700);

    /* 设置标志 */    
    p->flags |= DRV_MTEPD_FLAG_IN_ALARM;
  }
}

