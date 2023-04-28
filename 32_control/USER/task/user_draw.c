#include "user_draw.h"
#include "main.h"
#include "user_c.h"

/**
 * @brief 绘制数据
 *
 */
typedef __packed struct
{
  uint8_t graphic_name[3];   // 图形名
  uint32_t operate_tpye : 3; // 图形操作 0空操作 1增加 2修改 3删除
  uint32_t graphic_tpye : 3; // 图形类型 0直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整型数 7字符
  uint32_t layer : 4;        // 图层数0~9
  uint32_t color : 4;        // 颜色 0红蓝主色 1黄 2绿 3橙 4紫红 5粉 6青 7黑 8白
  uint32_t start_angle : 9;  // 起始角度，单位：°，范围[0,360]
  uint32_t end_angle : 9;    // 终止角度，单位：°，范围[0,360]
  uint32_t width : 10;       // 线宽
  uint32_t start_x : 11;     // 起点 x 坐标
  uint32_t start_y : 11;     // 起点 y 坐标
  uint32_t radius : 10;      // 字体大小或者半径
  uint32_t end_x : 11;       // 终点 x 坐标
  uint32_t end_y : 11;       // 终点 y 坐标
} graphic_data_struct_t;

/**
 * @brief 客户端绘制7个图形
 *
 */
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct[7]; // 绘制图像的数量即图像数据数组的长度，但要看清楚裁判系统给的增加图形数量对应的内容ID
} ext_client_custom_graphic_t;

/**
 * @brief 交互信息
 *
 */
typedef __packed struct user_draw
{
  /* data */
  uint16_t data_cmd_id;                       // 数据段内容ID
  uint16_t sender_ID;                         // 发送者ID
  uint16_t receiver_ID;                       // 接受者ID
  ext_client_custom_graphic_t graphic_custom; // 自定义图形数据
} ext_student_interactive_header_data_t;

#define MAX_SIZE 128       // 最大数据长度
#define FRAMEHERADER_LEN 5 // 数据帧头长度
#define CMD_LEN 2          // 命令长度
#define CRC_LEN 2          // CRC校验长度
uint8_t seq = 0;

ext_student_interactive_header_data_t custom_grapic_draw; // 自定义图像绘制

/**
 * @brief 自定义图形绘制
 * 图形
 */
static void draw_User_UI(void)
{
  custom_grapic_draw.data_cmd_id = DRAW_SEVEN_GRAPH_ID;  // 绘制七个图形（内容ID，查询裁判系统手册）
  custom_grapic_draw.sender_ID = NOW_ID;                 // 发送者ID，机器人对应ID，此处为蓝方英雄
  custom_grapic_draw.receiver_ID = CLIENT_INFANTRY_BLUE; // 接收者ID，操作手客户端ID，此处为蓝方英雄操作手客户端
  // 自定义图像数据
  {
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 0;
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 0;
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 1; // 图形名
    // 上面三个字节代表的是图形名，用于图形索引，可自行定义
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye = 1; // 图形操作，0：空操作；1：增加；2：修改；3：删除；
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye = 0; // 图形类型，0为直线，其他的查看用户手册
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer = 1;        // 图层数
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].color = 1;        // 颜色
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle = 0;
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle = 0;
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].width = 10;

    custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x = 600;
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y = 600;
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x = 100;
    custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y = 200;

    custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius = 0;
  }
}
/**
 * @brief 发送数据打包
 *
 * @param sof 数据帧起始字节，固定值为 0xA5
 * @param cmd_id 命令码 0x0301
 * @param p_data
 * @param len
 */
static void send_Draw_Data(uint8_t *p_data, uint16_t len)
{
  unsigned char i = 0;

  uint8_t tx_buff[MAX_SIZE];

  uint16_t frame_length = FRAMEHERADER_LEN + CMD_LEN + len + CRC_LEN; // 数据帧长度

  memset(tx_buff, 0, frame_length); // 存储数据的数组清零

  /*****帧头打包*****/
  tx_buff[0] = FH_SOF;                               // 数据帧起始字节
  memcpy(&tx_buff[1], (uint8_t *)&len, sizeof(len)); // 数据帧中data的长度
  tx_buff[3] = seq;                                  // 包序号
  append_CRC8_check_sum(tx_buff, FRAMEHERADER_LEN);  // 帧头校验CRC8

  /*****命令码打包*****/
  memcpy(&tx_buff[FRAMEHERADER_LEN], (uint8_t *)FH_COM_ID, CMD_LEN);

  /*****数据打包*****/
  memcpy(&tx_buff[FRAMEHERADER_LEN + CMD_LEN], p_data, len);
  append_CRC16_check_sum(tx_buff, frame_length); // 一帧数据校验CRC16
  if (seq == 0xff)
    seq = 0;
  else
    seq++;
 
  /*发送数据*/
  usart6_tx_dma_enable(tx_buff, frame_length);
}
/**
 * @brief 绘制UI界面任务
 *
 */
void draw_UI_task(void const *argument)
{
  draw_User_UI();
  while (1)
  {
    /* code */
    send_Draw_Data((uint8_t *)&custom_grapic_draw, sizeof(custom_grapic_draw));
    vTaskDelay(500); // 发送数据频率最大10hz
  }
}
