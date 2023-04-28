/**
 * @file user_draw.h
 * @author {白秉鑫}-{bbx20010518@outlook.com}
 * @brief
 * @version 0.1
 * @date 2023-04-26
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef USER_DRAW_H
#define USER_DRAW_H

#define FH_SOF 0xA5
#define FH_COM_ID 0x0301
/*-************************赛前修改TODO:****************************-*/
/**
 * @brief 当前机器人ID
 * BLUE_1 103
 * BLUE_2 104
 * BLUE_3 105
 * RED_1 3
 * RED_2 4
 * RED_3 5
 */
#define NOW_ID 105

/**
 * @brief 红方步兵客户端ID
 * 0x0103 1
 * 0x0104 2
 * 0x0105 3
 */
#define CLIENT_INFANTRY_RED 0x0105
/**
 * @brief 蓝方步兵客户端ID
 * 0x0167 1
 * 0x0168 2
 * 0x0169 3
 */
#define CLIENT_INFANTRY_BLUE 0x0169

/*-****************************************************-*/
#define COMMUNICATION_ID 0x0301    // 学生机器人间通信ID
#define DRAW_SEVEN_GRAPH_ID 0x0104 // 绘制7个图形ID
#define DRAW_CHARACTER_ID 0x0110   // 绘制字符ID

#define CLIENT_LENGHT 1920 // 屏幕长
#define CLIENT_WIDTH 1080  // 屏幕宽

void draw_UI_task(void const *argument);

#endif // USER_DRAW_H
