#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "jdy28m/jdy28m.h"


/* 摇杆滤波 */
#define EMA_ALPHA  252  // 这里的ALPHA定义为10表示回归速度(相当于0.1)，范围0到255
#define EMA_TARGET 0x7F // 回归目标值 0x7F

typedef struct {
    int     ema;
    int     newdata;
    bool    newdata_isUpdated;
    int     lost_cnt;
} ema_t;
extern ema_t  emaL, emaR ;

void ema_set_new_data(ema_t *self, int data) ;
int  ema_filter(ema_t *self);



/* 按键滤波 */
#define KEY_TARGET    MESH_KEY_NONE // 回归目标值

typedef struct {
    uint8_t     key;
    uint8_t     newdata;
    bool    newdata_isUpdated;
} key_t;
extern key_t  emaKey;

void    key_set_new_data(key_t *self, uint8_t MESH_user_KEY);
uint8_t key_filter(key_t *self);



#endif // FILTER_H  
