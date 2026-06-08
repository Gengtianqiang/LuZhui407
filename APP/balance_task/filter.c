#include "filter.h"

ema_t  emaL = 
{
	.ema = EMA_TARGET,
	.newdata = EMA_TARGET,
	.newdata_isUpdated = false,
};
ema_t  emaR = 
{
    .ema = EMA_TARGET,
    .newdata = EMA_TARGET,
    .newdata_isUpdated = false,
};

key_t  emaKey = 
{
   .key = KEY_TARGET,
   .newdata = KEY_TARGET,
   .newdata_isUpdated = false,
};



void ema_set_new_data(ema_t *self, int data) 
{
    self->newdata = data;
    self->newdata_isUpdated = true;
}

int ema_filter(ema_t *self)
{
	if (self->newdata_isUpdated) {
		// EMA滤波公式
		// ema = (ALPHA * new_data + (255 - ALPHA) * ema) / 255;
		// 简单低通滤波公式
		// self->ema = (self->ema * (255 - EMA_ALPHA) + self->newdata * EMA_ALPHA) / 255;
		self->ema = self->newdata;
        // 处理完新数据后重置标志位
		self->newdata_isUpdated = false; 
		self->lost_cnt = 0;
	} 
    else {
		// 没有新数据时，逐渐回归到127
		// self->ema = (EMA_ALPHA * EMA_TARGET + (255 - EMA_ALPHA) * self->ema) / 255;
		// // 没有新数据时，逐渐回归到127
		// if (self->ema < EMA_TARGET - 10) {
		// 	self->ema += 10;
		// }
        // else if (self->ema > EMA_TARGET + 10) {
		// 	self->ema -= 10;
		// }
		// else{
		// 	self->ema = EMA_TARGET;
		// }
		if(self->lost_cnt < 8){ // 18ms x 5 = 90ms
			self->lost_cnt++;
		}else{
			self->ema = EMA_TARGET;
		}
	}
    return self->ema;
}

void key_set_new_data(key_t *self, uint8_t MESH_user_KEY)
{
	self->newdata = MESH_user_KEY;
	self->newdata_isUpdated = true;
}

uint8_t key_filter(key_t *self)
{
	static uint8_t time = 0;
	if (self->newdata_isUpdated) {
		self->key = self->newdata;
		time = 0;
		self->newdata_isUpdated = false; // 处理完新数据后重置标志位
	} 
    else {
		if(time > 3){
			self->key = KEY_TARGET;	
			time = 0;
		}else{
			self->key = self->key;
			time++;
		}
	}
    return self->key;
}









