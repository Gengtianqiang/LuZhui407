#include "uart_task.h"


uint8_t dma_buffer_receive_usart1[RECEIVE_USART1_BUFFER] = {0};
uint8_t dma_buffer_Transmit_usart1[Transmit_USART1_BUFFER] = {0};
uint32_t Encoder_num = 0;
uint32_t Encoder_voctor = 0;

extern mesh_datasend_pkt_t             my_mesh_send_pkt;

void StartUsartTask(void *argument)
{
	/* USER CODE BEGIN StartUsartTask */
	
    jdy_inst(&jdy_handle,
		    &mesh_config,
			&tx_config,
			&rx_config,
			&time_config,
			&function_config);

	jdy_handle.p_mesh_submode->mesh_init(&jdy_handle);


	dtu_inst(&my_4g_dtu, &dtu_time_config);

	/* Infinite loop */
	for (;;)
	{
		osDelay(50);


	my_4g_dtu.parser_fun(&my_4g_dtu,dtu_rx_buffer);


	Protocol_Parse(&ring_rx_DMA_buf, &proto_data);



#ifdef AHAND_CAR
 	if (ParseTwrProtocol(&ring3_rx_DMA_buf, &bracket_data))
 	{
 			bracket_data.twr_status = (ParseBracketContentToFloats(&bracket_data));
 	}
#endif

#ifdef BEHIND_CAR
		Protocol_Parse(&ring3_rx_DMA_buf, &retuen_proto_data);
#endif


 	if(JDY_OK==jdy_task(&jdy_handle, &my_mesh_send_pkt,&proto_data)) {

 		my_mesh_send_pkt.valid = 1;

 	}else my_mesh_send_pkt.valid = 0;


	

	}
	/* USER CODE END StartUsartTask */
}


