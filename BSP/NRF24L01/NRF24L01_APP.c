#include "NRF24L01_APP.h"
#include "twr_control.h"
uint8_t SendFlag;								//发送标志位
uint8_t SendSuccessCount, SendFailedCount;		//发送成功计次，发送失败计次

uint8_t ReceiveFlag;							//接收标志位
uint8_t ReceiveSuccessCount, ReceiveFailedCount;//接收成功计次，接收失败计次

static void NRF24L01_SetAddress(uint8_t *Address, uint8_t Address4)
{
    Address[0] = 0x00;
    Address[1] = 0x00;
    Address[2] = 0x00;
    Address[3] = 0xAA;
    Address[4] = Address4;
}

uint8_t NRF24L01_TASK(void)
{
    

    #ifdef AHAND_CAR
    //头车前进过程不需要接收
    // NRF24L01_RxAddress = {0x00, 0x00, 0x00, 0xAA, 0xA1}; // user  //select_node_M()
    // ReceiveFlag = NRF24L01_Receive();

    //前进完成向二车发送前进指令
    if (1 == g_state_machine.ahand_flag)
    {   
        NRF24L01_SetAddress(NRF24L01_TxAddress, 0xA2); // user  //select_node_M()
        NRF24L01_TxPacket[0] = 0x00;
        NRF24L01_TxPacket[1] = 0x00;
        NRF24L01_TxPacket[2] = 0x00;
        NRF24L01_TxPacket[3] = 0x01;
        SendFlag = NRF24L01_Send();
        if (SendFlag == 1)			//发送标志位为1，
        {
            JDY_DEBUG_OUT("Successfully sent packet: %02X %02X %02X %02X\r\n", NRF24L01_TxPacket[0], NRF24L01_TxPacket[1], NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
        else
        {
            JDY_DEBUG_OUT("Failed to send packet: %02X %02X %02X %02X\r\n", NRF24L01_TxPacket[0], NRF24L01_TxPacket[1], NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
    }

        //向所有中间车发送返回指令
    else if (3 == g_state_machine.ahand_flag)
    {
        NRF24L01_SetAddress(NRF24L01_TxAddress, 0xA2); // user  //select_node_M()
        NRF24L01_TxPacket[0] = 0x00;
        NRF24L01_TxPacket[1] = 0x00;
        NRF24L01_TxPacket[2] = 0x00;
        NRF24L01_TxPacket[3] = 0x02;
        SendFlag = NRF24L01_Send();
        if (SendFlag == 1)			//发送标志位为1，
        {
            JDY_DEBUG_OUT("Successfully sent packet: %02X %02X %02X %02X\r\n", NRF24L01_TxPacket[0], NRF24L01_TxPacket[1], NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
        else
        {
            JDY_DEBUG_OUT("Failed to send packet: %02X %02X %02X %02X\r\n", NRF24L01_TxPacket[0], NRF24L01_TxPacket[1], NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
        }
    }
#endif
/*后车接收到所有中间车返回信号后返回*/
#ifdef BEHIND_CAR
    NRF24L01_SetAddress(NRF24L01_RxAddress, 0xA5); // user  //select_node_M()
    NRF24L01_UpdateRxAddress();
    ReceiveFlag = NRF24L01_Receive();
    if (1 == ReceiveFlag)
    {
        if (2 == NRF24L01_RxPacket[3]) {
            g_state_machine.behind_flag = 1;
            JDY_DEBUG_OUT("Behind Car received return signal.\n");
        }
    }
    

#endif



/*所有中间车接收到头车返回信号后集体向后车发送返回信号*/  
#ifdef MIDDLE_CAR
    // if (2 == self->p_mesh_submode->p_parser->recv_pkt.L)
    // {
        
    //     g_state_machine.middle_flag = 1;
    //     pkt->to_maddr = 0x0004U; // user  //select_node_M()
    //     pkt->L = 4;
    //     pkt->R = 4;
    //     pkt->valid = 0x01; // user  //key
    //     res = jdy_handle.p_mesh_submode->p_parser->pf_mesh_datasend_handler(self, pkt);
    // }

#endif

 /*一号中间车接收到头车前进信号后前进*/ 
#ifdef MIDDLE_CAR_FIRST
    NRF24L01_SetAddress(NRF24L01_RxAddress, 0xA2); // user  //select_node_M()
    NRF24L01_UpdateRxAddress();
    ReceiveFlag = NRF24L01_Receive();
    if (1 == ReceiveFlag)
    {
        if (1 == NRF24L01_RxPacket[3]) {
            g_state_machine.middle_flag = 1;
            JDY_DEBUG_OUT("Middle Car 1 received forward signal.\n");
        }
    

        else if (2 == NRF24L01_RxPacket[3])
        {
            g_state_machine.middle_flag = 2;
            JDY_DEBUG_OUT("Middle Car 1 received return signal.\n");
            NRF24L01_SetAddress(NRF24L01_TxAddress, 0xA5); // user  //select_node_M()
            NRF24L01_TxPacket[0] = 0x00;
            NRF24L01_TxPacket[1] = 0x00;
            NRF24L01_TxPacket[2] = 0x00;
            NRF24L01_TxPacket[3] = 0x02;
            SendFlag = NRF24L01_Send();
            if (SendFlag == 1)			//发送标志位为1，
            {
                JDY_DEBUG_OUT("Successfully sent packet: %02X %02X %02X %02X\r\n", NRF24L01_TxPacket[0], NRF24L01_TxPacket[1], NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
            }
            else
            {
                JDY_DEBUG_OUT("Failed to send packet: %02X %02X %02X %02X\r\n", NRF24L01_TxPacket[0], NRF24L01_TxPacket[1], NRF24L01_TxPacket[2], NRF24L01_TxPacket[3]);
            }
        }
    }
#endif
    return 0;
}







