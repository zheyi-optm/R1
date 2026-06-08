#include "bsp_uart.h"
#include "usart.h"

volatile imu_uart_ctx_t g_imu_uart_ctx = {0};

static uint16_t remote_last_pos = 0;   // ��һ��IDLEʱ��DMAдָ��

void BSP_USART_Init(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    (void)huart5.Instance->RDR;

    HAL_UART_Receive_DMA(&huart5, REMOTE_BUF, REMOTE_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}

/**
 * @brief  ��USART5�жϴ����������õ�IDLE����
 */
void REMOTE_IdleHandler(void)
{
    uint16_t cur_pos = REMOTE_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
    if (cur_pos != remote_last_pos) {
        REMOTE_ExtractFrame(REMOTE_BUF, REMOTE_BUF_SIZE,
                            remote_last_pos, cur_pos);
        remote_last_pos = cur_pos;
    }
}

static void REMOTE_ExtractFrame(const uint8_t *buf, uint16_t size,
                                uint16_t start, uint16_t end)
{
    static uint8_t  frame[9];
    static uint8_t  sync_state = 0;   // 0:��ͷ1, 1:��ͷ2, 2:������
    static uint8_t  frame_idx = 0;
    static uint8_t  head_byte = 0;    // ��¼��һ��֡ͷ�����ں����ж�֡����

    uint16_t i = start;
    while (i != end) {
        uint8_t byte = buf[i];
        switch (sync_state) {
            case 0:  // Ѱ��֡ͷ1��ֻ����0x5B��0x5C��
                if (byte == 0x5B || byte == 0x5C) {
                    head_byte = byte;
                    frame[0] = byte;
                    sync_state = 1;
                }
                // �����ֽ�ֱ�Ӷ���
                break;

            case 1:  // Ѱ��֡ͷ2��������֡ͷ1��ͬ��
                if (byte == head_byte) {
                    frame[1] = byte;
                    frame_idx = 2;
                    sync_state = 2;
                } else if (byte == 0x5B || byte == 0x5C) {
                    // ������һ���Ϸ�֡ͷ�����¿�ʼ
                    head_byte = byte;
                    frame[0] = byte;
                    // sync_state ����1
                } else {
                    // �Ƿ��ֽڣ��س�ʼ״̬
                    sync_state = 0;
                }
                break;

            case 2:  // �ռ������ֽڣ�ֱ������9�ֽ�
                frame[frame_idx++] = byte;
                if (frame_idx == 9) {
                    // ���֡β�Ƿ���֡ͷ��Ӧ��0x5B->0x2B��0x5C->0x2C
                    uint8_t expected_footer = (head_byte == 0x5B) ? 0x2B : 0x2C;
                    if (frame[8] == expected_footer) {
                        // ����֡ͷ���ò�ͬ�Ľ�������
                        if (head_byte == 0x5B || head_byte == 0x5C) 
												{
                            REMOTE_ParseData(frame, &RCctrl);   // ����֡
                        } 
                    }
                    // ����֡β�Ƿ���ȷ��������״̬����ֹ����
                    sync_state = 0;
                    frame_idx = 0;
                }
                // ��ѡ���ݴ�������ڵ�3���ֽڣ�����2���ֳ�����0x5B/0x5C������ͬ��
                // ��һ���ò�������ʡ�ԡ�
                break;
        }
        i = (i + 1) % size;
    }
}


/**
 * @brief ����RS485�շ�����
 * @param en: 1=����ģʽ, 0=����ģʽ
 * PD4 = USART2_DE (Ӳ������)
 */
void BSP_USART2_DE(uint8_t en)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, (en != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** 启动 USART2（IMU/RS485）ReceiveToIdle 中断接收 */
void BSP_USART2_StartRxIT(void)
{
    g_imu_uart_ctx.rx_ready = 0U;
    g_imu_uart_ctx.rx_size = 0U;
    g_imu_uart_ctx.start_rx_cnt++;
    g_imu_uart_ctx.uart2_gstate_dbg = (uint32_t)huart2.gState;
    g_imu_uart_ctx.uart2_rxstate_dbg = (uint32_t)huart2.RxState;
    g_imu_uart_ctx.uart2_isr_dbg = huart2.Instance->ISR;
    g_imu_uart_ctx.uart2_err_dbg = (uint32_t)huart2.ErrorCode;
    (void)HAL_UART_AbortReceive(&huart2);
    g_imu_uart_ctx.start_rx_ret =
        (uint32_t)HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)g_imu_uart_ctx.rx_buf, sizeof(g_imu_uart_ctx.rx_buf));
    if (g_imu_uart_ctx.start_rx_ret == 2U)
    {
        g_imu_uart_ctx.start_rx_busy_cnt++;
    }
}

/* ��HAL_UARTEx_RxEventCallback�д���HI14���� */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart2)  /* HI14������USART2 */
    {
        /* ������ݾ������������е���IMU_ParseFrame���� */
        g_imu_uart_ctx.rx_size = Size;
        g_imu_uart_ctx.rx_ready = 1U;
    }
}




