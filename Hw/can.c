#include "can.h"
#include "board.h"
#include "config.h"

static CanFrame_t s_rx[CFG_CAN_RX_SLOTS];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_bus_off_pending;

static void Can_ConfigFilters(uint8_t id)
{
  static const uint16_t bases[5] = {
    CFG_CAN_MOTION_BASE,
    CFG_CAN_VEL_BASE,
    CFG_CAN_GAINS_BASE,
    CFG_CAN_POS_BASE,
    CFG_CAN_MGMT_BASE
  };
  FDCAN_FilterTypeDef filter = {0};
  uint32_t i;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID2 = 0x7FFU;

  for (i = 0U; i < 5U; i++)
  {
    filter.FilterIndex = i;
    filter.FilterID1 = (uint32_t)bases[i] + (uint32_t)id;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void Can_Init(uint8_t node_id)
{
  uint8_t id = ((node_id >= 1U) && (node_id <= 63U)) ? node_id : 1U;

  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_bus_off_pending = 0U;

  Can_ConfigFilters(id);

  (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, CFG_NVIC_CAN, 0U);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     (uint32_t)(FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF),
                                     0U) != HAL_OK)
  {
    Error_Handler();
  }
}

void Can_ProcessRxIrq(void)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
  {
    uint8_t next;
    uint8_t i;

    if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
        (rx_header.DataLength != FDCAN_DLC_BYTES_8) ||
        (rx_header.FDFormat != FDCAN_CLASSIC_CAN))
    {
      continue;
    }

    next = (uint8_t)((s_rx_head + 1U) % CFG_CAN_RX_SLOTS);
    if (next == s_rx_tail)
    {
      continue;
    }

    s_rx[s_rx_head].id = rx_header.Identifier;
    for (i = 0U; i < 8U; i++)
    {
      s_rx[s_rx_head].data[i] = rx_data[i];
    }
    s_rx_head = next;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  (void)RxFifo0ITs;
  if (hfdcan->Instance == FDCAN1)
  {
    Can_ProcessRxIrq();
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if ((hfdcan == &hfdcan1) && ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U))
  {
    s_bus_off_pending = 1U;
  }
}

void Can_StopForFlash(void)
{
  NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  if (hfdcan1.State == HAL_FDCAN_STATE_BUSY)
  {
    (void)HAL_FDCAN_Stop(&hfdcan1);
  }
  SET_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
  hfdcan1.State = HAL_FDCAN_STATE_READY;
  hfdcan1.ErrorCode = HAL_FDCAN_ERROR_NONE;
}

void Can_Restart(void)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_bus_off_pending = 0U;
  hfdcan1.ErrorCode = HAL_FDCAN_ERROR_NONE;
  SET_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
  hfdcan1.State = HAL_FDCAN_STATE_READY;

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return;
  }

  while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
  {
  }

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, CFG_NVIC_CAN, 0U);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  (void)HAL_FDCAN_ActivateNotification(
      &hfdcan1,
      (uint32_t)(FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF),
      0U);
}

void Can_Service(void)
{
  if ((s_bus_off_pending == 0U) && ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U))
  {
    return;
  }
  Can_StopForFlash();
  Can_Restart();
}

uint8_t Can_PopRx(CanFrame_t *frame)
{
  uint8_t i;
  uint8_t tail;

  if (frame == 0)
  {
    return 0U;
  }

  tail = s_rx_tail;
  if (tail == s_rx_head)
  {
    return 0U;
  }

  frame->id = s_rx[tail].id;
  for (i = 0U; i < 8U; i++)
  {
    frame->data[i] = s_rx[tail].data[i];
  }
  s_rx_tail = (uint8_t)((tail + 1U) % CFG_CAN_RX_SLOTS);
  return 1U;
}

uint8_t Can_Send(uint32_t id, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef tx_header;

  tx_header.Identifier = id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data) != HAL_OK)
  {
    return 0U;
  }
  return 1U;
}
