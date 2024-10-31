# CAN LL Library for STM32F4

## HARDWARE CHECK

* Both boards share a common ground.

* Check the connectors and pads of the CAN interface.

## Define CAN Handler, Filter, and Header

```
    LL_CAN_Handler_t hcan;
    LL_CAN_FilterTypeDef_t hfilter;
    LL_CAN_TxHeaderTypeDef_t Txheader;
    LL_CAN_RxHeaderTypeDef_t Rxheader;
```
> [!IMPORTANT]
> Define the CAN instance first:

```
    hcan.Instance = _CAN1 
```

or 

```
    hcan.Instance = _CAN2
```

## CAN Initialization

***1. GPIO Initialization***

```
    LL_CAN_GPIO_Init(&hcan);
```

***2. Enable IRQ***

Example:

```
    NVIC_SetPriority(CAN1_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_EnableIRQ(CAN1_TX_IRQn);
```

***3. Enable Interrupt Flags***

Example:

```
    LL_CAN_ActivateInterrupt(&hcan, _CAN_IT_RX_FIFO0_MSG_PENDING_Pos | _CAN_IT_TX_MAILBOX_EMPTY_Pos);
```

> [!IMPORTANT]
> Use activate interrupt function after config filter.

***4. Initialize CAN Parameters (Baudrate, Mode, Status)***

```
    hcan.Init.Mode = _LOOPBACK_MODE;
    hcan.Init.status.AutoBusOff = DISABLE;
    hcan.Init.status.AutoRetransmission = ENABLE;
    hcan.Init.status.AutoWakeUp = DISABLE;
    hcan.Init.status.ReceiveFifoLocked = DISABLE;
    hcan.Init.status.TimeTriggeredMode = DISABLE;
    hcan.Init.status.TransmitFifoPriority = DISABLE;

    LL_CAN_Init(&hcan);
```
> [!NOTE]
> Bit rate will be calculated based on APB1 frequency: 4MHz, 8MHz, 12MHz, 24MHz.


***5. Configure CAN Filter***

```
    hfilter1.FilterActivation = _CAN_FILTER_ENABLE;
    hfilter1.FilterBank = 0;
    hfilter1.FilterFIFOAssignment = _CAN_FILTER_FIFO0;
    hfilter1.FilterIdHigh = 0;
    hfilter1.FilterIdLow = 0;
    hfilter1.FilterMaskIdHigh = 0;
    hfilter1.FilterMaskIdLow = 0;
    hfilter1.FilterMode = _CAN_FILTERMODE_IDMASK;
    hfilter1.FilterScale = _CAN_FILTERSCALE_32BIT;

    LL_CAN_ConfigFilter(&hcan1, &hfilter1);
```

> [!IMPORTANT]
> Enable the filter to use reception mode.

## Can start

> [!IMPORTANT]
> Call the CAN start function after all initializations to ensure proper protocol execution.

```
    LL_CAN_Start(&hcan);
```

## CAN Transmit and Receive Functions

***1. Transmission***

```
    LL_CAN_AddTxMessage(LL_CAN_Handler_t *hcan, const uint8_t data[], LL_CAN_TxHeaderTypeDef_t *htxheader, uint32_t *TxMailBox);

    LL_CAN_IsTxMessagePending(LL_CAN_Handler_t *hcan, uint32_t *TxMailBox);
```
> [!NOTE]   
> If using interrupt, call only the second function. 


***2. Reception***

```
    LL_CAN_GetRxFifoFillLevel(LL_CAN_Handler_t *hcan, uint32_t RxFifo);

    LL_CAN_GetRxMessage(LL_CAN_Handler_t *hcan, LL_CAN_RxHeaderTypeDef_t *hrxheader, uint8_t rxdata[], uint32_t RxFifo);

```

> [!NOTE]  
> If using interrupt, call only the second function.

## CAN IRQ FUNCTION

```
    /**
    * @brief This function handles CAN1 TX interrupts.
    */
    void CAN1_TX_IRQHandler(void)
    {
    LL_CAN_IRQHandler(&hcan1);
    }

    /**
    * @brief This function handles CAN1 RX0 interrupts.
    */
    void CAN1_RX0_IRQHandler(void)
    {
    LL_CAN_IRQHandler(&hcan1);
    }

    void CAN1_RX1_IRQHandler(void)
    {
    LL_CAN_IRQHandler(&hcan1);
    }

    /**
    * @brief This function handles CAN1 SCE interrupt. (Status Change Error)
    */
    void CAN1_SCE_IRQHandler(void)
    {
    LL_CAN_IRQHandler(&hcan1);
    }

```
