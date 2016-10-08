/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;



/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static CanTxMsgTypeDef        Tx1Message;
static CanRxMsgTypeDef        Rx1Message;
static CanTxMsgTypeDef        Tx2Message;
static CanRxMsgTypeDef        Rx2Message;
static uint8_t bCAN1_TxReq = 0;
static uint8_t bCAN2_TxReq = 0;

#define MAX_SEND_TRY_COUNT 15000 // this is about 20ms TX timeout (measured by scope)
static uint32_t iCAN1_Timeout = 0;
static uint32_t iCAN2_Timeout = 0;

/* USER CODE END PV */

// if defined CAN2 acting as loopback
//#define CAN1_LOOPBACK

// if defined CAN2 acting as loopback
//#define CAN2_LOOPBACK

// if defined CAN parameters are stored in MCU Flash
#define USE_FLASH

#ifdef USE_FLASH
  #define FLASH_USER_OFFSET 0x800F800
  int iCAN1_Prescaler        __attribute__((at(FLASH_USER_OFFSET+4*0))) = 6;
  int iCAN2_Prescaler        __attribute__((at(FLASH_USER_OFFSET+4*1))) = 6;
  int iCAN1_FilterIdHigh     __attribute__((at(FLASH_USER_OFFSET+4*2))) = 0;
  int iCAN1_FilterIdLow      __attribute__((at(FLASH_USER_OFFSET+4*3))) = 0;
  int iCAN1_FilterMaskIdHigh __attribute__((at(FLASH_USER_OFFSET+4*4))) = 0;
  int iCAN1_FilterMaskIdLow  __attribute__((at(FLASH_USER_OFFSET+4*5))) = 0;
  int iCAN2_FilterIdHigh     __attribute__((at(FLASH_USER_OFFSET+4*6))) = 0;
  int iCAN2_FilterIdLow      __attribute__((at(FLASH_USER_OFFSET+4*7))) = 0;
  int iCAN2_FilterMaskIdHigh __attribute__((at(FLASH_USER_OFFSET+4*8))) = 0;
  int iCAN2_FilterMaskIdLow  __attribute__((at(FLASH_USER_OFFSET+4*9))) = 0;
  int iReplace_Count          __attribute__((at(FLASH_USER_OFFSET+4*10))) = 0;
#else
  int iCAN1_Prescaler         = 6;
  int iCAN2_Prescaler         = 6;
  int iCAN1_FilterIdHigh      = 0;
  int iCAN1_FilterIdLow       = 0;
  int iCAN1_FilterMaskIdHigh  = 0;
  int iCAN1_FilterMaskIdLow   = 0;
  int iCAN2_FilterIdHigh      = 0;
  int iCAN2_FilterIdLow       = 0;
  int iCAN2_FilterMaskIdHigh  = 0;
  int iCAN2_FilterMaskIdLow   = 0;
  int iReplace_Count = 0;
#endif

#define FLASH_REPLACEMENT_OFFSET (FLASH_USER_OFFSET + 4*11)
#define FLASH_REPLACEMENT_SIZE (12)
unsigned int *iReplace_IDMask;
unsigned int *iReplace_IDFilter;
unsigned int *iReplace_NewIDMask;
unsigned int *iReplace_NewIDValue;
unsigned int *iReplace_DataMaskHigh;
unsigned int *iReplace_DataMaskLow;
unsigned int *iReplace_DataFilterHigh;
unsigned int *iReplace_DataFilterLow;
unsigned int *iReplace_NewDataMaskHigh;
unsigned int *iReplace_NewDataMaskLow;
unsigned int *iReplace_NewDataValueHigh;
unsigned int *iReplace_NewDataValueLow;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void User_GPIO_Init(void);
void LedG(uint8_t On);
void LedG_Toggle(void);
void LedB(uint8_t On);
void LedB_Toggle(void);
void LedR(uint8_t On);
void LedR_Toggle(void);
void ProcessModification(CanTxMsgTypeDef* pTxMsg);
void RunTests(void);
void CAN_CancelTransmit(CAN_HandleTypeDef* hcan);

void RxQueuePut(uint8_t data);
uint8_t RxQueueGet(void);
uint8_t RxQueueNotEmpty(void);

void UART_ProcessData(uint8_t rx);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  User_GPIO_Init();
  LedG(1);
  
  /*##-2- Start the Reception process and enable reception interrupt #########*/
  if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  
  /*##-2- Start the Reception process and enable reception interrupt #########*/
  if (HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  
  RunTests(); // this is function to run transmission tests 
 
  // this is loopback cycle
  while (1)
  {
    if (bCAN2_TxReq)
    {
      //LedR(1);
      ProcessModification(hcan2.pTxMsg);
      //LedR(0);
      if (HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
      {
        /* Transmition Error */
        LedR_Toggle();
        
        // if we tried hard so many times -- restart CAN
        iCAN2_Timeout++;
        if (iCAN2_Timeout > MAX_SEND_TRY_COUNT)
        {
          iCAN2_Timeout = 0;
          //CAN_CancelTransmit(&hcan2);
          HAL_NVIC_SystemReset();
        }
      }
      else
      {
        bCAN2_TxReq = 0;
        iCAN2_Timeout = 0;
      }
    }
    
    if (bCAN1_TxReq)
    {
      //ProcessModification(hcan1.pTxMsg); // TODO: maybe add CAN instance selector in replacement data?

      if (HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
      {
        /* Transmition Error */
        LedR_Toggle();
        
        // if we tried hard so many times -- restart CAN
        iCAN1_Timeout++;
        if (iCAN1_Timeout > MAX_SEND_TRY_COUNT)
        {
          iCAN1_Timeout = 0;
          //CAN_CancelTransmit(&hcan1);
          HAL_NVIC_SystemReset();
        }
      }
      else
      {
        bCAN1_TxReq = 0;
        iCAN1_Timeout = 0;
      }
    }
    
    if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet());
  }
  

  


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  CAN_FilterConfTypeDef  sFilterConfig;

  hcan1.Instance = CAN1;
  hcan1.pTxMsg = &Tx1Message;
  hcan1.pRxMsg = &Rx1Message;
  
  hcan1.Init.Prescaler = iCAN1_Prescaler; //3 -- 1Msps
  #ifdef CAN1_LOOPBACK
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  #else
    hcan1.Init.Mode = CAN_MODE_NORMAL;
  #endif
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_6TQ;
  hcan1.Init.BS2 = CAN_BS2_5TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);
  

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;


  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    // Filter configuration Error
    Error_Handler();
  }
}

/* CAN2 init function */
void MX_CAN2_Init(void)
{
  CAN_FilterConfTypeDef  sFilterConfig;

  hcan2.Instance = CAN2;
  hcan2.pTxMsg = &Tx2Message;
  hcan2.pRxMsg = &Rx2Message;
  
  hcan2.Init.Prescaler = iCAN2_Prescaler; //3 -- 1Msps
  #ifdef CAN2_LOOPBACK
    hcan2.Init.Mode = CAN_MODE_LOOPBACK;
  #else
    hcan2.Init.Mode = CAN_MODE_NORMAL;
  #endif
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_6TQ;
  hcan2.Init.BS2 = CAN_BS2_5TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan2);

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 14;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    // Filter configuration Error
    Error_Handler();
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

#define LEDG_PIN GPIO_PIN_7
#define LEDG_GPIO GPIOA

#define LEDB_PIN GPIO_PIN_6
#define LEDB_GPIO GPIOA

#define LEDR_PIN GPIO_PIN_5
#define LEDR_GPIO GPIOA

/* USER CODE BEGIN 4 */

void User_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = LEDG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(LEDG_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEDB_PIN;
  HAL_GPIO_Init(LEDB_GPIO, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = LEDR_PIN;
  HAL_GPIO_Init(LEDR_GPIO, &GPIO_InitStruct);
}

void LedG(uint8_t On)
{
  HAL_GPIO_WritePin(LEDG_GPIO, LEDG_PIN, On);
}

void LedG_Toggle()
{
  HAL_GPIO_TogglePin(LEDG_GPIO, LEDG_PIN);
}

void LedB(uint8_t On)
{
  HAL_GPIO_WritePin(LEDB_GPIO, LEDB_PIN, On);
}

void LedB_Toggle()
{
  HAL_GPIO_TogglePin(LEDB_GPIO, LEDB_PIN);
}

void LedR(uint8_t On)
{
  HAL_GPIO_WritePin(LEDR_GPIO, LEDR_PIN, On);
}

void LedR_Toggle()
{
  HAL_GPIO_TogglePin(LEDR_GPIO, LEDR_PIN);
}



void Error_Handler(void)
{
  LedR(1);
  while (1) ;
}

void CAN_CancelTransmit(CAN_HandleTypeDef* hcan)
{
  __HAL_CAN_CANCEL_TRANSMIT(hcan, CAN_TXMAILBOX_0);
  __HAL_CAN_CANCEL_TRANSMIT(hcan, CAN_TXMAILBOX_1);
  __HAL_CAN_CANCEL_TRANSMIT(hcan, CAN_TXMAILBOX_2); 
  __HAL_CAN_DISABLE_IT(hcan, CAN_IT_TME);
}


/**
  * @brief  Transmission  complete callback in non blocking mode
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{
  uint8_t bAccept = 0;
  
  if (CanHandle == &hcan1)
  {
    // software packet filter
    if (hcan1.pRxMsg->IDE == CAN_ID_STD)
    {
      // standard ID
      if ((hcan1.pRxMsg->StdId & iCAN1_FilterMaskIdLow) == iCAN1_FilterIdLow) bAccept = 1;
    }
    else
    {
      // extended ID
      if ((hcan1.pRxMsg->ExtId & iCAN1_FilterMaskIdLow) == iCAN1_FilterIdLow) bAccept = 1;
    }
    
    if (bAccept)
    {
      // CAN1 reception
      LedG_Toggle();

      // copy all stuff from RX CAN1 to TX CAN2
      hcan2.pTxMsg->StdId = hcan1.pRxMsg->StdId;
      hcan2.pTxMsg->RTR = hcan1.pRxMsg->RTR;
      hcan2.pTxMsg->IDE = hcan1.pRxMsg->IDE;
      hcan2.pTxMsg->ExtId = hcan1.pRxMsg->ExtId;
      hcan2.pTxMsg->DLC = hcan1.pRxMsg->DLC;
      hcan2.pTxMsg->Data[0] = hcan1.pRxMsg->Data[0];
      hcan2.pTxMsg->Data[1] = hcan1.pRxMsg->Data[1];
      hcan2.pTxMsg->Data[2] = hcan1.pRxMsg->Data[2];
      hcan2.pTxMsg->Data[3] = hcan1.pRxMsg->Data[3];
      hcan2.pTxMsg->Data[4] = hcan1.pRxMsg->Data[4];
      hcan2.pTxMsg->Data[5] = hcan1.pRxMsg->Data[5];
      hcan2.pTxMsg->Data[6] = hcan1.pRxMsg->Data[6];
      hcan2.pTxMsg->Data[7] = hcan1.pRxMsg->Data[7];
      
      bCAN2_TxReq = 1;  // requesting transmission for CAN2
    }
  }
  else
  {
    // software packet filter
    if (hcan2.pRxMsg->IDE == CAN_ID_STD)
    {
      // standard ID
      if ((hcan2.pRxMsg->StdId & iCAN2_FilterMaskIdLow) == iCAN2_FilterIdLow) bAccept = 1;
    }
    else
    {
      // extended ID
      if ((hcan2.pRxMsg->ExtId & iCAN2_FilterMaskIdLow) == iCAN2_FilterIdLow) bAccept = 1;
    }
    
    if (bAccept)
    {
      // CAN2 reception
      LedB_Toggle(); 
      
      hcan1.pTxMsg->StdId = hcan2.pRxMsg->StdId;
      hcan1.pTxMsg->RTR = hcan2.pRxMsg->RTR;
      hcan1.pTxMsg->IDE = hcan2.pRxMsg->IDE;
      hcan1.pTxMsg->ExtId = hcan2.pRxMsg->ExtId;
      hcan1.pTxMsg->DLC = hcan2.pRxMsg->DLC;
      hcan1.pTxMsg->Data[0] = hcan2.pRxMsg->Data[0];
      hcan1.pTxMsg->Data[1] = hcan2.pRxMsg->Data[1];
      hcan1.pTxMsg->Data[2] = hcan2.pRxMsg->Data[2];
      hcan1.pTxMsg->Data[3] = hcan2.pRxMsg->Data[3];
      hcan1.pTxMsg->Data[4] = hcan2.pRxMsg->Data[4];
      hcan1.pTxMsg->Data[5] = hcan2.pRxMsg->Data[5];
      hcan1.pTxMsg->Data[6] = hcan2.pRxMsg->Data[6];
      hcan1.pTxMsg->Data[7] = hcan2.pRxMsg->Data[7];
      
      bCAN1_TxReq = 1; // requesting transmission for CAN1
    }
  }
  

  /* Resume receive */
  __HAL_UNLOCK(CanHandle); // in case we arrived there from transmission
  if (HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
}

//----------------------------------------------------------------------------------------
// Serial UART over USB configuration code section (communication protocol FSM)
//----------------------------------------------------------------------------------------
typedef enum
{
  UART_STATE_IDLE, // waiting for command
  UART_STATE_SYNC_1, // receiving sync byte 1
  UART_STATE_SYNC_2, // receiving sync byte 2
  UART_STATE_ADDR_0, // receiving addr byte 0
  UART_STATE_ADDR_1, // receiving addr byte 1
  UART_STATE_ADDR_2, // receiving addr byte 1
  UART_STATE_ADDR_3, // receiving addr byte 1
  UART_STATE_LEN_0, // receiving length byte 2
  UART_STATE_LEN_1, // receiving sync byte 2
  UART_STATE_CMD, // command byte
  UART_STATE_WRITE_0, // receive write byte 0
  UART_STATE_WRITE_1, // receive write byte 0
  UART_STATE_WRITE_2, // receive write byte 0
  UART_STATE_WRITE_3, // receive write byte 0
} UART_RX_CMD_States;

UART_RX_CMD_States bUartRxState = UART_STATE_IDLE;
uint32_t iMemRWAddr = 0;
uint16_t iMemRWByteCount = 0;
uint8_t * bUARTMemTxBuff; // this is actually memory buffer for
uint8_t bUARTMemTxResponse[3] = {'1', '2', 'Y'};



// Variable used for Erase procedure
static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PAGEError = 0;
uint32_t iFlashData = 0;

#include "usbd_cdc_if.h"

void UART_ProcessData(uint8_t rx)
{
  if (bUartRxState == UART_STATE_IDLE)
  {
    if (rx == '1')
      bUartRxState = UART_STATE_SYNC_1;
  }
  else
  if (bUartRxState == UART_STATE_SYNC_1)
  {
    if (rx == '2')
      bUartRxState = UART_STATE_ADDR_0;
    else
      bUartRxState = UART_STATE_IDLE;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_0)
  {
    iMemRWAddr = rx << 24;
    bUartRxState = UART_STATE_ADDR_1;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_1)
  {
    iMemRWAddr |= rx << 16;
    bUartRxState = UART_STATE_ADDR_2;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_2)
  {
    iMemRWAddr |= rx << 8;
    bUartRxState = UART_STATE_ADDR_3;
  }
  else
  if (bUartRxState == UART_STATE_ADDR_3)
  {
    iMemRWAddr |= rx << 0;
    bUartRxState = UART_STATE_LEN_0;
  }
  else
  if (bUartRxState == UART_STATE_LEN_0)
  {
    iMemRWByteCount = rx << 16;
    bUartRxState = UART_STATE_LEN_1;
  }
  else
  if (bUartRxState == UART_STATE_LEN_1)
  {
    iMemRWByteCount |= rx << 0;
    bUartRxState = UART_STATE_CMD;
  }
  else
  if (bUartRxState == UART_STATE_CMD)
  {
    if (rx == 'R')
    {
      bUartRxState = UART_STATE_IDLE;
      bUARTMemTxBuff = ((uint8_t*)(iMemRWAddr)); // construct pointer to memory from integer address
      CDC_Transmit_FS(bUARTMemTxBuff, iMemRWByteCount);
    }
    else
    if (rx == 'E')
    {
      bUartRxState = UART_STATE_IDLE;
      HAL_NVIC_SystemReset();
    }
    else
    if (rx == 'F')
    {
      // Unlock the Flash to enable the flash control register access 
      HAL_FLASH_Unlock();

      // Fill EraseInit structure
      EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress = iMemRWAddr;
      EraseInitStruct.NbPages     = 1; // we arasing only one page (teh last one)
      
      // Erase the user Flash area
      if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
      {
        LedR(1);
        bUARTMemTxResponse[2] = 'N';
      }
      else
        bUARTMemTxResponse[2] = 'Y';
      
      // transmit back response
      CDC_Transmit_FS(bUARTMemTxResponse, 3);
  
      bUartRxState = UART_STATE_IDLE;
    }
    else
    if (rx == 'W')
    {
      bUartRxState = UART_STATE_WRITE_0;
      
      // Unlock the Flash to enable the flash control register access 
      HAL_FLASH_Unlock();
    }
  }
  else
  if (bUartRxState == UART_STATE_WRITE_0)
  {
    iFlashData = (rx)&0xFF;
    bUartRxState = UART_STATE_WRITE_1;
  }
  else
  if (bUartRxState == UART_STATE_WRITE_1)
  {
    iFlashData |= ((rx)&0xFF) << 8;
    bUartRxState = UART_STATE_WRITE_2;
  }
  else
  if (bUartRxState == UART_STATE_WRITE_2)
  {
    iFlashData |= ((rx)&0xFF) << 16;
    bUartRxState = UART_STATE_WRITE_3;
  }
  else
  if (bUartRxState == UART_STATE_WRITE_3)
  {
    iFlashData |= ((rx)&0xFF) << 24;
    bUartRxState = UART_STATE_WRITE_3;
    // writing data
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, iMemRWAddr, iFlashData) != HAL_OK)
    {
      // here is error situation
      LedR(1);
      bUARTMemTxResponse[2] = 'N';
      
      // lock flash control
      HAL_FLASH_Lock();

      // transmit back response
      CDC_Transmit_FS(bUARTMemTxResponse, 3);
      
      bUartRxState = UART_STATE_IDLE;
    }
    
    // incrementing address
    iMemRWByteCount = iMemRWByteCount - 4;
    iMemRWAddr = iMemRWAddr + 4;
    
    if (iMemRWByteCount <= 0)
    {
      // programming finished
      bUARTMemTxResponse[2] = 'Y';
      
      // lock flash control
      HAL_FLASH_Lock();

      // transmit back response 
      CDC_Transmit_FS(bUARTMemTxResponse, 3);
      
      bUartRxState = UART_STATE_IDLE;
    }
    else
      bUartRxState = UART_STATE_WRITE_0;
    
  }
}

//----------------------------------------------------------------------------------------
// Simple Queue implementation for serial port

#define RX_QUEUE_LENGTH 64
static uint8_t bRxQueue[RX_QUEUE_LENGTH];
static int iRxQueueCount = 0;
static int iRxQueueIn = 0;
static int iRxQueueOut = 0;

void RxQueuePut(uint8_t data)
{
  if (iRxQueueCount < RX_QUEUE_LENGTH)
  {
    bRxQueue[iRxQueueIn] = data;
    iRxQueueIn = iRxQueueIn + 1;
    if (iRxQueueIn >= RX_QUEUE_LENGTH) iRxQueueIn = 0;
    iRxQueueCount++;
  }
}

uint8_t RxQueueGet()
{
  uint8_t data = 0;
  
  if (iRxQueueCount > 0)
  {
    data = bRxQueue[iRxQueueOut];
    iRxQueueOut = iRxQueueOut + 1;
    if (iRxQueueOut >= RX_QUEUE_LENGTH) iRxQueueOut = 0;
    iRxQueueCount--;
  }
  
  return data;
}

uint8_t RxQueueNotEmpty() { return iRxQueueCount > 0; }


//----------------------------------------------------------------------------------------
// CAN packets data modification procedure
//----------------------------------------------------------------------------------------

void ProcessModification(CanTxMsgTypeDef* pTxMsg)
{
  unsigned int *iID;
  int i;
  unsigned int *iBuffer;
  unsigned int *iDataLow;
  unsigned int *iDataHigh;
  
  
  if (pTxMsg->IDE == CAN_ID_STD)
    iID = &pTxMsg->StdId;
  else
    iID = &pTxMsg->ExtId;
  
  iDataLow = (unsigned int *)(&pTxMsg->Data);
  iDataHigh = (unsigned int *)(&pTxMsg->Data[4]);

  iBuffer = (unsigned int *)(FLASH_REPLACEMENT_OFFSET - FLASH_REPLACEMENT_SIZE*4); //-FLASH_REPLACEMENT_SIZE needed because we shift pointer at the beginning of the for loop 
  iReplace_IDMask = iBuffer; iBuffer++;
  iReplace_IDFilter = iBuffer; iBuffer++;
  iReplace_NewIDMask = iBuffer; iBuffer++;
  iReplace_NewIDValue = iBuffer; iBuffer++;
  iReplace_DataMaskHigh = iBuffer; iBuffer++;
  iReplace_DataMaskLow = iBuffer; iBuffer++;
  iReplace_DataFilterHigh = iBuffer; iBuffer++;
  iReplace_DataFilterLow = iBuffer; iBuffer++;
  iReplace_NewDataMaskHigh = iBuffer; iBuffer++;
  iReplace_NewDataMaskLow = iBuffer; iBuffer++;
  iReplace_NewDataValueHigh = iBuffer; iBuffer++;
  iReplace_NewDataValueLow = iBuffer; iBuffer++;
  
  for (i = 0; i < iReplace_Count; i++)
  {
    // increment pointers to move to next replacement record
    iReplace_IDMask += FLASH_REPLACEMENT_SIZE;
    iReplace_IDFilter += FLASH_REPLACEMENT_SIZE;
    iReplace_NewIDMask += FLASH_REPLACEMENT_SIZE;
    iReplace_NewIDValue += FLASH_REPLACEMENT_SIZE;
    iReplace_DataMaskHigh += FLASH_REPLACEMENT_SIZE;
    iReplace_DataMaskLow += FLASH_REPLACEMENT_SIZE;
    iReplace_DataFilterHigh += FLASH_REPLACEMENT_SIZE;
    iReplace_DataFilterLow += FLASH_REPLACEMENT_SIZE;
    iReplace_NewDataMaskHigh += FLASH_REPLACEMENT_SIZE;
    iReplace_NewDataMaskLow += FLASH_REPLACEMENT_SIZE;
    iReplace_NewDataValueHigh += FLASH_REPLACEMENT_SIZE;
    iReplace_NewDataValueLow += FLASH_REPLACEMENT_SIZE;
    
    // if bit in "ID Mask" =1 AND bit in recieved ID = bit in "ID Filter", then packed ID is accepted for modification
    // AND
    // if bit in "Data Mask" =1 AND bit in recieved ID = bit in "Data Filter", then packed data is accepted for modification
    
    if ((*iID & *iReplace_IDMask) != (*iReplace_IDFilter & *iReplace_IDMask)) continue;
    if ((*iDataLow & *iReplace_DataMaskLow) != (*iReplace_DataFilterLow & *iReplace_DataMaskLow)) continue;
    if ((*iDataHigh & *iReplace_DataMaskHigh) != (*iReplace_DataFilterHigh & *iReplace_DataMaskHigh)) continue;
     
    // here is data packed accepted by filter
    
    // if bit =1 in "New ID Mask" then the value of this bit is being replaced by bit from "New ID Value"
    *iID = (*iID & ~(*iReplace_NewIDMask)) | *iReplace_NewIDValue;
    
    // if bit =1 in "New Data Mask" then the value of this bit is being replaced by bit from "New Data Value"
    *iDataLow = (*iDataLow & ~(*iReplace_NewDataMaskLow)) | (*iReplace_NewDataValueLow & *iReplace_NewDataMaskLow);
    *iDataHigh = (*iDataHigh & ~(*iReplace_NewDataMaskHigh)) | (*iReplace_NewDataValueHigh & *iReplace_NewDataMaskHigh);
  }

}


//----------------------------------------------------------------------------------------
// Functional tests for transmission
//----------------------------------------------------------------------------------------
// type of CAN ID for tests
//#define TEST_STD // if not defined, then Extended ID used

// if defined CAN1 constantly transmits the data
//#define TEST_CAN1_TRANSMIT

// if defined CAN2 constantly transmits the data
//#define TEST_CAN2_TRANSMIT

// if defined then we waiting for received data and then checks the contents (used in TEST_CANX_TRANSMIT)
//#define TEST_LOOPBACK_CHECK

// incremented data and ID then masked to constrain the range
//#define TEST_TRANSMIT_MASK 0xF
#define TEST_TRANSMIT_MASK 0xFFFFFFFF

// if defined CAN1 transmits one packet every 1s
//#define TEST_CAN1_INJECTION

// if defined CAN2 transmits one packet every 1s
//#define TEST_CAN2_INJECTION

// if defined then no transmission
//#define TEST_LISTEN_ONLY

void RunTests()
{
  uint32_t tickstart = 0;
  CanTxMsgTypeDef *pTxMsg;
  CanRxMsgTypeDef *pRxMsg;
  CAN_HandleTypeDef *hcan;
  
  
  #ifdef TEST_LISTEN_ONLY
    while (1) if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet());
  #endif
  

  #if defined(TEST_CAN1_INJECTION) || defined(TEST_CAN2_INJECTION)
    // selecting buffer and CAN instance 
    #if defined(TEST_CAN1_INJECTION)
      pTxMsg = hcan1.pTxMsg;
      hcan = &hcan1;
    #else        
      pTxMsg = hcan2.pTxMsg;
      hcan = &hcan2;
    #endif
  
    pTxMsg->StdId = 0x7EA;
    pTxMsg->ExtId = 0x7EA;
    pTxMsg->IDE = CAN_ID_STD;
    pTxMsg->DLC = 8;
    pTxMsg->Data[7] = 0x44;
    pTxMsg->Data[6] = 0x54;
    pTxMsg->Data[5] = 0x4A;
    pTxMsg->Data[4] = 0x01;
    pTxMsg->Data[3] = 0x02;
    pTxMsg->Data[2] = 0x49;
    pTxMsg->Data[1] = 0x14;
    pTxMsg->Data[0] = 0x10;
  
    #ifdef TEST_STD
      pTxMsg->IDE = CAN_ID_STD;
    #else
      pTxMsg->IDE = CAN_ID_EXT;
    #endif
  
    while (1)
    {
      if (HAL_CAN_Transmit_IT(hcan) != HAL_OK)
      {
        LedR_Toggle();
      }
      
      // waiting one second
      tickstart = HAL_GetTick();
      while((HAL_GetTick() - tickstart) < 1000)
      {
        if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet());
      }
    }
  #endif

  #if defined(TEST_CAN1_TRANSMIT) || defined(TEST_CAN2_TRANSMIT)
    // selecting buffer and CAN instance 
    #if defined(TEST_CAN1_TRANSMIT)
      pTxMsg = hcan1.pTxMsg;
      pRxMsg = hcan1.pRxMsg;
      hcan = &hcan1;
    #else        
      pTxMsg = hcan2.pTxMsg;
      pRxMsg = hcan2.pRxMsg;
      hcan = &hcan2;
    #endif
    
    pTxMsg->StdId = 0;
    pTxMsg->ExtId = 1;
    pTxMsg->IDE = CAN_ID_STD;
    pTxMsg->DLC = 8;
    pTxMsg->Data[0] = 0;
    pTxMsg->Data[1] = 1;
    pTxMsg->Data[2] = 2;
    pTxMsg->Data[3] = 3;
    pTxMsg->Data[4] = 4;
    pTxMsg->Data[5] = 5;
    pTxMsg->Data[6] = 6;
    pTxMsg->Data[7] = 7;
    
    #ifdef TEST_STD
      pTxMsg->IDE = CAN_ID_STD;
    #else
      pTxMsg->IDE = CAN_ID_EXT;
    #endif
    
    while (1)
    {
      #if defined(TEST_CAN1_TRANSMIT)
        bCAN2_TxReq = 0; 
      #else
        bCAN1_TxReq = 0; 
      #endif
      
      // sending
      //if (HAL_CAN_Transmit_IT(hcan) != HAL_OK)
      if (HAL_CAN_Transmit(hcan, 10) != HAL_OK)
      {
        LedR_Toggle();
      }
      
      #ifdef TEST_LOOPBACK_CHECK
        // waiting for reception
        #if defined(TEST_CAN1_TRANSMIT)
          while (bCAN2_TxReq == 0) 
        #else
          while (bCAN1_TxReq == 0) 
        #endif
        { 
          if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet()); 
        };
        
        #ifdef TEST_STD
          if (pTxMsg->StdId != pRxMsg->StdId) LedR(1);
        #else
          if (pTxMsg->ExtId != pRxMsg->ExtId) LedR(1);
        #endif
        
        if (pTxMsg->Data[0] != pRxMsg->Data[0]) LedR(1);
        if (pTxMsg->Data[1] != pRxMsg->Data[1]) LedR(1);
        if (pTxMsg->Data[2] != pRxMsg->Data[2]) LedR(1);
        if (pTxMsg->Data[3] != pRxMsg->Data[3]) LedR(1);
        if (pTxMsg->Data[4] != pRxMsg->Data[4]) LedR(1);
        if (pTxMsg->Data[5] != pRxMsg->Data[5]) LedR(1);
        if (pTxMsg->Data[6] != pRxMsg->Data[6]) LedR(1);
        if (pTxMsg->Data[7] != pRxMsg->Data[7]) LedR(1);
      #endif
        
      pTxMsg->StdId++;  pTxMsg->StdId &= ((1 << 11)-1);
      pTxMsg->ExtId++;  pTxMsg->ExtId &= ((1 << 24)-1);
      pTxMsg->Data[0]++;
      pTxMsg->Data[1]++;
      pTxMsg->Data[2]++;
      pTxMsg->Data[3]++;
      pTxMsg->Data[4]++;
      pTxMsg->Data[5]++;
      pTxMsg->Data[6]++;
      pTxMsg->Data[7]++;

      pTxMsg->StdId &= TEST_TRANSMIT_MASK;
      pTxMsg->ExtId &= TEST_TRANSMIT_MASK;
//if (pTxMsg->StdId & 0x8) pTxMsg->StdId |= 0x100; if (pTxMsg->ExtId & 0x8) pTxMsg->ExtId |= 0x100;
      pTxMsg->Data[0] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[1] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[2] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[3] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[4] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[5] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[6] &= TEST_TRANSMIT_MASK;
      pTxMsg->Data[7] &= TEST_TRANSMIT_MASK;
      
      if (RxQueueNotEmpty()) UART_ProcessData(RxQueueGet()); 
    }
  #endif
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
