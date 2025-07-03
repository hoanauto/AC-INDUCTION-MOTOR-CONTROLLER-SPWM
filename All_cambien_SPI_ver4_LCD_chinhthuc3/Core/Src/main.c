/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#define Len1   0
#define Len2   1
#define Len3   2






/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman.h"
#include "CLCD_I2C.h"
#include <stdio.h>  
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint32_t T1 = 0;
volatile uint32_t T2 = 0;
volatile uint32_t T3 = 0;
volatile uint32_t OverNgat = 0;
volatile int State = 0, state_xe =1;
volatile uint32_t T3_T1 = 0;
volatile double F = 0,T = 0;
volatile double tocdo = 0,vantocxe = 0;
volatile uint32_t  luuadc = 1100,tayga = 0, tinhhieului =0;
uint8_t bar1[8] = {0x1C,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1C};
uint8_t bar2[8] = {0x07,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x07};
uint8_t bar3[8] = {0x1F,0x1F,0x00,0x00,0x00,0x00,0x1F,0x1F};
uint8_t bar4[8] = {0x1E,0x1C,0x00,0x00,0x00,0x00,0x18,0x1C};
uint8_t bar5[8] = {0x0F,0x07,0x00,0x00,0x00,0x00,0x03,0x07};
uint8_t bar6[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F};
uint8_t bar7[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0F};
uint8_t bar8[8] = {0x1F,0x1F,0x00,0x00,0x00,0x00,0x00,0x00};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
CLCD_I2C_Name LCD1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t txData0 =0;
uint8_t txData1= 1;
uint8_t txData2= 2;
uint8_t txData4 =4;
uint8_t txData5= 5;
uint8_t txDataphanh = 6;
uint8_t txDatalui = 7;
uint32_t adc_sensortayga;
int  level_tayga=0;
const int stator_speed_table[4] = {550, 740, 1205 ,1430};// 3 phan tu nen so 3 nhung so index la 0,1,2
int new_leveltayga=0, want_tocdostator = 0, set_leveltayga = 0;
int i = 0;
void CLCD_I2C_WriteIntFixed(CLCD_I2C_Name* LCD, uint8_t x, uint8_t y, int value, uint8_t width) {
    char buffer[10];
    sprintf(buffer, "%*d", width, value); // width = s? ký t? c? d?nh, can ph?i
    CLCD_I2C_SetCursor(LCD, x, y);
    CLCD_I2C_WriteString(LCD, buffer);
}
void CreateCustomChars(CLCD_I2C_Name* LCD) {
    CLCD_I2C_CreateChar(LCD, 0, bar1);
    CLCD_I2C_CreateChar(LCD, 1, bar2);
    CLCD_I2C_CreateChar(LCD, 2, bar3);
    CLCD_I2C_CreateChar(LCD, 3, bar4);
    CLCD_I2C_CreateChar(LCD, 4, bar5);
    CLCD_I2C_CreateChar(LCD, 5, bar6);
    CLCD_I2C_CreateChar(LCD, 6, bar7);
    CLCD_I2C_CreateChar(LCD, 7, bar8);
}

void BigNum0(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 1);
    CLCD_I2C_WriteChar(LCD, 7);
    CLCD_I2C_WriteChar(LCD, 0);
    CLCD_I2C_SetCursor(LCD, col, row + 1);
    CLCD_I2C_WriteChar(LCD, 1);
    CLCD_I2C_WriteChar(LCD, 5);
    CLCD_I2C_WriteChar(LCD, 0);
}


void BigNum1(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, 0);
    CLCD_I2C_SetCursor(LCD, col, row +1);
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, 0);
}


void BigNum2(CLCD_I2C_Name* LCD, uint8_t col , uint8_t row) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 4); // bar5 (v? trí 4)
    CLCD_I2C_WriteChar(LCD, 2); // bar3 (v? trí 2)
    CLCD_I2C_WriteChar(LCD, 0); // bar1 (v? trí 0)
    CLCD_I2C_SetCursor(LCD, col, row+1);
    CLCD_I2C_WriteChar(LCD, 1); // bar2 (v? trí 1)
    CLCD_I2C_WriteChar(LCD, 5); // bar6 (v? trí 5)
    CLCD_I2C_WriteChar(LCD, 5); // bar6 (v? trí 5)
}

void BigNum3(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 4); // bar5
    CLCD_I2C_WriteChar(LCD, 2); // bar3
    CLCD_I2C_WriteChar(LCD, 0); // bar1
    CLCD_I2C_SetCursor(LCD, col, row+1);
    CLCD_I2C_WriteChar(LCD, 6); // bar7
    CLCD_I2C_WriteChar(LCD, 5); // bar6
    CLCD_I2C_WriteChar(LCD, 0); // bar1
}

void BigNum4(CLCD_I2C_Name* LCD, uint8_t col,uint8_t row ) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 1); // bar2
    CLCD_I2C_WriteChar(LCD, 5); // bar6
    CLCD_I2C_WriteChar(LCD, 0); // bar1
    CLCD_I2C_SetCursor(LCD, col, row+1);
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, 0); // bar1
}

void BigNum5(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row ) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 1); // bar2
    CLCD_I2C_WriteChar(LCD, 4); // bar5
    CLCD_I2C_WriteChar(LCD, 3); // bar4
    CLCD_I2C_SetCursor(LCD, col, row+1);
    CLCD_I2C_WriteChar(LCD, 6); // bar7
    CLCD_I2C_WriteChar(LCD, 5); // bar6
    CLCD_I2C_WriteChar(LCD, 0); // bar1
}

void BigNum6(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 1); // bar2
    CLCD_I2C_WriteChar(LCD, 4); // bar5
    CLCD_I2C_WriteChar(LCD, 3); // bar4
    CLCD_I2C_SetCursor(LCD, col, row+1);
    CLCD_I2C_WriteChar(LCD, 1); // bar2
    CLCD_I2C_WriteChar(LCD, 5); // bar6
    CLCD_I2C_WriteChar(LCD, 0); // bar1
}

void BigNum7(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row ) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 1); // bar2
    CLCD_I2C_WriteChar(LCD, 7); // bar8
    CLCD_I2C_WriteChar(LCD, 0); // bar1
    CLCD_I2C_SetCursor(LCD, col, row+1);
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, ' ');
    CLCD_I2C_WriteChar(LCD, 0); // bar1
}

void BigNum8(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row) {
    CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 1); // bar2
    CLCD_I2C_WriteChar(LCD, 4); // bar5
    CLCD_I2C_WriteChar(LCD, 0); // bar1
    CLCD_I2C_SetCursor(LCD, col, row+1);
    CLCD_I2C_WriteChar(LCD, 1); // bar2
    CLCD_I2C_WriteChar(LCD, 5); // bar6
    CLCD_I2C_WriteChar(LCD, 0); // bar1
}
void RPM(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row) {
		 CLCD_I2C_SetCursor(LCD, col, row);
    CLCD_I2C_WriteChar(LCD, 0); 
    CLCD_I2C_WriteChar(LCD, 0);
    CLCD_I2C_SetCursor(LCD, col+1, row-1);
    CLCD_I2C_WriteChar(LCD, 0); 
    CLCD_I2C_WriteChar(LCD, 0);  
	}

void THANHRPM(CLCD_I2C_Name* LCD, uint8_t col, uint8_t row, uint8_t level, uint8_t maxLevel) {
    // level: s? thanh hi?n t?i (tùy theo RPM)
    // maxLevel: t?ng s? thanh t?i da (ví d? 8 thanh)
    
    for(uint8_t i = 0; i < maxLevel; i++) {
        CLCD_I2C_SetCursor(LCD, col + i, row);
        if(i < level) {
            CLCD_I2C_WriteChar(LCD, 0);  // Thanh sáng (ký t? tùy ch?nh 0)
        } else {
            CLCD_I2C_WriteChar(LCD, ' '); // Kho?ng tr?ng (không sáng)
        }
    }
}
void printBigNumberAtRow(CLCD_I2C_Name* LCD, int num, uint8_t col, uint8_t row) {
    if(row > 2) row = 2; // d?m b?o có d? 2 dòng d? v? s? l?n
    switch(num) {
        case 0: BigNum0(LCD, col, row); break;
        case 1: BigNum1(LCD, col, row); break;
        case 2: BigNum2(LCD, col, row); break;
        case 3: BigNum3(LCD, col, row); break;
        case 4: BigNum4(LCD, col, row); break;
        case 5: BigNum5(LCD, col, row); break;
        case 6: BigNum6(LCD, col, row); break;
        case 7: BigNum7(LCD, col, row); break;
        case 8: BigNum8(LCD, col, row); break;
        default:
            // Xóa vùng n?u s? ngoài ph?m vi
            CLCD_I2C_SetCursor(LCD, col, row);
            CLCD_I2C_WriteChar(LCD, ' ');
            CLCD_I2C_WriteChar(LCD, ' ');
            CLCD_I2C_WriteChar(LCD, ' ');
            CLCD_I2C_SetCursor(LCD, col, row+1);
            CLCD_I2C_WriteChar(LCD, ' ');
            CLCD_I2C_WriteChar(LCD, ' ');
            CLCD_I2C_WriteChar(LCD, ' ');
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	SimpleKalmanFilter(2,2,0.001f);
	CLCD_I2C_Init(&LCD1,&hi2c1,0x4e,20,4);
CreateCustomChars(&LCD1);
 CLCD_I2C_SetCursor(&LCD1, 0, 0);
 CLCD_I2C_WriteString(&LCD1, "DATN: EV DREAM 2025");
    CLCD_I2C_SetCursor(&LCD1, 0, 1);
    CLCD_I2C_WriteString(&LCD1, "SVTH: TRONG HOAN");
		CLCD_I2C_SetCursor(&LCD1, 6, 2);
    CLCD_I2C_WriteString(&LCD1, "DINH HIEN");
		CLCD_I2C_SetCursor(&LCD1, 0, 3);
		CLCD_I2C_WriteString(&LCD1, "GVHD: LE THANH PHUC");
    HAL_Delay(4000);  // Ð?i 3 giây d? d?c thông di?p

    CLCD_I2C_Clear(&LCD1); // Xóa màn hình chu?n b? hi?n th? s?
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		uint8_t maxBars =14;
		uint16_t maxRPM = 1500;
		// phanh
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)== GPIO_PIN_SET){ 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, &txDataphanh, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(200);
		if (T3_T1 ==0)
			{	tocdo = 0;}
		else if (T3_T1 !=0)
			{ F = 1/(0.000064*(T3_T1));
				tocdo = F*60;
				vantocxe = (tocdo*297)/62500;}
	if(OverNgat > 200)
			{T3_T1=0; 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			State = Len1;}
		else { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);}}
		
		tinhhieului = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		if ( tocdo ==0 && tinhhieului == 1)
	{ state_xe = 0;}
else if (tocdo ==0 && tinhhieului == 0)
	{ state_xe = 1;}
	else { state_xe = state_xe;}
	
	
if (state_xe ==1)
	{
		//  cam bien toc do
	if (T3_T1 ==0)
		{tocdo = 0;
		vantocxe = 0;}
	else if (T3_T1 !=0)
		{F = 1/(0.000064*(T3_T1));
		tocdo = F*60;
			vantocxe = (tocdo*297)/62500;}
		// dung xe
	 if(OverNgat > 200)
			{T3_T1=0; 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			State = Len1;}
		else { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);}
		// ràng tocdo doi tanso
				
		 if(tocdo >= 0 && tocdo<550)
			{i=0;
				set_leveltayga = 1;
			}
		else if(tocdo > 550 && tocdo <740)
			{i = 1;
				set_leveltayga =2;
			}
		else if(tocdo>740 && tocdo <1205)
			{i = 2;
				set_leveltayga = 3;
			}
			else if(tocdo>1205 && tocdo <1430)
			{i = 3;
				set_leveltayga = 4;
			}
			//cambientayga
				HAL_ADC_Start_DMA(&hadc1,&adc_sensortayga, 1);
			if (adc_sensortayga < 1100)
				{level_tayga = 0;}
		else if ( adc_sensortayga > 1100 && adc_sensortayga <1700){
				level_tayga = 1;
				want_tocdostator = 550;}
			else if( adc_sensortayga > 1700 && adc_sensortayga <2300){
				level_tayga = 2;
			want_tocdostator = 740;}
			else if( adc_sensortayga > 2300 && adc_sensortayga <2800){
				level_tayga = 3;
				want_tocdostator = 1205;}
			else if( adc_sensortayga > 2800){
				level_tayga = 4;
			want_tocdostator= 1430;}
			// su li tin hieu
			if(level_tayga ==0){
		new_leveltayga = 0;
			}
		else if(level_tayga !=0){
 if (level_tayga > set_leveltayga)
	{ new_leveltayga = set_leveltayga;
		
		if (((float)tocdo >= 0.8*(float)stator_speed_table[i])) // s bang 0.2
	{ new_leveltayga = new_leveltayga+1; // neu khong co i = i + 1 thi new_leveltayga se lien tuc tang
		i= i+1; 
	}else new_leveltayga =new_leveltayga;
	}
	else if( level_tayga == set_leveltayga)
	{ new_leveltayga = set_leveltayga;
			}
	else if(level_tayga < set_leveltayga)
	{ new_leveltayga =0;
		 if((float)tocdo < (float)0.85*want_tocdostator)
		{ new_leveltayga = level_tayga;}
			else{ new_leveltayga =0;}
		
		}
		}
  if(new_leveltayga ==0)
	{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, &txData0, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
	  else if(new_leveltayga ==1)
	{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, &txData1, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
		else if(new_leveltayga ==2)
		{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, &txData2, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
				else if(new_leveltayga ==3)
				{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, &txData4, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				}
	else if(new_leveltayga ==4)
				{ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, &txData5, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				}
}
	else if (state_xe ==0)
		{ if (T3_T1 ==0)
		{
			tocdo = 0;
		vantocxe= 0;}
		else if (T3_T1 !=0)
			{
  F = 1/(0.000064*(T3_T1));
		tocdo = F*60;
		vantocxe = (tocdo*297)/62500;	}
			
			if(OverNgat > 200)
			{T3_T1=0; 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			State = Len1;}
		else { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);}
			HAL_ADC_Start_DMA(&hadc1,&adc_sensortayga, 1);
				if (adc_sensortayga > 1100)
					{ 
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &txDatalui, 1,10);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
	HAL_Delay(200);
					}
		else { 
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, &txData0, 1,10);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
		HAL_Delay(200);}
		}
		if(tocdo > 0)
		{
		RPM(&LCD1,0,3);
		RPM(&LCD1,2,1);}
		else {CLCD_I2C_SetCursor(&LCD1, 0, 1);  
    CLCD_I2C_WriteString(&LCD1, "     ");
    CLCD_I2C_SetCursor(&LCD1, 0, 1);  
    CLCD_I2C_WriteString(&LCD1, "     ");  
    CLCD_I2C_SetCursor(&LCD1, 0, 3);  
    CLCD_I2C_WriteString(&LCD1, "     ");  
		CLCD_I2C_SetCursor(&LCD1, 0, 2);  
    CLCD_I2C_WriteString(&LCD1, "     ");  
    CLCD_I2C_SetCursor(&LCD1, 0, 0);  
    CLCD_I2C_WriteString(&LCD1, "     ");  }
 printBigNumberAtRow(&LCD1, vantocxe, 15, 2);  // Hi?n th? s? l?n ? c?t 0
CLCD_I2C_SetCursor(&LCD1, 5, 2);
		CLCD_I2C_WriteString(&LCD1, "GEAR");
CLCD_I2C_WriteIntFixed(&LCD1, 10, 2, new_leveltayga, 3);  // Ghi s? t?i v? trí c?t 3

// Hi?n th? RPM (4 ký t? s?)
CLCD_I2C_SetCursor(&LCD1, 5, 3);  // dòng th? 2, c?t 0
		CLCD_I2C_WriteString(&LCD1, "RPM");
CLCD_I2C_WriteIntFixed(&LCD1, 9, 3, tocdo, 4);  // Ghi t?i c?t 4, t?i da 4 ch? s?
        uint8_t level = (tocdo * maxBars) / maxRPM;
    if(level > maxBars) 
			level = maxBars; // gi?i h?n
    
    THANHRPM(&LCD1, 4, 0, level, maxBars);	
		HAL_Delay(150);




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
}}
 
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim3)
{
		if(State == Len1)
	{
		T1 = TIM3->CCR2;
		OverNgat = 0;
		State = Len2;
	}
	else if(State == Len2)
	{
		T2 = TIM3->CCR2;
	
		State = Len3;
	}
	else if(State == Len3)
	{
		T3 = TIM3->CCR2;
		T3_T1 = (T3 + (OverNgat* 256)) - T1;
		State = Len1;
		T1=0;
		T2=0;
		T3=0;
		
		}

	
	}
		

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim3)
{
	OverNgat++;


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_4;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4607;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA3 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS1_Pin */
  GPIO_InitStruct.Pin = SPI_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
