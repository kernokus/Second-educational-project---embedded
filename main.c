//step-driver+stm32+encoder Khoroshev,Sumarev,Bersenev
#include "stm32f0xx_gpio.h"
////////////////////////////////////////////////////////////////////
typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef signed short   SWORD;
////////////////////////////////////////////////////////////////////
void Timer1_Init()
{
  NVIC_InitTypeDef NVIC_InitStructure; //����������� ���������� ����������->��� ���������
  TIM_TimeBaseInitTypeDef TIM; // ����������� �������

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //��������� ������������ 

  /* Enable the TIM global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn; //�����, ���������� �� ����������
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;//��������� ���������� 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); //������������� , ��������� � ���������� NVIC(Nested vectored interrupt controller)

  TIM.TIM_Period = 100 - 1;   //Firq = Ftim / 100 = 10000 Hz
  TIM.TIM_Prescaler = 48 - 1; //Ftim = 48000000 / 48 = 1000000 Hz , �� ���� �������� 48 ���
  TIM.TIM_ClockDivision = 0;  
  TIM.TIM_CounterMode = TIM_CounterMode_Up;//������� �� ���������� 
  TIM.TIM_RepetitionCounter = 0; //�� ������������ counter ����������, ��������� ������� �� ������ n=0 ������������ ��������
  TIM_TimeBaseInit(TIM1, &TIM); //�������������� , ��������� � ���������� �������

  /* TIM IT enable */
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);//���������  ���������� ������� �� ������� TIM_IT_Update

  /* TIM counter enable */
  TIM_Cmd(TIM1, ENABLE); //��������� �������
}
////////////////////////////////////////////////////////////////////
BYTE  Enc_A_Prev = 1; //������������� ���������� �������� � � �
BYTE  Enc_B_Prev = 1;
BYTE  SD_State   = 0;
#define SD_DIS  GPIO_Pin_1 //PA1 EN
#define SD_DIR  GPIO_Pin_2 //PA2 DIR
#define SD_STEP GPIO_Pin_3 //PA3 STEP
////////////////////////////////////////////////////////////////////
BYTE Enc_Process()
{
  BYTE tmp;
  BYTE A = 1;
  BYTE B = 1;
  BYTE sd_CMD = 0; //nothing to do
  //PB1, PB2
  tmp = (GPIOB->IDR >> 1) & 0x03; //�������� ���� � � �������� ��� ����,����� 1 � 2.
  A = tmp & 0x01; //��������� 1 ���
  B = (tmp >> 1); //0 ���
/*
����� ������� ������ ������ ���� ������� �������� ��������� + ��������
*/
  if((Enc_B_Prev == 1) && (B == 0))
  {
    //1->0
    if(A == 1)
      sd_CMD = 1;//�� �������
    else
      sd_CMD = 2;//������ �������
  }
  if((Enc_B_Prev == 0) && (B == 1))
  {
    //1->0
    if(A == 1)
      sd_CMD = 2;//������ �������
    else
      sd_CMD = 1;//�� �������
  }
  Enc_A_Prev = A; //���������� ���������� �������� ����� return
  Enc_B_Prev = B;
  return sd_CMD; //�������� ��������������� ������� � ����������� �� ���������� ��������
}
////////////////////////////////////////////////////////////////////
/*�������� ��� ������� �� �������
void Task_1()
{
  if(SD_State == 0)
  {
    SD_State = 1;
    GPIOA->BSRR = SD_STEP; //BSRR - ��������� ���� 
  }
  else
  {
    SD_State = 0;
    GPIOA->BRR = SD_STEP;
  }
}
*/
////////////////////////////////////////////////////////////////////
void Task_2(BYTE CMD) //��������� ������� �� ��������, ��������� ��� ��������=��� ���������
{
  switch(SD_State)
  {
    case 0://wait CMD to step drive
      if(CMD == 1) //�� �������
      {
        GPIOA->BSRR = SD_DIR; //������� ������� ��������� ����� �� �������
        GPIOA->BSRR = SD_STEP; //��� ���
        SD_State = 1;
      }
      else if(CMD == 2) //������ �������
      {
        GPIOA->BRR = SD_DIR;//BRR-����� = ������� ������� ��������� ������ �������
        GPIOA->BSRR = SD_STEP;////��� ���
        SD_State = 1;
      }
      break;

    case 1: //��������� �� ���������
      SD_State = 0;
      GPIOA->BRR = SD_STEP;
      break;
  }
}
////////////////////////////////////////////////////////////////////
SWORD Task_3_pos = 0;
BYTE  Task_3_cnt = 0;
void Task_3(BYTE CMD)
{//������ if ���������� �����������
  if(CMD == 1) //�� �������
    Task_3_pos++; //��� � + �������
  else if(CMD == 2) //������ �������
    Task_3_pos--; //��� � + �������
//������ if ���������� ����������� �������� � ��� ��� ���� �����������
  if(Task_3_pos >= 0) 
  {
    GPIOA->BRR = SD_DIR; //������� ������ �������
    Task_3_cnt += Task_3_pos; //���������� ���������
  }
  else
  {
    GPIOA->BSRR = SD_DIR; //������� �� �������
    Task_3_cnt -= Task_3_pos;// ���������� ���������
  }

  if(Task_3_cnt & (1<<5))  //100000
    GPIOA->BSRR = SD_STEP;// �� �������
  else
    GPIOA->BRR = SD_STEP;//������ �������
}
////////////////////////////////////////////////////////////////////
void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
  if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
  {
    BYTE what_to_do = Enc_Process(); //�������� ��������������� ���������� ��������� ��������

    //Task_1();
    //Task_2(what_to_do);
    Task_3(what_to_do); //���������� ������ � �������� �� ������� ���������

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  }
}
////////////////////////////////////////////////////////////////////
int main()
{
  //�������� ������������ GPIO
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//push-pull
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����������� � 3.3 �
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  Timer1_Init();

  //GPIOA->BSRR = SD_DIS; //disable step drive
  GPIOA->BRR = SD_DIS;  //enable step drive

  GPIOA->BRR = SD_DIR;//task1

  /*while(1)
  {
    //GPIOA->BSRR = GPIO_Pin_1;
    //GPIOA->BRR = GPIO_Pin_1;
    //GPIO_SetBits(GPIOA, GPIO_Pin_1);
    //GPIO_ResetBits(GPIOA, GPIO_Pin_1);
  }
  */
}
