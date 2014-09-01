#include <stdio.h>
#include <math.h>
#include <stm32f4xx.h>
//#include "stm32f4xx.h"
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_flash.h>
#include <stm32f4xx_usart.h>
#include <misc.h>

#include "MPU6050.h"




#define MPU9150_RA_MAG_ADDRESS 0x0C
#define MPU6050_MAG_ADDRESS     (MPU9150_RA_MAG_ADDRESS<<1)
#define MPU9150_RA_MAG_XOUT_L 0x03
#define MPU9150_RA_MAG_XOUT_H 0x04
#define MPU9150_RA_MAG_YOUT_L 0x05
#define MPU9150_RA_MAG_YOUT_H 0x06
#define MPU9150_RA_MAG_ZOUT_L 0x07
#define MPU9150_RA_MAG_ZOUT_H 0x08



uint8_t i2c_adresi=MPU6050_DEFAULT_ADDRESS;

volatile unsigned int sistemMiliSaati=0;


char str[64];

volatile short MyDelay;

void SysTick_Handler(void);
void _delay(unsigned int value);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void init_usart(void);
void initGPIO();



void MPU9150_setupCompass(void);
void MPU9150_writeSensor(uint8_t regAddr,uint8_t data);



#define PI 3.14159265f


volatile unsigned int i2c_kontrol_sayaci=0;


void EulerHesapla(int16_t x, int16_t y, int16_t z, double *pitch, double *roll);

void AccelOku(int16_t *x, int16_t *y, int16_t *z);
void GyroOku(int16_t *x, int16_t *y, int16_t *z);
void MagnetOku(int16_t *x, int16_t *y, int16_t *z);
void ThermoOku(unsigned char *sicaklikTam,unsigned char *sicaklikVirgul);

float EsasOlcu(float position);

void SysTick_Handler(void)
{
    if (MyDelay)
    {
        MyDelay--;
    }

    //sistemMiliSaati++;


    // SysTick_Config(SysTick_LOAD_RELOAD_Pos);
    // STK_CTRL&=~0x00010000;//count flag slincek
}

void Timer7Init(void);
void TIM7_IRQHandler (void);
void I2C_Kilitlenmesi(void);

void _delay(unsigned int value)
{
    while (value>0)
    {
        value--;
    }
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s)
{

    while (*s)
    {
        // wait until data register is empty
        while (!(USARTx->SR & 0x00000040))
            ;
        USART_SendData(USARTx, *s);
        *s++;
    }
}


void init_usart(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* enable peripheral clock for USART2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* GPIOA Configuration:  USART2 TX on PA2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART2 pins to AF2 */
    // TX = PA2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE); // enable USART2
}

void initGPIO()
{

    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}



void AccelOku(int16_t *x, int16_t *y, int16_t *z)
{
    u8 tmpBuffer[6];

    MPU6050_I2C_BufferRead(0x68 << 1, tmpBuffer, 0x3B, 6); // ACCELEROMETER
    s16 AccXH = tmpBuffer[0];
    s16 AccXL = tmpBuffer[1];
    s16 AccYH = tmpBuffer[2];
    s16 AccYL = tmpBuffer[3];
    s16 AccZH = tmpBuffer[4];
    s16 AccZL = tmpBuffer[5];

    *x = ((s16) ((u16) AccXH << 8) + AccXL);
    *y = ((s16) ((u16) AccYH << 8) + AccYL);
    *z = ((s16) ((u16) AccZH << 8) + AccZL);
}

void GyroOku(int16_t *x, int16_t *y, int16_t *z)
{
    u8 tmpBuffer[6];

    MPU6050_I2C_BufferRead(0x68 << 1, tmpBuffer, 0x43, 6); // GYRO

    s16 GyroXH = tmpBuffer[0];
    s16 GyroXL = tmpBuffer[1];
    s16 GyroYH = tmpBuffer[2];
    s16 GyroYL = tmpBuffer[3];
    s16 GyroZH = tmpBuffer[4];
    s16 GyroZL = tmpBuffer[5];

    *x = ((s16) ((u16) GyroXH << 8) + GyroXL);
    *y = ((s16) ((u16) GyroYH << 8) + GyroYL);
    *z = ((s16) ((u16) GyroZH << 8) + GyroZL);

}
void MagnetOku(int16_t *x, int16_t *y, int16_t *z)
{
    u8 tmpBuffer[6];

    i2c_adresi=MPU6050_MAG_ADDRESS;

    MPU9150_writeSensor(0x0A, 0x01);//enable the magnetometer
    MyDelay = 10;
    while (MyDelay) {}

    //MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU9150_RA_MAG_XOUT_L, 6); // magneto

    MPU6050_I2C_BufferRead(MPU6050_MAG_ADDRESS, tmpBuffer, MPU9150_RA_MAG_XOUT_L, 6); // magneto
    i2c_adresi=MPU6050_DEFAULT_ADDRESS;

    //MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_EXT_SENS_DATA_00, 6); // magneto


    s16 MagXH = tmpBuffer[0];
    s16 MagXL = tmpBuffer[1];
    s16 MagYH = tmpBuffer[2];
    s16 MagYL = tmpBuffer[3];
    s16 MagZH = tmpBuffer[4];
    s16 MagZL = tmpBuffer[5];

    *x = ((s16) ((u16) MagXH << 8) + MagXL);
    *y = ((s16) ((u16) MagYH << 8) + MagYL);
    *z = ((s16) ((u16) MagZH << 8) + MagZL);
}

void ThermoOku(unsigned char *sicaklikTam,unsigned char *sicaklikVirgul)
{
    u8 tmpBuffer[2];
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_TEMP_OUT_H, 2);

    s16 sicakH=tmpBuffer[0];
    s16 sicakL=tmpBuffer[1];
    s16 sicaklik=((s16)((u16)sicakH << 8) + sicakL);

    //sicaklik=(sicaklik + 12412.0) / 340.0;
    //ref. man. sayfa 29: Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 35
    //sprintf(str,"sicaklik: %d \r\n",sicaklik);
    //bilgi: http://playground.arduino.cc/Main/MPU-9150

    float sicak=(sicaklik / 340.0) + 35;

    *sicaklikTam=(u16)sicak;
    *sicaklikVirgul=(sicak*100)-((u16)sicak*100); //virgul kisi iki basamaklı olmasi icin 100 ile carptik, 10 ile carpsak virgul tek basamakli olurdu

}


void EulerHesapla(int16_t x, int16_t y, int16_t z, double *pitch, double *roll)
{
    *pitch = x / sqrt((y*y) + (z*z));
    *pitch = atan(*pitch) * 180;
    *pitch = *pitch / PI;
    *pitch = round(*pitch);

    *roll = y / sqrt((x*x) + (z*z));
    *roll = atan(*roll) * 180;
    *roll = *roll / PI;
    *roll = round(*roll);
}

float EsasOlcu(float position){
	if (position > 360.00){
		position = position - 360.00;
	}
	else if (position < 0){
		position = 360 + position;
	}

	return position;
}





void MPU9150_setupCompass(void)
{
    /*
    i2c_adresi = MPU6050_MAG_ADDRESS;      //change Adress to Compass

    MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
    MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
    MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

    i2c_adresi = MPU6050_DEFAULT_ADDRESS; //change Adress to MPU

    MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
    MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
    MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
    MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
    MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
    MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
    MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
    MPU9150_writeSensor(0x64, 0x01); //overvride register
    MPU9150_writeSensor(0x67, 0x03); //set delay rate
    MPU9150_writeSensor(0x01, 0x80);

    MPU9150_writeSensor(0x34, 0x04); //set i2c slv4 delay
    MPU9150_writeSensor(0x64, 0x00); //override register
    MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
    MPU9150_writeSensor(0x64, 0x01); //override register
    MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
    MPU9150_writeSensor(0x34, 0x13); //disable slv4

    */


    MPU9150_writeSensor(MPU6050_RA_FIFO_EN, 0xFF); // fifonun slave 0,1 ve 2 bitlerini aktif ediyoruz
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG,1, 1); // bypass'i aktif ediyoruz(0x37 registerinin 1. bit=1 olacak)

    //compass init:

    //http://electronics.stackexchange.com/questions/95833/contacting-invensense-mpu9150-auxiliary-magnetometer
    MPU9150_writeSensor(0x6B,0x01); // Clock source is PLLGYROZ
    MPU9150_writeSensor(0x19,0x04); // Set SMPLRT_DIV to 0x04; this gives a 200 Hz sample rate when using the DLPF
    MPU9150_writeSensor(0x1A,0x03); // 42 Hz LPF
    MPU9150_writeSensor(0x6A,0x01); // Disable master mode and clear all signal paths
    MPU9150_writeSensor(0x1B,0x18); // Full gyro range
    MPU9150_writeSensor(0x1C,0x08); // 4G accel range
    MPU9150_writeSensor(0x37,0x32); // Interrupts pin stays high until cleared, cleared on any read, I2C bypass
    MPU9150_writeSensor(0x38,0x01); // Data ready interrupt enabled

}

void MPU9150_writeSensor(uint8_t regAddr,uint8_t data)
{
    int i=0;
    for(i=0; i<8; i++)
    {
        char sayi=!(((data)&(1<<i)) - (1<<i));
        MPU6050_WriteBit(i2c_adresi, regAddr,i, sayi);
    }
}


void Timer7Init(void)
{
    RCC->APB1ENR|=0x00000020;         // Timer7 CLK'u aktif edelim (84 Mhz)
    TIM7->CR1=0x0080;                      // Otomatik Reload
    TIM7->PSC =42000-1;                   // Prescaler deðerimiz 42000, Count frekansimiz = fCK_PSC / (Yuklenen Deger + 1) 84E6 / (42000) = 2000 Hz
    TIM7->ARR =1000;                        // Counter, Decimal 1000 olunca basa donsun 0.5 sn demek
    TIM7->DIER=0x0001;                     // Update Int enable
    NVIC->ISER[1] = 0X00800000;        // NVIC de Timer 7 interrupta izin verelim
    TIM7->CR1|=0x0001;
}




void TIM7_IRQHandler(void)
{

    if(i2c_kontrol_sayaci==3)
    {
        I2C_Kilitlenmesi();
        i2c_kontrol_sayaci=6;
    }
    else
    {
        i2c_kontrol_sayaci++;
    }
    TIM7->SR=0;                                        // Timer Int Flagý silelim
    //GPIOD->ODR= 0x0000F000;     // Ledler yansin

}

void I2C_Kilitlenmesi(void)
{
    USART_puts(USART2,"hata aldik \r\n");

    GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

    while (1)
    {
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
        _delay(0x00FFFFFF);
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        _delay(0x00FFFFFF);
    }
}

/*
Yön bulma:
		if ((acilar.theta >= 338 && acilar.theta <= 360) || (acilar.theta >= 0 && acilar.theta <= 22)) cout << " kuzey" << endl;
		else if (acilar.theta >= 23 && acilar.theta <= 67) cout << " kuzey bati" << endl;
		else if (acilar.theta >= 68 && acilar.theta <= 112) cout << " bati" << endl;
		else if (acilar.theta >= 113 && acilar.theta <= 157) cout << " guney bati" << endl;
		else if (acilar.theta >= 158 && acilar.theta <= 202) cout << " guney" << endl;
		else if (acilar.theta >= 203 && acilar.theta <= 248) cout << " guney dogu" << endl;
		else if (acilar.theta >= 249 && acilar.theta <= 292) cout << " dogu" << endl;
		else if (acilar.theta >= 293 && acilar.theta <= 337) cout << " kuzey dogu" << endl;


*/

