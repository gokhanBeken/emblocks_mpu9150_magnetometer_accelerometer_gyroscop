#include "main.h"


int main(void)
{
    SystemInit();

    initGPIO();
    init_usart();

    SysTick_Config(SystemCoreClock / 1000); //1milisaniyelik system clock elde edelim

    RCC_ClocksTypeDef RCC_ClockFreq;
    RCC_GetClocksFreq(&RCC_ClockFreq);

    Timer7Init();
    NVIC_DisableIRQ(TIM7_IRQn);

    MPU6050_I2C_Init();
    GPIO_SetBits(GPIOD, GPIO_Pin_15);
    MPU6050_Initialize();
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);

    if (MPU6050_TestConnection() == SUCCESS)
    {
        // connection success
    }

    MPU6050_SetFullScaleGyroRange(0);

    MPU9150_setupCompass();

    while (1)
    {

        aciOku();
        float pusulaDerece=atan2(hamVeri.magY,hamVeri.magX) * 180 / PI;

        sprintf(str, "gyrx: %d , gyroy: %d , gyroz: %d , \r\n", hamVeri.gyroX,hamVeri.gyroY,hamVeri.gyroZ);
        USART_puts(USART2, str);

        sprintf(str, "accelx: %d , accely: %d , accelz: %d , \r\n", hamVeri.accelX,hamVeri.accelY,hamVeri.accelZ);
    	USART_puts(USART2, str);

        sprintf(str, "pusula: %d MagX: %d , MagY: %d , MagZ: %d , \r\n", (int)pusulaDerece, hamVeri.magX,hamVeri.magY,hamVeri.magZ);
        USART_puts(USART2, str);

        EulerHesapla(hamVeri.accelX,hamVeri.accelY,hamVeri.accelZ);
        sprintf(str, "pitch: %d , roll: %d , theta: %d \r\n", (int)acilar.pitch,(int)acilar.roll,(int)pusulaDerece);
    	USART_puts(USART2, str);


        GPIO_ToggleBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);


        MyDelay = 2;
        while (MyDelay) {}
    }
}


