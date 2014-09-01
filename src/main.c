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
        unsigned char thermoTam=0,thermoVirgul=0;
        int16_t accelX=0, accelY=0, accelZ=0;
        int16_t gyroX=0, gyroY=0, gyroZ=0;
        int16_t magX=0, magY=0, magZ=0;
        double pitch=0,roll=0;




        GyroOku(&gyroX,&gyroY,&gyroZ);
        sprintf(str, "gyrx: %d , gyroy: %d , gyroz: %d , \r\n", gyroX,gyroY,gyroZ);
        USART_puts(USART2, str);

        AccelOku(&accelX,&accelY,&accelZ);
        sprintf(str, "accelx: %d , accely: %d , accelz: %d , \r\n", accelX,accelY,accelZ);
        USART_puts(USART2, str);

        MagnetOku(&magX,&magY,&magZ);
        float pusulaDerece=atan2(magY,magX) * 180 / PI;
		pusulaDerece = round(pusulaDerece);
		pusulaDerece = EsasOlcu(pusulaDerece);
		pusulaDerece -= 63; //kalibre deðeri
		pusulaDerece = EsasOlcu(pusulaDerece);



        sprintf(str, "pusula: %d MagX: %d , MagY: %d , MagZ: %d , \r\n", (int)pusulaDerece, magX,magY,magZ);
        USART_puts(USART2, str);


        EulerHesapla(accelX,accelY,accelZ,&pitch,&roll);
        sprintf(str, "pitch: %d , roll: %d , theta: %d \r\n", (int)pitch,(int)roll,(int)pusulaDerece);
        USART_puts(USART2, str);

        ThermoOku(&thermoTam,&thermoVirgul);
        sprintf(str,"sicaklik: %d,%d'C \r\n",thermoTam,thermoVirgul);
        USART_puts(USART2, str);



        GPIO_ToggleBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);


        MyDelay = 2;
        while (MyDelay) {}
    }
}


