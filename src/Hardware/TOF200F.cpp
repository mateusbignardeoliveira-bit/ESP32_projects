#include "TOF200F.h"



TOF200F::TOF200F(TCA& multiplexador)
:
tca(multiplexador)
{

    canal = 0;


    indiceLeitura = 0;

    totalSoma = 0;

    distanciaAtual = 0;



    for(int i = 0; i < NUM_LEITURAS; i++)
    {
        leituras[i] = 0;
    }

}




bool TOF200F::begin(uint8_t canalTCA)
{

    canal = canalTCA;



    // Seleciona o canal do TCA
    tca.selecionarCanal(canal);



    // Inicializa VL53L0X

    if(!lox.begin(0x29))
    {
        return false;
    }



    // Mesmo ajuste do seu código original

    lox.setMeasurementTimingBudgetMicroSeconds(200000);



    return true;

}




void TOF200F::update()
{

    // Garante que estamos falando com o TOF

    tca.selecionarCanal(canal);



    VL53L0X_RangingMeasurementData_t medida;



    lox.rangingTest(&medida, false);



    if(medida.RangeStatus != 4)
    {


        int leituraBruta = medida.RangeMilliMeter;



        int leituraCalibrada = leituraBruta + OFFSET_CORRECAO_MM;



        if(leituraCalibrada < 0)
        {
            leituraCalibrada = 0;
        }



        // Remove leitura antiga
        totalSoma -= leituras[indiceLeitura];



        // Coloca nova leitura
        leituras[indiceLeitura] = leituraCalibrada;



        totalSoma += leituras[indiceLeitura];



        indiceLeitura++;



        if(indiceLeitura >= NUM_LEITURAS)
        {
            indiceLeitura = 0;
        }



        distanciaAtual = totalSoma / NUM_LEITURAS;

    }

}





int TOF200F::getDistance()
{
    return distanciaAtual;
}