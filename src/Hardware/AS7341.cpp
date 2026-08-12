#include "AS7341.h"



AS7341::AS7341(TCA& multiplexador) : tca(multiplexador)
{

    canal = 0;


    dados =
    {
        0,0,0,0,
        0,0,0,0,
        0,0
    };

}




void AS7341::begin(uint8_t canalTCA)
{

    canal = canalTCA;


    // Seleciona o sensor correto no TCA
    tca.selecionarCanal(canal);



    // Inicialização da biblioteca Waveshare

    AS7341_Init(eSpm);



    // Configurações vindas do seu código funcional

    AS7341_ATIME_config(100);

    AS7341_ASTEP_config(999);

    AS7341_AGAIN_config(6);



    // Mantém o LED interno habilitado inicialmente

    AS7341_EnableLED(true);

}





void AS7341::update()
{

    // Sempre garante que o sensor correto está selecionado

    tca.selecionarCanal(canal);



    sModeOneData_t data1;

    sModeTwoData_t data2;



    // Primeira parte da leitura

    AS7341_startMeasure(eF1F4ClearNIR);


    data1 = AS7341_ReadSpectralDataOne();



    dados.F1 = data1.channel1;

    dados.F2 = data1.channel2;

    dados.F3 = data1.channel3;

    dados.F4 = data1.channel4;



    // Segunda parte da leitura

    AS7341_startMeasure(eF5F8ClearNIR);


    data2 = AS7341_ReadSpectralDataTwo();



    dados.F5 = data2.channel5;

    dados.F6 = data2.channel6;

    dados.F7 = data2.channel7;

    dados.F8 = data2.channel8;



    dados.clear = data2.CLEAR;

    dados.nir = data2.NIR;

}





AS7341Data AS7341::getData()
{

    return dados;

}