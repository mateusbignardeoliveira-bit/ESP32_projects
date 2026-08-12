#ifndef TOF200F_H
#define TOF200F_H

#include <Arduino.h>

#include "TCA.h"

#include "Adafruit_VL53L0X.h"



class TOF200F
{

private:

    TCA& tca;


    uint8_t canal;


    Adafruit_VL53L0X lox;



    // Calibração
    const int OFFSET_CORRECAO_MM = -20;



    // Filtro média móvel
    static const int NUM_LEITURAS = 10;

    int leituras[NUM_LEITURAS];

    int indiceLeitura;

    long totalSoma;



    int distanciaAtual;



public:


    // Recebe o TCA que controla o sensor
    TOF200F(TCA& multiplexador);



    // Inicializa o sensor
    bool begin(uint8_t canalTCA);



    // Atualiza a leitura
    void update();



    // Retorna distância filtrada em mm
    int getDistance();



};



#endif