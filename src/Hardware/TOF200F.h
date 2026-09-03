#ifndef TOF200F_H
#define TOF200F_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

#include "TCA.h"


class TOF200F
{

private:

    TCA& tca;

    Adafruit_VL53L0X lox;

    uint8_t canal;


    // --------------------------------------------------------
    // Filtro da distância
    // --------------------------------------------------------

    static const int NUM_LEITURAS = 3;

    int leituras[NUM_LEITURAS];

    int indiceLeitura;

    int totalSoma;

    int quantidadeLeituras;

    int distanciaAtual;


    // --------------------------------------------------------
    // Última leitura individual
    // --------------------------------------------------------

    int ultimaLeitura;

    bool novaLeitura;

    bool ultimaLeituraValida;


    // --------------------------------------------------------
    // Correção de calibração
    // --------------------------------------------------------

    static constexpr int OFFSET_CORRECAO_MM = -20;


public:

    TOF200F(
        TCA& multiplexador
    );


    bool begin(
        uint8_t canalTCA
    );


    // Não bloqueante
    void update();


    // Distância filtrada
    int getDistance();


    // Última leitura individual calibrada
    int getUltimaLeitura();


    // Indica se uma nova medição foi recebida
    bool temNovaLeitura();


    // Indica se a última medição foi válida
    bool ultimaLeituraValidaAgora();

};

#endif