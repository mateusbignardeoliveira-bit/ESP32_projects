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


    // ========================================================
    // FILTRO
    // ========================================================

    static constexpr int NUM_LEITURAS = 3;

    int leituras[NUM_LEITURAS];

    int indiceLeitura;

    int totalSoma;

    int quantidadeLeituras;

    int distanciaAtual;


    // ========================================================
    // ÚLTIMA LEITURA
    // ========================================================

    int ultimaLeitura;

    bool novaLeitura;

    bool ultimaLeituraValida;


    // ========================================================
    // CONTROLE DE RECUPERAÇÃO
    // ========================================================

    int leiturasInvalidas;

    static constexpr int LIMITE_INVALIDAS_REINICIO = 10;


    // ========================================================
    // CALIBRAÇÃO
    // ========================================================

    static constexpr int OFFSET_CORRECAO_MM = -20;


    // ========================================================
    // CONFIGURAÇÃO
    // ========================================================

    static constexpr uint16_t PERIODO_MEDICAO_MS = 25;


    void limparFiltro();

    bool reiniciarMedicao();


public:

    TOF200F(
        TCA& multiplexador
    );


    bool begin(
        uint8_t canalTCA
    );


    // Atualiza o sensor sem bloquear
    void update();


    // Distância filtrada
    int getDistance();


    // Última leitura individual calibrada
    int getUltimaLeitura();


    // Existe uma nova medição disponível?
    bool temNovaLeitura();


    // A última medição foi válida?
    bool ultimaLeituraValidaAgora();


    // Reinicia completamente o estado lógico do sensor
    void reset();

};

#endif