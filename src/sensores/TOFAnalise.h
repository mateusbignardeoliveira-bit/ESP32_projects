#ifndef TOF_ANALISE_H
#define TOF_ANALISE_H

#include <Arduino.h>

#include "../Hardware/TOF200F.h"


class TOFAnalise
{

private:

    TOF200F& tof;


    // ========================================================
    // ESTADO
    // ========================================================

    int distancia;

    bool valido;

    bool obstaculo;


    // ========================================================
    // HISTERese
    // ========================================================

    // Confirma obstáculo somente realmente perto.
    static constexpr int DISTANCIA_CONFIRMACAO_MM = 80;


    // Para liberar precisa estar claramente longe.
    static constexpr int DISTANCIA_LIBERACAO_MM = 120;


    // ========================================================
    // CONFIRMAÇÃO
    // ========================================================

    static constexpr int LEITURAS_PARA_CONFIRMAR = 5;

    static constexpr int LEITURAS_PARA_LIBERAR = 3;


    int leiturasBaixas;

    int leiturasAltas;


public:

    TOFAnalise(
        TOF200F& sensor
    );


    void update();


    int getDistancia();


    bool isValido() const;


    bool temObstaculo() const;


    int getLimiteObstaculo();


    int getLeiturasBaixas() const;


    int getLeiturasAltas() const;


    void reset();

};

#endif