#ifndef TOF_ANALISE_H
#define TOF_ANALISE_H

#include <Arduino.h>

#include "../hardware/TOF200F.h"


class TOFAnalise
{

private:

    TOF200F& tof;


    // --------------------------------------------------------
    // Estado
    // --------------------------------------------------------

    int distancia;

    bool valido;

    bool obstaculo;


    // --------------------------------------------------------
    // HISTERese
    // --------------------------------------------------------

    // Para confirmar obstáculo, a leitura precisa estar
    // realmente próxima.
    static constexpr int DISTANCIA_CONFIRMACAO_MM = 80;


    // Depois de confirmado, só consideramos que o obstáculo
    // desapareceu quando a distância estiver claramente maior.
    static constexpr int DISTANCIA_LIBERACAO_MM = 120;


    // Quantidade de leituras consecutivas para confirmar.
    static constexpr int LEITURAS_PARA_CONFIRMAR = 5;


    // Quantidade de leituras consecutivas para liberar.
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


    // Retorna o limite principal usado na detecção
    int getLimiteObstaculo();


    // Quantidade atual de leituras próximas
    int getLeiturasBaixas() const;

};

#endif