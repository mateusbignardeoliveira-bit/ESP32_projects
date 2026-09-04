#ifndef LINHA_ANALISE_H
#define LINHA_ANALISE_H

#include <Arduino.h>
#include "../hardware/ArrayLinha.h"


struct LinhaData
{
    bool linhaDetectada;

    float posicao;

    float erro;

    float intensidade;

    float largura;

    // Intensidade normalizada de cada sensor
    // 0.0 = branco
    // 1.0 = preto
    float sensores[8];
};


class LinhaAnalise
{

private:

    // ============================================================
    // CALIBRAÇÃO REALIZADA NO ROBÔ
    // ============================================================

    // Valores medidos com o array sobre o BRANCO
    const int branco[8] =
    {
        183,
        164,
        163,
        164,
        167,
        166,
        168,
        178
    };


    // Valores medidos com o array sobre o PRETO
    const int preto[8] =
    {
        3138,
        2448,
        2316,
        2223,
        2360,
        2682,
        2755,
        2995
    };


    // Posição física dos sensores
    // Negativo = esquerda
    // Positivo = direita
    const float pesos[8] =
    {
        -7.0f,
        -5.0f,
        -3.0f,
        -1.0f,
         1.0f,
         3.0f,
         5.0f,
         7.0f
    };


    // ============================================================
    // SENSIBILIDADE DA DETECÇÃO
    // ============================================================

    // A linha preta real que medimos entre S4 e S5 produziu
    // aproximadamente:
    //
    // S4 = 342 -> ~0.087 normalizado
    // S5 = 318 -> ~0.069 normalizado
    //
    // Portanto 0.05 permite detectar essa linha com margem.
    static constexpr float LIMIAR_DETECCAO = 0.05f;


    // Sensor considerado ativo para cálculo da largura.
    // Antes era 0.20f, o que era muito alto para a linha
    // preta real encontrada nos testes.
    static constexpr float LIMIAR_SENSOR_ATIVO = 0.05f;


    LinhaData resultado;


    float normalizarSensor(
        int valor,
        int indice
    );


public:

    LinhaAnalise();

    void update(
        const ArrayData& dados
    );

    LinhaData getData();
};

#endif