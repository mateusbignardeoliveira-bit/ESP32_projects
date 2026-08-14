#ifndef CONTROLE_SEGUE_LINHA_H
#define CONTROLE_SEGUE_LINHA_H

#include <Arduino.h>

#include "../sensores/LinhaAnalise.h"


struct ControleData
{
    float proporcional;
    float integral;
    float derivativo;

    float correcao;

    int velocidadeBaseAtual;

    int velocidadeEsquerda;
    int velocidadeDireita;
};


class ControleSegueLinha
{

private:

    // ========================================================
    // PID
    // ========================================================

    float Kp;
    float Ki;
    float Kd;


    // ========================================================
    // MEMÓRIA PID
    // ========================================================

    float erroAnterior;

    float integral;


    // ========================================================
    // TEMPO
    // ========================================================

    unsigned long tempoAnterior;


    // ========================================================
    // VELOCIDADE
    // ========================================================

    int velocidadeBase;

    int velocidadeMaxima;


    // ========================================================
    // VELOCIDADE ADAPTATIVA
    // ========================================================

    float erroVelocidadeMinimo;

    float erroVelocidadeMaximo;


    int velocidadeMinimaCurva;


    // ========================================================
    // RESULTADO
    // ========================================================

    ControleData resultado;


    // ========================================================
    // CALCULA VELOCIDADE BASE
    // ========================================================

    int calcularVelocidadeBase(
        float erro
    );


public:

    ControleSegueLinha();


    void configurarPID(
        float kp,
        float ki,
        float kd
    );


    void configurarVelocidade(
        int base,
        int maxima
    );


    void configurarVelocidadeAdaptativa(
        float erroMinimo,
        float erroMaximo,
        int velocidadeMinima
    );


    void reset();


    void update(
        const LinhaData& linha
    );


    ControleData getData();

};

#endif