#ifndef CONTROLE_SEGUE_LINHA_H
#define CONTROLE_SEGUE_LINHA_H

#include <Arduino.h>
#include "../sensores/LinhaAnalise.h"


// ============================================================
// DADOS DO CONTROLADOR
// ============================================================

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


// ============================================================
// CLASSE
// ============================================================

class ControleSegueLinha
{

private:

    // ========================================================
    // GANHOS PID
    // ========================================================

    float Kp;
    float Ki;
    float Kd;


    // ========================================================
    // MEMÓRIA DO PID
    // ========================================================

    float erroAnterior;
    float integral;

    unsigned long tempoAnterior;


    // ========================================================
    // VELOCIDADES
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
    // MÉTODOS INTERNOS
    // ========================================================

    float limitar(
        float valor,
        float minimo,
        float maximo
    );


    int calcularVelocidadeBase(
        float erro
    );


public:

    // ========================================================
    // CONSTRUTOR
    // ========================================================

    ControleSegueLinha();


    // ========================================================
    // CONFIGURAÇÃO PID
    // ========================================================

    void configurarPID(
        float kp,
        float ki,
        float kd
    );


    // ========================================================
    // CONFIGURAÇÃO DE VELOCIDADE
    // ========================================================

    void configurarVelocidade(
        int base,
        int maxima
    );


    // ========================================================
    // CONFIGURAÇÃO DA VELOCIDADE ADAPTATIVA
    // ========================================================

    void configurarVelocidadeAdaptativa(
        float erroMinimo,
        float erroMaximo,
        int velocidadeMinima
    );


    // ========================================================
    // RESET DO CONTROLADOR
    // ========================================================

    void reset();


    // ========================================================
    // ATUALIZA O PID
    // ========================================================

    void update(
        const LinhaData& linha
    );


    // ========================================================
    // OBTÉM RESULTADO
    // ========================================================

    ControleData getData();

};

#endif