#ifndef CURVA_ANALISE_H
#define CURVA_ANALISE_H

#include <Arduino.h>

#include "../sensores/LinhaAnalise.h"


// ============================================================
// TENDÊNCIA DA LINHA
// ============================================================

enum TendenciaLinha
{
    TENDENCIA_NENHUMA,

    TENDENCIA_ESQUERDA,

    TENDENCIA_DIREITA
};


// ============================================================
// DADOS DA ANÁLISE
// ============================================================

struct CurvaData
{

    // --------------------------------------------------------
    // Quantidade total de sensores pretos
    // --------------------------------------------------------

    int sensoresPretos;


    // --------------------------------------------------------
    // Sensores pretos de cada lado
    // --------------------------------------------------------

    int sensoresEsquerda;

    int sensoresDireita;


    // --------------------------------------------------------
    // Centro da linha
    // --------------------------------------------------------

    bool linhaCentral;


    // --------------------------------------------------------
    // Linha ocupando vários sensores
    // --------------------------------------------------------

    bool linhaLarga;


    // --------------------------------------------------------
    // Todos os sensores brancos
    // --------------------------------------------------------

    bool todosBrancos;


    // --------------------------------------------------------
    // 4 ou mais sensores pretos
    //
    // Indica que deve começar a avaliação especial.
    // --------------------------------------------------------

    bool avaliacaoEspecial;


    // --------------------------------------------------------
    // Tendência antes da avaliação especial
    // --------------------------------------------------------

    TendenciaLinha tendenciaAntesAvaliacao;


    // --------------------------------------------------------
    // Quantidade de sensores de cada lado no momento
    // --------------------------------------------------------

    int maiorQuantidadeLado;


    // --------------------------------------------------------
    // Compatibilidade / informação adicional
    // --------------------------------------------------------

    bool cruzamento;

};


// ============================================================
// CLASSE
// ============================================================

class CurvaAnalise
{

private:

    // ========================================================
    // LIMIAR DE SENSOR PRETO
    // ========================================================

    const float LIMIAR_PRETO =
        0.35f;


    // ========================================================
    // LIMIAR PARA CONSIDERAR TENDÊNCIA
    // ========================================================

    const int MINIMO_SENSORES_TENDENCIA =
        2;


    // ========================================================
    // QUANTIDADE PARA AVALIAÇÃO ESPECIAL
    // ========================================================

    const int MINIMO_PRETOS_AVALIACAO =
        4;


    // ========================================================
    // SENSORES CENTRAIS
    //
    // 0 1 2 3 4 5 6 7
    //       ↑ ↑
    //       centro
    // ========================================================

    const int SENSOR_CENTRO_ESQUERDO =
        3;

    const int SENSOR_CENTRO_DIREITO =
        4;


    // ========================================================
    // DADOS
    // ========================================================

    CurvaData resultado;


    // ========================================================
    // MEMÓRIA DA TENDÊNCIA
    // ========================================================

    TendenciaLinha tendenciaAtual;

    TendenciaLinha tendenciaAnterior;


    // ========================================================
    // FUNÇÃO INTERNA
    // ========================================================

    TendenciaLinha calcularTendencia(
        int sensoresEsquerda,
        int sensoresDireita
    );


public:

    // ========================================================
    // CONSTRUTOR
    // ========================================================

    CurvaAnalise();


    // ========================================================
    // ATUALIZA
    // ========================================================

    void update(
        const LinhaData& linha
    );


    // ========================================================
    // DADOS
    // ========================================================

    CurvaData getData();


    // ========================================================
    // TENDÊNCIA
    // ========================================================

    TendenciaLinha getTendencia();


    TendenciaLinha getTendenciaAnterior();

};

#endif