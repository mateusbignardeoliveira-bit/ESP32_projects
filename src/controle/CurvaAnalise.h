#ifndef CURVA_ANALISE_H
#define CURVA_ANALISE_H

#include <Arduino.h>

#include "sensores/LinhaAnalise.h"


// ============================================================
// DIREÇÃO
// ============================================================

enum DirecaoCurva
{
    CURVA_NENHUMA,
    CURVA_ESQUERDA,
    CURVA_DIREITA
};


// ============================================================
// DADOS DA ANÁLISE DA REGIÃO DA LINHA
// ============================================================

struct CurvaData
{
    // --------------------------------------------------------
    // Quantidade de sensores pretos
    // --------------------------------------------------------

    int sensoresPretos;

    int sensoresEsquerda;

    int sensoresDireita;


    // --------------------------------------------------------
    // Características da linha
    // --------------------------------------------------------

    bool linhaCentral;

    bool linhaLarga;

    bool todosBrancos;


    // --------------------------------------------------------
    // Entrada na avaliação especial
    // --------------------------------------------------------

    bool avaliacaoEspecial;


    // --------------------------------------------------------
    // Compatibilidade temporária
    //
    // Estes campos ainda existem porque o Main antigo
    // utiliza CurvaData.
    //
    // Eles NÃO serão usados para decidir curvas.
    // Serão removidos quando o novo sistema de decisão
    // estiver integrado.
    // --------------------------------------------------------

    bool curva90;

    DirecaoCurva direcao;

    bool cruzamento;
};


// ============================================================
// CLASSE
// ============================================================

class CurvaAnalise
{
private:

    // --------------------------------------------------------
    // Sensor considerado preto
    // --------------------------------------------------------

    static constexpr float LIMIAR_PRETO = 0.35f;


    // --------------------------------------------------------
    // Quantidade de pretos para iniciar avaliação especial
    // --------------------------------------------------------

    static constexpr int MINIMO_PRETOS_AVALIACAO = 4;


    // --------------------------------------------------------
    // Sensores centrais
    //
    // Array:
    //
    // s1 s2 s3 s4 s5 s6 s7 s8
    //
    // Centro:
    // s4 e s5
    // --------------------------------------------------------

    static constexpr int SENSOR_CENTRO_ESQUERDO = 3;
    static constexpr int SENSOR_CENTRO_DIREITO = 4;


    // --------------------------------------------------------
    // Resultado atual
    // --------------------------------------------------------

    CurvaData resultado;


public:

    CurvaAnalise();


    // --------------------------------------------------------
    // Atualiza análise
    // --------------------------------------------------------

    void update(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Retorna resultado
    // --------------------------------------------------------

    CurvaData getData();
};

#endif