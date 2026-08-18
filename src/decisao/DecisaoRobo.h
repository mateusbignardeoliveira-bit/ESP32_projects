#ifndef DECISAO_ROBO_H
#define DECISAO_ROBO_H

#include "../sensores/AS7341Analise.h"
#include "../sensores/LinhaAnalise.h"
#include "../sensores/TOFAnalise.h"

// ============================================================
// AÇÕES RELACIONADAS AO VERDE
// ============================================================

enum TipoAcaoVerde
{
    VERDE_SEM_EVENTO,
    VERDE_PASSAGEM_RETA,
    VERDE_CURVA_ESQUERDA,
    VERDE_CURVA_DIREITA,
    VERDE_MEIA_VOLTA
};

// ============================================================
// EVENTOS DO ARRAY
// ============================================================

enum TipoEventoArray
{
    ARRAY_NORMAL,
    ARRAY_CURVA_ESQUERDA,
    ARRAY_CURVA_DIREITA,
    ARRAY_INTERSECCAO
};

// ============================================================
// EVENTOS DO TOF
// ============================================================

enum TipoEventoToF
{
    TOF_NORMAL,
    TOF_OBSTACULO
};

// ============================================================
// RESULTADO DA DECISÃO
// ============================================================

struct DecisaoData
{
    // --------------------------------------------------------
    // TOF
    // --------------------------------------------------------

    TipoEventoToF eventoToF;

    bool obstaculoDetectado;

    int distanciaToF;

    // --------------------------------------------------------
    // VERDE
    // --------------------------------------------------------

    TipoAcaoVerde acaoVerde;

    bool verdeDetectado;

    bool verdeEsquerda;

    bool verdeDireita;

    // --------------------------------------------------------
    // ARRAY
    // --------------------------------------------------------

    TipoEventoArray eventoArray;

    uint8_t sensoresPretos;

    float posicaoDigital;

    // --------------------------------------------------------
    // Evento especial
    // --------------------------------------------------------

    bool eventoEspecial;
};

// ============================================================
// CLASSE
// ============================================================

class DecisaoRobo
{
private:

    DecisaoData resultado;

    // --------------------------------------------------------
    // Analisa array
    // --------------------------------------------------------

    TipoEventoArray analisarArray(
        const LinhaData& linha
    );

public:

    // ========================================================
    // CONSTRUTOR
    // ========================================================

    DecisaoRobo();

    // ========================================================
    // UPDATE
    //
    // PRIORIDADE:
    //
    // 1. TOF
    // 2. VERDE
    // 3. ARRAY
    // ========================================================

    void update(
        const AS7341Resultado& esquerda,
        const AS7341Resultado& direita,
        const LinhaData& linha,
        TOFAnalise& tof
    );

    // ========================================================
    // GET DATA
    // ========================================================

    const DecisaoData& getData() const;
};

#endif