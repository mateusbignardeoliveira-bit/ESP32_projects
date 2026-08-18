#ifndef DECISAO_ROBO_H
#define DECISAO_ROBO_H

#include "../sensores/AS7341Analise.h"
#include "../sensores/LinhaAnalise.h"

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
// RESULTADO DA DECISÃO
// ============================================================

struct DecisaoData
{
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
    // Existe algum evento especial?
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
    // Analisa somente os eventos do array
    // --------------------------------------------------------

    TipoEventoArray analisarArray(
        const LinhaData& linha
    );

public:

    DecisaoRobo();

    // --------------------------------------------------------
    // Atualiza decisão
    // --------------------------------------------------------

    void update(
        const AS7341Resultado& esquerda,
        const AS7341Resultado& direita,
        const LinhaData& linha
    );

    // --------------------------------------------------------
    // Retorna decisão
    // --------------------------------------------------------

    const DecisaoData& getData() const;
};

#endif