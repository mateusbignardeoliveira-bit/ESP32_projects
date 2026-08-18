#include "DecisaoRobo.h"

// ============================================================
// CONSTRUTOR
// ============================================================

DecisaoRobo::DecisaoRobo()
{
    resultado.acaoVerde =
        VERDE_SEM_EVENTO;

    resultado.verdeDetectado =
        false;

    resultado.verdeEsquerda =
        false;

    resultado.verdeDireita =
        false;

    resultado.eventoArray =
        ARRAY_NORMAL;

    resultado.sensoresPretos =
        0;

    resultado.posicaoDigital =
        0.0f;

    resultado.eventoEspecial =
        false;
}

// ============================================================
// ANALISA ARRAY
// ============================================================

TipoEventoArray DecisaoRobo::analisarArray(
    const LinhaData& linha
)
{
    // --------------------------------------------------------
    // 0 ou 1 sensor preto
    //
    // Situação normal.
    // --------------------------------------------------------

    if(linha.sensoresPretos <= 1)
    {
        return ARRAY_NORMAL;
    }

    // --------------------------------------------------------
    // 5 ou mais sensores pretos
    //
    // Intersecção.
    // --------------------------------------------------------

    if(linha.sensoresPretos >= 5)
    {
        return ARRAY_INTERSECCAO;
    }

    // --------------------------------------------------------
    // 2, 3 ou 4 sensores pretos
    //
    // Curva fechada ou falha da pista.
    //
    // O lado é determinado pela posição digital calculada
    // pela LinhaAnalise.
    //
    // Negativo = esquerda
    // Positivo = direita
    // --------------------------------------------------------

    if(linha.posicaoDigital < 0.0f)
    {
        return ARRAY_CURVA_ESQUERDA;
    }

    if(linha.posicaoDigital > 0.0f)
    {
        return ARRAY_CURVA_DIREITA;
    }

    // --------------------------------------------------------
    // Caso exatamente central.
    //
    // É uma situação muito improvável.
    //
    // Como não temos informação suficiente para escolher
    // um lado, usamos o evento digital como desempate.
    // --------------------------------------------------------

    if(linha.eventoDigital)
    {
        // Sem direção definida.
        //
        // Mantemos a decisão neutra para não inventar
        // uma direção.
        return ARRAY_NORMAL;
    }

    return ARRAY_NORMAL;
}

// ============================================================
// UPDATE
// ============================================================

void DecisaoRobo::update(
    const AS7341Resultado& esquerda,
    const AS7341Resultado& direita,
    const LinhaData& linha
)
{
    // --------------------------------------------------------
    // Limpa a decisão anterior
    // --------------------------------------------------------

    resultado.acaoVerde =
        VERDE_SEM_EVENTO;

    resultado.eventoArray =
        ARRAY_NORMAL;

    resultado.eventoEspecial =
        false;

    // --------------------------------------------------------
    // Dados dos AS7341
    // --------------------------------------------------------

    resultado.verdeEsquerda =
        esquerda.verdeDetectado;

    resultado.verdeDireita =
        direita.verdeDetectado;

    resultado.verdeDetectado =
        resultado.verdeEsquerda ||
        resultado.verdeDireita;

    // --------------------------------------------------------
    // Dados do array
    // --------------------------------------------------------

    resultado.sensoresPretos =
        linha.sensoresPretos;

    resultado.posicaoDigital =
        linha.posicaoDigital;

    // ========================================================
    // PRIORIDADE 1 — VERDE
    // ========================================================

    if(resultado.verdeDetectado)
    {
        // ----------------------------------------------------
        // Verde + 2 ou mais sensores pretos
        //
        // Situação de intersecção/passagem.
        //
        // Deve seguir reto.
        // ----------------------------------------------------

        if(resultado.sensoresPretos >= 2)
        {
            resultado.acaoVerde =
                VERDE_PASSAGEM_RETA;

            resultado.eventoEspecial =
                true;

            return;
        }

        // ----------------------------------------------------
        // Verde nos dois AS7341
        //
        // Só chega aqui com 0 ou 1 sensor preto.
        //
        // Deve fazer 180°.
        // ----------------------------------------------------

        if(
            resultado.verdeEsquerda &&
            resultado.verdeDireita
        )
        {
            resultado.acaoVerde =
                VERDE_MEIA_VOLTA;

            resultado.eventoEspecial =
                true;

            return;
        }

        // ----------------------------------------------------
        // Verde somente à esquerda
        //
        // 0 ou 1 sensor preto.
        //
        // Curva para esquerda.
        // ----------------------------------------------------

        if(resultado.verdeEsquerda)
        {
            resultado.acaoVerde =
                VERDE_CURVA_ESQUERDA;

            resultado.eventoEspecial =
                true;

            return;
        }

        // ----------------------------------------------------
        // Verde somente à direita
        //
        // 0 ou 1 sensor preto.
        //
        // Curva para direita.
        // ----------------------------------------------------

        if(resultado.verdeDireita)
        {
            resultado.acaoVerde =
                VERDE_CURVA_DIREITA;

            resultado.eventoEspecial =
                true;

            return;
        }
    }

    // ========================================================
    // PRIORIDADE 2 — ARRAY
    //
    // Só analisamos o evento do array quando não houve
    // uma decisão de verde.
    // ========================================================

    resultado.eventoArray =
        analisarArray(linha);

    // --------------------------------------------------------
    // Situação normal
    // --------------------------------------------------------

    if(
        resultado.eventoArray ==
        ARRAY_NORMAL
    )
    {
        resultado.eventoEspecial =
            false;

        return;
    }

    // --------------------------------------------------------
    // Curva ou intersecção
    // --------------------------------------------------------

    resultado.eventoEspecial =
        true;
}

// ============================================================
// GET DATA
// ============================================================

const DecisaoData& DecisaoRobo::getData() const
{
    return resultado;
}