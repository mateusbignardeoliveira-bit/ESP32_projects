#include "DecisaoRobo.h"

// ============================================================
// CONSTRUTOR
// ============================================================

DecisaoRobo::DecisaoRobo()
{
    resultado.eventoToF =
        TOF_NORMAL;

    resultado.obstaculoDetectado =
        false;

    resultado.distanciaToF =
        0;

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
    // --------------------------------------------------------

    if(linha.sensoresPretos <= 1)
    {
        return ARRAY_NORMAL;
    }

    // --------------------------------------------------------
    // 5 ou mais sensores pretos
    // = INTERSECÇÃO
    // --------------------------------------------------------

    if(linha.sensoresPretos >= 5)
    {
        return ARRAY_INTERSECCAO;
    }

    // --------------------------------------------------------
    // 2, 3 ou 4 sensores
    //
    // Define lado pela posição digital.
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
    // Caso central improvável
    // --------------------------------------------------------

    return ARRAY_NORMAL;
}

// ============================================================
// UPDATE
// ============================================================

void DecisaoRobo::update(
    const AS7341Resultado& esquerda,
    const AS7341Resultado& direita,
    const LinhaData& linha,
    TOFAnalise& tof
)
{
    // ========================================================
    // LIMPA EVENTOS DA ITERAÇÃO
    // ========================================================

    resultado.eventoToF =
        TOF_NORMAL;

    resultado.obstaculoDetectado =
        false;

    resultado.acaoVerde =
        VERDE_SEM_EVENTO;

    resultado.eventoArray =
        ARRAY_NORMAL;

    resultado.eventoEspecial =
        false;

    // ========================================================
    // DADOS DO TOF
    // ========================================================

    resultado.distanciaToF =
        tof.getDistancia();

    // ========================================================
    // PRIORIDADE 1 — OBSTÁCULO
    // ========================================================

    if(
        tof.isValido() &&
        tof.temObstaculo()
    )
    {
        resultado.eventoToF =
            TOF_OBSTACULO;

        resultado.obstaculoDetectado =
            true;

        resultado.eventoEspecial =
            true;

        // ----------------------------------------------------
        // Mantemos os dados dos outros sensores disponíveis
        // para diagnóstico, mas eles não decidem a ação.
        // ----------------------------------------------------

        resultado.verdeEsquerda =
            esquerda.verdeDetectado;

        resultado.verdeDireita =
            direita.verdeDetectado;

        resultado.verdeDetectado =
            resultado.verdeEsquerda ||
            resultado.verdeDireita;

        resultado.sensoresPretos =
            linha.sensoresPretos;

        resultado.posicaoDigital =
            linha.posicaoDigital;

        return;
    }

    // ========================================================
    // DADOS DOS AS7341
    // ========================================================

    resultado.verdeEsquerda =
        esquerda.verdeDetectado;

    resultado.verdeDireita =
        direita.verdeDetectado;

    resultado.verdeDetectado =
        resultado.verdeEsquerda ||
        resultado.verdeDireita;

    // ========================================================
    // DADOS DO ARRAY
    // ========================================================

    resultado.sensoresPretos =
        linha.sensoresPretos;

    resultado.posicaoDigital =
        linha.posicaoDigital;

    // ========================================================
    // PRIORIDADE 2 — VERDE
    // ========================================================

    if(resultado.verdeDetectado)
    {
        // ----------------------------------------------------
        // Verde + 2 ou mais pretos
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
        // Verde dos dois lados
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
        // Verde esquerda
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
        // Verde direita
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
    // PRIORIDADE 3 — ARRAY
    // ========================================================

    resultado.eventoArray =
        analisarArray(linha);

    // --------------------------------------------------------
    // Normal
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
    // Evento especial do array
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