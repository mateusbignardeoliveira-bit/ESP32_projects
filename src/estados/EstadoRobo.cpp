#include "EstadoRobo.h"

// ============================================================
// CONSTRUTOR
// ============================================================

EstadoRoboControl::EstadoRoboControl()
{
    estadoAtual =
        ESTADO_SEGUINDO_LINHA;

    ultimoLado =
        0;

    inicioPerdaLinha =
        0;
}

// ============================================================
// DETERMINAR LADO
// ============================================================

int EstadoRoboControl::determinarLado(
    const LinhaData& linha
)
{
    // --------------------------------------------------------
    // Posição analógica
    // --------------------------------------------------------

    if(linha.linhaDetectada)
    {
        if(linha.posicao < -1.0f)
        {
            return -1;
        }

        if(linha.posicao > 1.0f)
        {
            return 1;
        }

        return 0;
    }

    // --------------------------------------------------------
    // Posição digital
    // --------------------------------------------------------

    if(linha.sensoresPretos > 0)
    {
        if(linha.posicaoDigital < -1.0f)
        {
            return -1;
        }

        if(linha.posicaoDigital > 1.0f)
        {
            return 1;
        }
    }

    return 0;
}

// ============================================================
// MEMÓRIA DA LINHA
// ============================================================

void EstadoRoboControl::atualizarMemoriaLinha(
    const LinhaData& linha
)
{
    int lado =
        determinarLado(linha);

    if(lado != 0)
    {
        ultimoLado =
            lado;
    }
}

// ============================================================
// UPDATE PRINCIPAL
//
// PRIORIDADE:
//
// 1. Verde
// 2. Array
// ============================================================

void EstadoRoboControl::update(
    const DecisaoData& decisao
)
{
    // ========================================================
    // PRIORIDADE 1 — VERDE
    // ========================================================

    if(decisao.verdeDetectado)
    {
        // ----------------------------------------------------
        // Dois ou mais sensores pretos
        //
        // Mesmo havendo verde, é passagem/intersecção.
        // ----------------------------------------------------

        if(decisao.sensoresPretos >= 2)
        {
            estadoAtual =
                ESTADO_PASSAGEM_RETA;

            return;
        }

        // ----------------------------------------------------
        // Dois verdes
        //
        // Com 0 ou 1 sensor preto:
        // meia volta.
        // ----------------------------------------------------

        if(
            decisao.verdeEsquerda &&
            decisao.verdeDireita
        )
        {
            estadoAtual =
                ESTADO_MEIA_VOLTA;

            return;
        }

        // ----------------------------------------------------
        // Verde somente esquerda
        // ----------------------------------------------------

        if(decisao.verdeEsquerda)
        {
            estadoAtual =
                ESTADO_CURVA_ESQUERDA;

            return;
        }

        // ----------------------------------------------------
        // Verde somente direita
        // ----------------------------------------------------

        if(decisao.verdeDireita)
        {
            estadoAtual =
                ESTADO_CURVA_DIREITA;

            return;
        }
    }

    // ========================================================
    // PRIORIDADE 2 — ARRAY
    // ========================================================

    switch(decisao.eventoArray)
    {
        // ----------------------------------------------------
        // Normal
        // ----------------------------------------------------

        case ARRAY_NORMAL:

            estadoAtual =
                ESTADO_SEGUINDO_LINHA;

            return;

        // ----------------------------------------------------
        // Curva esquerda
        // ----------------------------------------------------

        case ARRAY_CURVA_ESQUERDA:

            estadoAtual =
                ESTADO_CURVA_ESQUERDA;

            return;

        // ----------------------------------------------------
        // Curva direita
        // ----------------------------------------------------

        case ARRAY_CURVA_DIREITA:

            estadoAtual =
                ESTADO_CURVA_DIREITA;

            return;

        // ----------------------------------------------------
        // Intersecção
        // ----------------------------------------------------

        case ARRAY_INTERSECCAO:

            estadoAtual =
                ESTADO_INTERSECCAO;

            return;

        default:

            estadoAtual =
                ESTADO_SEGUINDO_LINHA;

            return;
    }
}

// ============================================================
// UPDATE ANTIGO COM TOF
//
// Mantido por compatibilidade.
// ============================================================

void EstadoRoboControl::update(
    const LinhaData& linha,
    const TOFAnalise& tof
)
{
    // ========================================================
    // OBSTÁCULO
    // ========================================================

    if(
        tof.isValido() &&
        tof.temObstaculo()
    )
    {
        estadoAtual =
            ESTADO_OBSTACULO;

        return;
    }

    // ========================================================
    // MEMÓRIA
    // ========================================================

    atualizarMemoriaLinha(linha);

    // ========================================================
    // LINHA DETECTADA
    // ========================================================

    if(linha.linhaDetectada)
    {
        inicioPerdaLinha =
            0;

        if(linha.posicao <= -LIMITE_CURVA)
        {
            estadoAtual =
                ESTADO_CURVA_ESQUERDA;

            return;
        }

        if(linha.posicao >= LIMITE_CURVA)
        {
            estadoAtual =
                ESTADO_CURVA_DIREITA;

            return;
        }

        if(
            fabs(linha.erro) >=
            LIMITE_DESALINHAMENTO
        )
        {
            estadoAtual =
                ESTADO_DESALINHADO;

            return;
        }

        estadoAtual =
            ESTADO_SEGUINDO_LINHA;

        return;
    }

    // ========================================================
    // LINHA NÃO DETECTADA
    // ========================================================

    if(inicioPerdaLinha == 0)
    {
        inicioPerdaLinha =
            millis();
    }

    unsigned long tempoPerdido =
        millis() -
        inicioPerdaLinha;

    if(
        tempoPerdido >=
        TEMPO_RECUPERACAO
    )
    {
        estadoAtual =
            ESTADO_RECUPERACAO;

        return;
    }

    estadoAtual =
        ESTADO_DESALINHADO;
}

// ============================================================
// GET ESTADO
// ============================================================

EstadoRobo EstadoRoboControl::getEstado()
{
    return estadoAtual;
}

// ============================================================
// DEFINIR ESTADO
// ============================================================

void EstadoRoboControl::definirEstado(
    EstadoRobo novoEstado
)
{
    estadoAtual =
        novoEstado;
}

// ============================================================
// NOME DO ESTADO
// ============================================================

const char* EstadoRoboControl::getNomeEstado()
{
    switch(estadoAtual)
    {
        case ESTADO_SEGUINDO_LINHA:
            return "SEGUINDO_LINHA";

        case ESTADO_CURVA_ESQUERDA:
            return "CURVA_ESQUERDA";

        case ESTADO_CURVA_DIREITA:
            return "CURVA_DIREITA";

        case ESTADO_INTERSECCAO:
            return "INTERSECCAO";

        case ESTADO_PASSAGEM_RETA:
            return "PASSAGEM_RETA";

        case ESTADO_MEIA_VOLTA:
            return "MEIA_VOLTA";

        case ESTADO_DESALINHADO:
            return "DESALINHADO";

        case ESTADO_OBSTACULO:
            return "OBSTACULO";

        case ESTADO_RECUPERACAO:
            return "RECUPERACAO";

        default:
            return "DESCONHECIDO";
    }
}