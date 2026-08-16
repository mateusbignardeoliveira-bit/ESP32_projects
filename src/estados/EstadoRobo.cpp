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
    // Usa primeiro a posição analógica
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
    // Se perdeu a leitura analógica,
    // tenta usar a posição digital
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


    // --------------------------------------------------------
    // Não foi possível determinar
    // --------------------------------------------------------

    return 0;

}


// ============================================================
// ATUALIZAR MEMÓRIA DA LINHA
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
// UPDATE
// ============================================================

void EstadoRoboControl::update(
    const LinhaData& linha,
    const TOFAnalise& tof
)
{

    // ========================================================
    // 1. OBSTÁCULO
    //
    // Maior prioridade do sistema.
    // ========================================================

    if(tof.isValido() &&
       tof.temObstaculo())
    {

        estadoAtual =
            ESTADO_OBSTACULO;

        return;

    }


    // ========================================================
    // Atualiza memória da linha
    // ========================================================

    atualizarMemoriaLinha(linha);


    // ========================================================
    // 2. LINHA NORMAL
    // ========================================================

    if(linha.linhaDetectada)
    {

        inicioPerdaLinha =
            0;


        // ----------------------------------------------------
        // Curva para esquerda
        // ----------------------------------------------------

        if(linha.posicao <= -LIMITE_CURVA)
        {

            estadoAtual =
                ESTADO_CURVA_ESQUERDA;

            return;

        }


        // ----------------------------------------------------
        // Curva para direita
        // ----------------------------------------------------

        if(linha.posicao >= LIMITE_CURVA)
        {

            estadoAtual =
                ESTADO_CURVA_DIREITA;

            return;

        }


        // ----------------------------------------------------
        // Desalinhamento forte
        // ----------------------------------------------------

        if(fabs(linha.erro) >=
           LIMITE_DESALINHAMENTO)
        {

            estadoAtual =
                ESTADO_DESALINHADO;

            return;

        }


        // ----------------------------------------------------
        // Seguimento normal
        // ----------------------------------------------------

        estadoAtual =
            ESTADO_SEGUINDO_LINHA;

        return;

    }


    // ========================================================
    // 3. LINHA PERDIDA
    // ========================================================

    if(inicioPerdaLinha == 0)
    {

        inicioPerdaLinha =
            millis();

    }


    unsigned long tempoPerdido =
        millis() - inicioPerdaLinha;


    // ========================================================
    // Recuperação
    // ========================================================

    if(tempoPerdido >=
       TEMPO_RECUPERACAO)
    {

        estadoAtual =
            ESTADO_RECUPERACAO;

        return;

    }


    // --------------------------------------------------------
    // Durante uma perda muito curta,
    // considera desalinhamento
    // --------------------------------------------------------

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
// DEFINIR ESTADO MANUAL
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