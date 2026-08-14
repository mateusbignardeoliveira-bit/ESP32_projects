#include "LinhaAnalise.h"


// ============================================================
// CONSTRUTOR
// ============================================================

LinhaAnalise::LinhaAnalise()
{

    resultado.situacao =
        LINHA_NORMAL;

    resultado.linhaDetectada =
        false;

    resultado.posicao =
        0.0f;

    resultado.erro =
        0.0f;

    resultado.intensidade =
        0.0f;

    resultado.largura =
        0;

    resultado.sensoresPretos =
        0;

    resultado.posicaoDigital =
        0.0f;

    resultado.eventoDigital =
        false;

}



// ============================================================
// NORMALIZA SENSOR
// ============================================================

float LinhaAnalise::normalizarSensor(
    int valor,
    int indice
)
{

    int limiteBranco =
        branco[indice];


    int limitePreto =
        preto[indice];


    // --------------------------------------------------------
    // Proteção
    // --------------------------------------------------------

    if(limitePreto <= limiteBranco)
    {
        return 0.0f;
    }


    // --------------------------------------------------------
    // Normalização
    // --------------------------------------------------------

    float normalizado =
        (float)(valor - limiteBranco)
        /
        (float)(limitePreto - limiteBranco);


    // --------------------------------------------------------
    // Limita 0...1
    // --------------------------------------------------------

    if(normalizado < 0.0f)
    {
        normalizado = 0.0f;
    }


    if(normalizado > 1.0f)
    {
        normalizado = 1.0f;
    }


    return normalizado;

}



// ============================================================
// OBTÉM ANALÓGICOS
// ============================================================

void LinhaAnalise::obterValoresAnalogicos(
    const ArrayData& dados,
    int valores[8]
)
{

    valores[0] = dados.s1;
    valores[1] = dados.s2;
    valores[2] = dados.s3;
    valores[3] = dados.s4;

    valores[4] = dados.s5;
    valores[5] = dados.s6;
    valores[6] = dados.s7;
    valores[7] = dados.s8;

}



// ============================================================
// OBTÉM DIGITAIS
// ============================================================

void LinhaAnalise::obterValoresDigitais(
    const ArrayData& dados,
    uint8_t valores[8]
)
{

    valores[0] = dados.d1;
    valores[1] = dados.d2;
    valores[2] = dados.d3;
    valores[3] = dados.d4;

    valores[4] = dados.d5;
    valores[5] = dados.d6;
    valores[6] = dados.d7;
    valores[7] = dados.d8;

}



// ============================================================
// UPDATE
// ============================================================

void LinhaAnalise::update(
    const ArrayData& dados
)
{

    // ========================================================
    // ARRAYS TEMPORÁRIOS
    // ========================================================

    int valoresAnalogicos[8];

    uint8_t valoresDigitais[8];


    obterValoresAnalogicos(
        dados,
        valoresAnalogicos
    );


    obterValoresDigitais(
        dados,
        valoresDigitais
    );


    // ========================================================
    // VARIÁVEIS ANALÓGICAS
    // ========================================================

    float somaIntensidade =
        0.0f;


    float somaPosicao =
        0.0f;


    float maiorIntensidade =
        0.0f;


    uint8_t sensoresAtivos =
        0;


    // ========================================================
    // PROCESSA ANALÓGICOS
    // ========================================================

    for(int i = 0; i < 8; i++)
    {

        float intensidade =
            normalizarSensor(
                valoresAnalogicos[i],
                i
            );


        // ----------------------------------------------------
        // Soma intensidade
        // ----------------------------------------------------

        somaIntensidade +=
            intensidade;


        // ----------------------------------------------------
        // Soma ponderada
        // ----------------------------------------------------

        somaPosicao +=
            intensidade *
            pesos[i];


        // ----------------------------------------------------
        // Maior sensor
        // ----------------------------------------------------

        if(intensidade > maiorIntensidade)
        {
            maiorIntensidade =
                intensidade;
        }


        // ----------------------------------------------------
        // Largura
        // ----------------------------------------------------

        if(intensidade >= 0.20f)
        {
            sensoresAtivos++;
        }

    }


    // ========================================================
    // LINHA DETECTADA
    // ========================================================

    resultado.linhaDetectada =
        maiorIntensidade >= 0.08f;


    // ========================================================
    // POSIÇÃO ANALÓGICA
    // ========================================================

    if(somaIntensidade > 0.01f)
    {

        resultado.posicao =
            somaPosicao /
            somaIntensidade;

    }
    else
    {

        resultado.posicao =
            0.0f;

    }


    // ========================================================
    // ERRO
    // ========================================================

    resultado.erro =
        resultado.posicao;


    // ========================================================
    // INTENSIDADE
    // ========================================================

    resultado.intensidade =
        somaIntensidade;


    // ========================================================
    // LARGURA
    // ========================================================

    resultado.largura =
        sensoresAtivos;


    // ========================================================
    // PROCESSAMENTO DIGITAL
    //
    // IMPORTANTE:
    //
    // 0 = PRETO
    // 1 = BRANCO
    // ========================================================

    uint8_t sensoresPretos =
        0;


    float somaDigital =
        0.0f;


    float somaPesoDigital =
        0.0f;


    for(int i = 0; i < 8; i++)
    {

        if(valoresDigitais[i] == 0)
        {

            sensoresPretos++;


            somaPesoDigital +=
                pesos[i];


            somaDigital +=
                1.0f;

        }

    }


    // ========================================================
    // SALVA QUANTIDADE DE PRETOS
    // ========================================================

    resultado.sensoresPretos =
        sensoresPretos;


    // ========================================================
    // POSIÇÃO DIGITAL
    // ========================================================

    if(somaDigital > 0.0f)
    {

        resultado.posicaoDigital =
            somaPesoDigital /
            somaDigital;

    }
    else
    {

        resultado.posicaoDigital =
            0.0f;

    }


    // ========================================================
    // EVENTO DIGITAL
    //
    // 0 = situação normal
    // 1 = desalinhamento
    // 2+ = evento
    // ========================================================

    resultado.eventoDigital =
        sensoresPretos >= 2;


    // ========================================================
    // SITUAÇÃO
    // ========================================================

    if(sensoresPretos == 0)
    {

        resultado.situacao =
            LINHA_NORMAL;

    }

    else if(sensoresPretos == 1)
    {

        resultado.situacao =
            LINHA_DESALINHADA;

    }

    else
    {

        resultado.situacao =
            LINHA_EVENTO;

    }

}



// ============================================================
// GET DATA
// ============================================================

LinhaData LinhaAnalise::getData()
{

    return resultado;

}