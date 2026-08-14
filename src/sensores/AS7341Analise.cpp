#include "AS7341Analise.h"


// ============================================================
// CONSTRUTOR
// ============================================================

AS7341Analise::AS7341Analise()
{
}


// ============================================================
// INTENSIDADE GERAL
// ============================================================

float AS7341Analise::calcularIntensidade(
    const AS7341Data& dados
)
{

    if(!dados.valido)
    {
        return 0;
    }


    // Utilizamos o Clear como referência da
    // quantidade total de luz recebida.

    return (float)dados.clear;

}


// ============================================================
// AZUL
// ============================================================

float AS7341Analise::calcularAzul(
    const AS7341Data& dados
)
{

    if(!dados.valido)
    {
        return 0;
    }


    // F1 e F2 possuem maior sensibilidade
    // na região azul.

    return (
        (float)dados.F1 +
        (float)dados.F2
    ) / 2.0f;

}


// ============================================================
// VERDE
// ============================================================

float AS7341Analise::calcularVerde(
    const AS7341Data& dados
)
{

    if(!dados.valido)
    {
        return 0;
    }


    // F3 e F4 ficam na região verde.

    return (
        (float)dados.F3 +
        (float)dados.F4
    ) / 2.0f;

}


// ============================================================
// VERMELHO
// ============================================================

float AS7341Analise::calcularVermelho(
    const AS7341Data& dados
)
{

    if(!dados.valido)
    {
        return 0;
    }


    // F6 e F7 possuem forte resposta na região
    // vermelho / vermelho profundo.

    return (
        (float)dados.F6 +
        (float)dados.F7
    ) / 2.0f;

}


// ============================================================
// CANAL DOMINANTE
// ============================================================

uint8_t AS7341Analise::descobrirCanalDominante(
    float azul,
    float verde,
    float vermelho
)
{

    if(
        azul >= verde &&
        azul >= vermelho
    )
    {
        return 1;
    }


    if(
        verde >= azul &&
        verde >= vermelho
    )
    {
        return 2;
    }


    return 3;

}


// ============================================================
// ANALISAR
// ============================================================

AS7341Resultado AS7341Analise::analisar(
    const AS7341Data& dados
)
{

    AS7341Resultado resultado;


    // --------------------------------------------------------
    // Sensor inválido
    // --------------------------------------------------------

    if(!dados.valido)
    {

        resultado.valido = false;

        resultado.intensidade = 0;

        resultado.azul = 0;

        resultado.verde = 0;

        resultado.vermelho = 0;

        resultado.razaoVermelhoVerde = 0;

        resultado.razaoAzulVerde = 0;

        resultado.razaoNIR = 0;

        resultado.canalDominante = 0;

        return resultado;

    }


    resultado.valido = true;


    // --------------------------------------------------------
    // Intensidade
    // --------------------------------------------------------

    resultado.intensidade =
        calcularIntensidade(dados);


    // --------------------------------------------------------
    // Componentes
    // --------------------------------------------------------

    resultado.azul =
        calcularAzul(dados);


    resultado.verde =
        calcularVerde(dados);


    resultado.vermelho =
        calcularVermelho(dados);


    // --------------------------------------------------------
    // Razão vermelho / verde
    // --------------------------------------------------------

    if(resultado.verde > 0)
    {

        resultado.razaoVermelhoVerde =
            resultado.vermelho /
            resultado.verde;

    }
    else
    {

        resultado.razaoVermelhoVerde = 0;

    }


    // --------------------------------------------------------
    // Razão azul / verde
    // --------------------------------------------------------

    if(resultado.verde > 0)
    {

        resultado.razaoAzulVerde =
            resultado.azul /
            resultado.verde;

    }
    else
    {

        resultado.razaoAzulVerde = 0;

    }


    // --------------------------------------------------------
    // Razão NIR / Clear
    // --------------------------------------------------------

    if(dados.clear > 0)
    {

        resultado.razaoNIR =
            (float)dados.nir /
            (float)dados.clear;

    }
    else
    {

        resultado.razaoNIR = 0;

    }


    // --------------------------------------------------------
    // Canal dominante
    //
    // 1 = azul
    // 2 = verde
    // 3 = vermelho
    // --------------------------------------------------------

    resultado.canalDominante =
        descobrirCanalDominante(
            resultado.azul,
            resultado.verde,
            resultado.vermelho
        );


    return resultado;

}


// ============================================================
// COMPARAR DIREITA / ESQUERDA
// ============================================================

AS7341Comparacao AS7341Analise::comparar(
    const AS7341Resultado& direita,
    const AS7341Resultado& esquerda
)
{

    AS7341Comparacao resultado;


    resultado.diferencaIntensidade =
        direita.intensidade -
        esquerda.intensidade;


    resultado.diferencaVermelho =
        direita.vermelho -
        esquerda.vermelho;


    resultado.diferencaVerde =
        direita.verde -
        esquerda.verde;


    resultado.diferencaAzul =
        direita.azul -
        esquerda.azul;


    resultado.direitaMaisIntensa =
        direita.intensidade >
        esquerda.intensidade;


    resultado.esquerdaMaisIntensa =
        esquerda.intensidade >
        direita.intensidade;


    return resultado;

}