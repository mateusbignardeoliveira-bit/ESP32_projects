#include "AS7341Analise.h"

// ============================================================
// CALIBRAÇÃO DA DETECÇÃO DE VERDE
// ============================================================
//
// Baseado nas leituras reais do robô:
//
// VERDE:
// F4/F3 ≈ 2.3 ~ 2.65
// F4/F6 ≈ 1.49
//
// BRANCO:
// F4/F3 ≈ 1.28 ~ 1.50
// F4/F6 ≈ 0.80 ~ 0.92
//
// PRETO:
// F4/F3 ≈ 1.10 ~ 1.41
// F4/F6 ≈ 0.57 ~ 0.68
//
// Portanto usamos duas condições simultâneas.
//

static const float LIMIAR_F4_F3 = 1.80f;

static const float LIMIAR_F4_F6 = 1.05f;


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
//
// 0 = inválido
// 1 = azul
// 2 = verde
// 3 = vermelho
//
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
// DETECTAR VERDE
// ============================================================
//
// A detecção agora usa diretamente os canais:
//
// F3 = referência na região verde
// F4 = canal principal usado para caracterizar o verde
// F6 = referência na região vermelho profundo
//
// O verde precisa:
//
// F4/F3 > 1.90
//
// E:
//
// F4/F6 > 1.15
//
// Isso foi definido usando as medições reais:
//
// VERDE:
// F4/F3 ≈ 2.3 ~ 2.65
// F4/F6 ≈ 1.49
//
// BRANCO:
// F4/F3 ≈ 1.28 ~ 1.50
// F4/F6 ≈ 0.80 ~ 0.92
//
// PRETO:
// F4/F3 ≈ 1.10 ~ 1.41
// F4/F6 ≈ 0.57 ~ 0.68
//
// ============================================================

bool AS7341Analise::detectarVerde(
    const AS7341Data& dados
)
{
    if(!dados.valido)
    {
        return false;
    }

    const float f3 =
        (float)dados.F3;

    const float f4 =
        (float)dados.F4;

    const float f6 =
        (float)dados.F6;


    // --------------------------------------------------------
    // Evita divisão por zero
    // --------------------------------------------------------

    if(f3 <= 0 || f6 <= 0)
    {
        return false;
    }


    // --------------------------------------------------------
    // Razões espectrais
    // --------------------------------------------------------

    const float razaoF4F3 =
        f4 / f3;

    const float razaoF4F6 =
        f4 / f6;


    // --------------------------------------------------------
    // Detecção
    // --------------------------------------------------------

    if(
        razaoF4F3 > LIMIAR_F4_F3 &&
        razaoF4F6 > LIMIAR_F4_F6
    )
    {
        return true;
    }


    return false;
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
    // Inicialização segura
    // --------------------------------------------------------

    resultado.valido = false;

    resultado.intensidade = 0;

    resultado.azul = 0;
    resultado.verde = 0;
    resultado.vermelho = 0;

    resultado.razaoVermelhoVerde = 0;
    resultado.razaoAzulVerde = 0;

    resultado.razaoNIR = 0;

    resultado.canalDominante = 0;

    resultado.verdeDetectado = false;


    // --------------------------------------------------------
    // Sensor inválido
    // --------------------------------------------------------

    if(!dados.valido)
    {
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
    // --------------------------------------------------------

    resultado.canalDominante =
        descobrirCanalDominante(
            resultado.azul,
            resultado.verde,
            resultado.vermelho
        );


    // --------------------------------------------------------
    // VERDE
    // --------------------------------------------------------

    resultado.verdeDetectado =
        detectarVerde(dados);


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