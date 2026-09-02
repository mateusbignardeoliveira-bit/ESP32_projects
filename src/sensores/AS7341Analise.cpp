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
// Mantemos a margem atualmente utilizada pelo projeto.
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
        return 0.0f;
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
        return 0.0f;
    }


    // F1 e F2 possuem maior resposta
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
        return 0.0f;
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
        return 0.0f;
    }


    // F6 e F7 possuem forte resposta
    // na região vermelho / vermelho profundo.

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
// A detecção utiliza:
//
// F4/F3
//
// e
//
// F4/F6
//
// O verde precisa satisfazer as duas condições.
//

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

    if(
        f3 <= 0.0f ||
        f6 <= 0.0f
    )
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
// DETECTAR VERMELHO
// ============================================================
//
// A estrutura da detecção já existe.
//
// Porém ainda NÃO colocamos um limiar arbitrário.
//
// Precisamos usar as leituras reais do seu robô para definir
// a separação entre:
//
// - vermelho
// - branco
// - preto
// - verde
//
// Portanto, nesta etapa:
//
// false = detecção ainda não calibrada.
//
// ============================================================

bool AS7341Analise::detectarVermelho(
    const AS7341Data& dados
)
{
    if(!dados.valido)
    {
        return false;
    }


    // --------------------------------------------------------
    // NÃO ATIVAR AINDA
    // --------------------------------------------------------
    //
    // O cálculo de vermelho já está disponível em:
    //
    // dados.F6
    // dados.F7
    //
    // e também em:
    //
    // resultado.vermelho
    //
    // A decisão será adicionada depois das medições reais.
    //

    return false;
}


// ============================================================
// DETECTAR CINZA
// ============================================================
//
// Cinza também não deve ser identificado simplesmente por:
//
// "vermelho baixo"
// "verde baixo"
// "azul baixo"
//
// Isso poderia confundir:
//
// - preto
// - branco
// - cinza
//
// Por isso precisamos das leituras reais do sensor sobre o
// material cinza usado na pista.
//
// Nesta etapa a função existe, mas permanece desativada.
//

bool AS7341Analise::detectarCinza(
    const AS7341Data& dados
)
{
    if(!dados.valido)
    {
        return false;
    }


    // --------------------------------------------------------
    // NÃO ATIVAR AINDA
    // --------------------------------------------------------

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


    // ========================================================
    // INICIALIZAÇÃO SEGURA
    // ========================================================

    resultado.valido = false;

    resultado.intensidade = 0.0f;

    resultado.azul = 0.0f;

    resultado.verde = 0.0f;

    resultado.vermelho = 0.0f;

    resultado.razaoVermelhoVerde = 0.0f;

    resultado.razaoAzulVerde = 0.0f;

    resultado.razaoNIR = 0.0f;

    resultado.canalDominante = 0;

    resultado.verdeDetectado = false;

    resultado.vermelhoDetectado = false;

    resultado.cinzaDetectado = false;


    // ========================================================
    // SENSOR INVÁLIDO
    // ========================================================

    if(!dados.valido)
    {
        return resultado;
    }


    resultado.valido = true;


    // ========================================================
    // INTENSIDADE
    // ========================================================

    resultado.intensidade =
        calcularIntensidade(
            dados
        );


    // ========================================================
    // COMPONENTES
    // ========================================================

    resultado.azul =
        calcularAzul(
            dados
        );


    resultado.verde =
        calcularVerde(
            dados
        );


    resultado.vermelho =
        calcularVermelho(
            dados
        );


    // ========================================================
    // RAZÃO VERMELHO / VERDE
    // ========================================================

    if(
        resultado.verde > 0.0f
    )
    {
        resultado.razaoVermelhoVerde =
            resultado.vermelho /
            resultado.verde;
    }
    else
    {
        resultado.razaoVermelhoVerde =
            0.0f;
    }


    // ========================================================
    // RAZÃO AZUL / VERDE
    // ========================================================

    if(
        resultado.verde > 0.0f
    )
    {
        resultado.razaoAzulVerde =
            resultado.azul /
            resultado.verde;
    }
    else
    {
        resultado.razaoAzulVerde =
            0.0f;
    }


    // ========================================================
    // RAZÃO NIR / CLEAR
    // ========================================================

    if(
        dados.clear > 0
    )
    {
        resultado.razaoNIR =
            (float)dados.nir /
            (float)dados.clear;
    }
    else
    {
        resultado.razaoNIR =
            0.0f;
    }


    // ========================================================
    // CANAL DOMINANTE
    // ========================================================

    resultado.canalDominante =
        descobrirCanalDominante(
            resultado.azul,
            resultado.verde,
            resultado.vermelho
        );


    // ========================================================
    // VERDE
    // ========================================================

    resultado.verdeDetectado =
        detectarVerde(
            dados
        );


    // ========================================================
    // VERMELHO
    // ========================================================

    resultado.vermelhoDetectado =
        detectarVermelho(
            dados
        );


    // ========================================================
    // CINZA
    // ========================================================

    resultado.cinzaDetectado =
        detectarCinza(
            dados
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