#include "AS7341Analise.h"

#include <Arduino.h>
#include <math.h>


// ============================================================
// LIMIARES GERAIS DE INTENSIDADE
// ============================================================
//
// Valores calibrados a partir das leituras reais:
//
// PRETO  -> Clear = 9 ~ 10
// VERDE  -> Clear = 16 ~ 19
// VERMELHO -> Clear = 22 ~ 25
// CINZA  -> Clear = 36
// BRANCO -> Clear = 36
//
// O Clear será utilizado principalmente para impedir que
// o ruído de leituras muito baixas seja interpretado como cor.
// ============================================================

static const float LIMIAR_MIN_VERDE_CLEAR = 12.0f;
static const float LIMIAR_MIN_VERMELHO_CLEAR = 15.0f;
static const float LIMIAR_MIN_CINZA_CLEAR = 32.0f;


// ============================================================
// LIMIARES DE VERDE
// ============================================================
//
// Verde medido:
//
// ESQUERDA:
// R/G   ≈ 0.64
// G/B   ≈ 2.20
// F4/F6 = 2.00
//
// DIREITA:
// R/G   ≈ 0.75
// G/B   ≈ 1.71
// F4/F6 = 1.50
//
// Branco:
//
// R/G   ≈ 1.56 ~ 1.80
// G/B   ≈ 1.25 ~ 1.30
// F4/F6 ≈ 0.66 ~ 0.75
//
// Portanto usamos três características simultaneamente.
// ============================================================

static const float LIMIAR_VERDE_R_G_MAX = 1.15f;
static const float LIMIAR_VERDE_G_B_MIN = 1.30f;
static const float LIMIAR_VERDE_F4_F6_MIN = 1.10f;


// ============================================================
// LIMIARES DE VERMELHO
// ============================================================
//
// Vermelho medido:
//
// R/G   ≈ 5.8 ~ 6.3
// R/B   ≈ 5.8 ~ 6.3
// R/C   ≈ 0.62 ~ 0.66
//
// As demais cores ficam muito abaixo nessas características.
//
// Usamos margem considerável para evitar falso positivo.
// ============================================================

static const float LIMIAR_VERMELHO_R_G_MIN = 4.0f;
static const float LIMIAR_VERMELHO_R_B_MIN = 4.0f;
static const float LIMIAR_VERMELHO_R_CLEAR_MIN = 0.58f;


// ============================================================
// LIMIARES DE CINZA
// ============================================================
//
// Cinza:
//
// R/Clear ≈ 0.40 ~ 0.49
// G/B     ≈ 0.96 ~ 1.06
// Clear   = 36
//
// Branco:
//
// R/Clear ≈ 0.88 ~ 0.93
// G/B     ≈ 1.25 ~ 1.30
// Clear   = 36
//
// Preto:
//
// R/Clear ≈ 0.22 ~ 0.35
// Clear   = 9 ~ 10
//
// O cinza, portanto, é identificado por:
// - intensidade alta
// - vermelho representando uma parcela intermediária
//   do Clear
// - verde e azul relativamente próximos
//
// Não usamos mais a antiga variação RGB de 25%, pois ela
// não corresponde às leituras reais do material.
// ============================================================

static const float LIMIAR_CINZA_R_CLEAR_MIN = 0.38f;
static const float LIMIAR_CINZA_R_CLEAR_MAX = 0.52f;

static const float LIMIAR_CINZA_G_B_MIN = 0.90f;
static const float LIMIAR_CINZA_G_B_MAX = 1.10f;


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
        return 0.0f;

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
        return 0.0f;

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
        return 0.0f;

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
        return 0.0f;

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
// ============================================================

uint8_t AS7341Analise::descobrirCanalDominante(
    float azul,
    float verde,
    float vermelho
)
{
    if(
        azul <= 0.0f &&
        verde <= 0.0f &&
        vermelho <= 0.0f
    )
    {
        return 0;
    }

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
// Regras calibradas:
//
// 1. Clear precisa estar acima do preto
// 2. Vermelho precisa ser menor que verde
// 3. Verde precisa superar azul
// 4. F4 precisa superar F6
//
// Não usamos mais F4/F3 como critério principal porque
// o branco também apresentou F4/F3 elevado.
// ============================================================

bool AS7341Analise::detectarVerde(
    const AS7341Data& dados
)
{
    if(!dados.valido)
        return false;

    const float azul =
        calcularAzul(dados);

    const float verde =
        calcularVerde(dados);

    const float vermelho =
        calcularVermelho(dados);

    const float clear =
        calcularIntensidade(dados);

    const float f4 =
        (float)dados.F4;

    const float f6 =
        (float)dados.F6;


    // --------------------------------------------------------
    // Leitura muito escura -> não classificar como verde.
    // --------------------------------------------------------

    if(
        clear < LIMIAR_MIN_VERDE_CLEAR
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Valores precisam ser válidos.
    // --------------------------------------------------------

    if(
        azul <= 0.0f ||
        verde <= 0.0f ||
        vermelho <= 0.0f ||
        f6 <= 0.0f
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Razão vermelho / verde
    // --------------------------------------------------------

    const float razaoRG =
        vermelho / verde;


    // --------------------------------------------------------
    // Razão verde / azul
    // --------------------------------------------------------

    const float razaoGB =
        verde / azul;


    // --------------------------------------------------------
    // Relação F4 / F6
    // --------------------------------------------------------

    const float razaoF4F6 =
        f4 / f6;


    // --------------------------------------------------------
    // Verde precisa ter:
    //
    // R/G baixo
    // G/B suficientemente alto
    // F4/F6 alto
    // --------------------------------------------------------

    if(
        razaoRG >= LIMIAR_VERDE_R_G_MAX
    )
    {
        return false;
    }

    if(
        razaoGB <= LIMIAR_VERDE_G_B_MIN
    )
    {
        return false;
    }

    if(
        razaoF4F6 <= LIMIAR_VERDE_F4_F6_MIN
    )
    {
        return false;
    }


    return true;
}


// ============================================================
// DETECTAR VERMELHO
// ============================================================
//
// Vermelho real:
//
// R/G ≈ 6
// R/B ≈ 6
// R/Clear ≈ 0.63
//
// Utilizamos três condições simultâneas.
// ============================================================

bool AS7341Analise::detectarVermelho(
    const AS7341Data& dados
)
{
    if(!dados.valido)
        return false;

    const float azul =
        calcularAzul(dados);

    const float verde =
        calcularVerde(dados);

    const float vermelho =
        calcularVermelho(dados);

    const float clear =
        calcularIntensidade(dados);


    // --------------------------------------------------------
    // Leitura precisa possuir intensidade suficiente.
    // --------------------------------------------------------

    if(
        clear < LIMIAR_MIN_VERMELHO_CLEAR
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Valores precisam ser válidos.
    // --------------------------------------------------------

    if(
        azul <= 0.0f ||
        verde <= 0.0f ||
        vermelho <= 0.0f ||
        clear <= 0.0f
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Razão vermelho / verde
    // --------------------------------------------------------

    const float razaoRG =
        vermelho / verde;


    // --------------------------------------------------------
    // Razão vermelho / azul
    // --------------------------------------------------------

    const float razaoRB =
        vermelho / azul;


    // --------------------------------------------------------
    // Participação do vermelho no Clear
    // --------------------------------------------------------

    const float razaoRClear =
        vermelho / clear;


    // --------------------------------------------------------
    // Vermelho precisa ser fortemente dominante.
    // --------------------------------------------------------

    if(
        razaoRG <= LIMIAR_VERMELHO_R_G_MIN
    )
    {
        return false;
    }

    if(
        razaoRB <= LIMIAR_VERMELHO_R_B_MIN
    )
    {
        return false;
    }

    if(
        razaoRClear <= LIMIAR_VERMELHO_R_CLEAR_MIN
    )
    {
        return false;
    }


    return true;
}


// ============================================================
// DETECTAR CINZA
// ============================================================
//
// O antigo algoritmo procurava RGB quase iguais.
//
// Isso não funciona para este sensor:
//
// CINZA REAL:
// Azul  ≈ 8.5 ~ 11.5
// Verde ≈ 9.0 ~ 11.0
// Vermelho ≈ 14.5 ~ 17.5
//
// Portanto usamos uma assinatura relativa:
//
// - Clear alto
// - R/Clear intermediário
// - G/B próximo de 1
//
// Isso separa:
//
// CINZA:
// R/C ≈ 0.40 ~ 0.49
// G/B ≈ 0.96 ~ 1.06
//
// BRANCO:
// R/C ≈ 0.88 ~ 0.93
// G/B ≈ 1.25 ~ 1.30
//
// PRETO:
// Clear ≈ 9 ~ 10
// ============================================================

bool AS7341Analise::detectarCinza(
    const AS7341Data& dados
)
{
    if(!dados.valido)
        return false;

    const float azul =
        calcularAzul(dados);

    const float verde =
        calcularVerde(dados);

    const float vermelho =
        calcularVermelho(dados);

    const float clear =
        calcularIntensidade(dados);


    // --------------------------------------------------------
    // Intensidade precisa estar na região clara.
    //
    // Preto fica automaticamente excluído.
    // --------------------------------------------------------

    if(
        clear < LIMIAR_MIN_CINZA_CLEAR
    )
    {
        return false;
    }


    if(
        azul <= 0.0f ||
        verde <= 0.0f ||
        vermelho <= 0.0f
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Participação do vermelho no Clear.
    // --------------------------------------------------------

    const float razaoRClear =
        vermelho / clear;


    // --------------------------------------------------------
    // Relação verde / azul.
    // --------------------------------------------------------

    const float razaoGB =
        verde / azul;


    // --------------------------------------------------------
    // O cinza precisa ficar na faixa intermediária de
    // vermelho relativo ao Clear.
    // --------------------------------------------------------

    if(
        razaoRClear < LIMIAR_CINZA_R_CLEAR_MIN ||
        razaoRClear > LIMIAR_CINZA_R_CLEAR_MAX
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Verde e azul precisam ser relativamente próximos.
    // --------------------------------------------------------

    if(
        razaoGB < LIMIAR_CINZA_G_B_MIN ||
        razaoGB > LIMIAR_CINZA_G_B_MAX
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Verde tem prioridade.
    // --------------------------------------------------------

    if(
        detectarVerde(dados)
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Vermelho tem prioridade.
    // --------------------------------------------------------

    if(
        detectarVermelho(dados)
    )
    {
        return false;
    }


    return true;
}


// ============================================================
// ANALISAR
// ============================================================

AS7341Resultado AS7341Analise::analisar(
    const AS7341Data& dados
)
{
    AS7341Resultado resultado;

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


    // --------------------------------------------------------
    // SENSOR INVÁLIDO
    // --------------------------------------------------------

    if(!dados.valido)
    {
        return resultado;
    }


    resultado.valido = true;


    // --------------------------------------------------------
    // COMPONENTES
    // --------------------------------------------------------

    resultado.intensidade =
        calcularIntensidade(dados);

    resultado.azul =
        calcularAzul(dados);

    resultado.verde =
        calcularVerde(dados);

    resultado.vermelho =
        calcularVermelho(dados);


    // --------------------------------------------------------
    // RAZÃO VERMELHO / VERDE
    // --------------------------------------------------------

    if(resultado.verde > 0.0f)
    {
        resultado.razaoVermelhoVerde =
            resultado.vermelho /
            resultado.verde;
    }


    // --------------------------------------------------------
    // RAZÃO AZUL / VERDE
    // --------------------------------------------------------

    if(resultado.verde > 0.0f)
    {
        resultado.razaoAzulVerde =
            resultado.azul /
            resultado.verde;
    }


    // --------------------------------------------------------
    // RAZÃO NIR / CLEAR
    // --------------------------------------------------------

    if(dados.clear > 0)
    {
        resultado.razaoNIR =
            (float)dados.nir /
            (float)dados.clear;
    }


    // --------------------------------------------------------
    // CANAL DOMINANTE
    // --------------------------------------------------------

    resultado.canalDominante =
        descobrirCanalDominante(
            resultado.azul,
            resultado.verde,
            resultado.vermelho
        );


    // --------------------------------------------------------
    // DETECÇÃO
    // --------------------------------------------------------

    // Vermelho e verde são as cores mais específicas.
    resultado.verdeDetectado =
        detectarVerde(dados);

    resultado.vermelhoDetectado =
        detectarVermelho(dados);

    resultado.cinzaDetectado =
        detectarCinza(dados);


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