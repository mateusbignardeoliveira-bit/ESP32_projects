#include "AS7341Analise.h"

#include <Arduino.h>
#include <math.h>

// ============================================================
// LIMIARES GERAIS DE INTENSIDADE
// ============================================================
//
// O Clear é usado principalmente para impedir que leituras
// muito baixas/ruidosas sejam classificadas como cor.
//
// Verde real medido anteriormente:
// Clear ≈ 16 ~ 19
//
// Preto:
// Clear ≈ 9 ~ 10
//
// Como agora usamos um score de verde, deixamos uma margem
// um pouco maior para tons de verde mais escuros.
// ============================================================

static const float LIMIAR_MIN_VERDE_CLEAR = 11.0f;
static const float LIMIAR_MIN_VERMELHO_CLEAR = 15.0f;

// ============================================================
// DETECÇÃO DE VERDE POR SCORE
// ============================================================
//
// Em vez de exigir que todos os critérios passem ao mesmo
// tempo, cada característica contribui para um score.
//
// Isso torna a detecção mais tolerante a:
//
// - verde claro
// - verde escuro
// - verde levemente amarelado
// - verde levemente azulado
// - diferenças de iluminação
// - pequenas diferenças entre os dois AS7341
//
// Características utilizadas:
//
// 1. R/G
//    Verde -> relativamente baixo
//
// 2. G/B
//    Verde -> relativamente alto
//
// 3. F4/F6
//    Verde -> F4 tende a superar F6
//
// O score final varia aproximadamente de 0 a 1.
//
// Quanto maior, mais características de verde a leitura possui.
// ============================================================

// ------------------------------------------------------------
// R/G
// ------------------------------------------------------------
//
// Muito verde:
//     <= 0.80
//
// Verde aceitável:
//     ~1.00
//
// Verde claro/amarelado:
//     ~1.20 ~ 1.30
//
// Próximo de branco:
//     ~1.50+
//
// Acima de 1.45 consideramos que a característica deixou de
// contribuir para a detecção de verde.
// ------------------------------------------------------------

static const float VERDE_RG_MUITO_FORTE = 0.80f;
static const float VERDE_RG_FRACO = 1.45f;

// ------------------------------------------------------------
// G/B
// ------------------------------------------------------------
//
// Verde forte:
//     >= 1.80
//
// Verde moderado:
//     ~1.40
//
// Verde mais difícil:
//     ~1.15
//
// Abaixo de 1.05 praticamente não há característica verde.
// ------------------------------------------------------------

static const float VERDE_GB_MUITO_FORTE = 1.80f;
static const float VERDE_GB_FRACO = 1.05f;

// ------------------------------------------------------------
// F4/F6
// ------------------------------------------------------------
//
// Verde forte:
//     >= 1.40
//
// Verde moderado:
//     ~1.10
//
// Verde mais difícil:
//     ~0.90
//
// Abaixo de 0.75 a característica deixa de contribuir.
// ------------------------------------------------------------

static const float VERDE_F4F6_MUITO_FORTE = 1.40f;
static const float VERDE_F4F6_FRACO = 0.75f;

// ------------------------------------------------------------
// SCORE MÍNIMO
// ------------------------------------------------------------
//
// Antes:
//     os 3 critérios precisavam passar.
//
// Agora:
//     os 3 critérios contribuem com pesos.
//
// 0.55 permite detectar tonalidades diferentes sem deixar
// a classificação excessivamente permissiva.
// ------------------------------------------------------------

static const float SCORE_MINIMO_VERDE = 0.55f;

// ============================================================
// PROTEÇÃO CONTRA BRANCO
// ============================================================
//
// O branco medido apresentou aproximadamente:
//
// R/G   ≈ 1.56 ~ 1.80
// G/B   ≈ 1.25 ~ 1.30
// F4/F6 ≈ 0.66 ~ 0.75
//
// Portanto não basta apenas ter G/B razoável.
//
// O vermelho também não pode estar muito próximo do verde
// quando o F4/F6 estiver baixo.
//
// Essa proteção evita transformar branco iluminado em verde.
// ============================================================

static const float LIMIAR_BRANCO_RG = 1.50f;
static const float LIMIAR_BRANCO_F4F6 = 0.82f;

// ============================================================
// LIMIARES DE VERMELHO
// ============================================================

static const float LIMIAR_VERMELHO_R_G_MIN = 3.0f;
static const float LIMIAR_VERMELHO_R_B_MIN = 3.0f;
static const float LIMIAR_VERMELHO_R_CLEAR_MIN = 0.56f;

// ============================================================
// FUNÇÃO AUXILIAR
// ============================================================
//
// Converte uma característica em score de 0 a 1.
//
// menor -> melhor
//
// Exemplo:
//
// valor <= melhor  -> 1.0
// valor >= pior    -> 0.0
//
// Entre os dois:
// interpolação linear.
// ============================================================

static float calcularScoreMenorMelhor(
float valor,
float melhor,
float pior
)
{
if(valor <= melhor)
{
return 1.0f;
}

if(valor >= pior)
{
    return 0.0f;
}

return 1.0f -
    (
        (valor - melhor) /
        (pior - melhor)
    );

}

// ============================================================
// FUNÇÃO AUXILIAR
// ============================================================
//
// Converte uma característica em score de 0 a 1.
//
// maior -> melhor
// ============================================================

static float calcularScoreMaiorMelhor(
float valor,
float melhor,
float pior
)
{
if(valor >= melhor)
{
return 1.0f;
}

if(valor <= pior)
{
    return 0.0f;
}

return (
    valor - pior
) /
(
    melhor - pior
);

}

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
// A detecção agora utiliza SCORE.
//
// Isso é propositalmente diferente da versão anterior:
//
// ANTES:
//
//     R/G < limite
//     E
//     G/B > limite
//     E
//     F4/F6 > limite
//
// AGORA:
//
//     cada característica recebe uma nota
//     e a combinação das três determina o resultado.
//
// Isso permite que uma característica esteja um pouco pior
// sem necessariamente eliminar um verde verdadeiro.
// ============================================================

bool AS7341Analise::detectarVerde(
const AS7341Data& dados
)
{
if(!dados.valido)
return false;

// --------------------------------------------------------
// Componentes espectrais
// --------------------------------------------------------

const float azul =
    calcularAzul(dados);

const float verde =
    calcularVerde(dados);

const float vermelho =
    calcularVermelho(dados);

const float clear =
    calcularIntensidade(dados);


// --------------------------------------------------------
// Canais individuais
// --------------------------------------------------------

const float f4 =
    (float)dados.F4;

const float f6 =
    (float)dados.F6;


// --------------------------------------------------------
// Leitura muito escura
// --------------------------------------------------------

if(
    clear < LIMIAR_MIN_VERDE_CLEAR
)
{
    return false;
}


// --------------------------------------------------------
// Valores inválidos
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
// RAZÃO R/G
// --------------------------------------------------------

const float razaoRG =
    vermelho /
    verde;


// --------------------------------------------------------
// RAZÃO G/B
// --------------------------------------------------------

const float razaoGB =
    verde /
    azul;


// --------------------------------------------------------
// RAZÃO F4/F6
// --------------------------------------------------------

const float razaoF4F6 =
    f4 /
    f6;


// ========================================================
// SCORE R/G
// ========================================================
//
// Quanto menor R/G, melhor para verde.
// ========================================================

const float scoreRG =
    calcularScoreMenorMelhor(
        razaoRG,
        VERDE_RG_MUITO_FORTE,
        VERDE_RG_FRACO
    );


// ========================================================
// SCORE G/B
// ========================================================
//
// Quanto maior G/B, melhor para verde.
// ========================================================

const float scoreGB =
    calcularScoreMaiorMelhor(
        razaoGB,
        VERDE_GB_MUITO_FORTE,
        VERDE_GB_FRACO
    );


// ========================================================
// SCORE F4/F6
// ========================================================
//
// Quanto maior F4/F6, melhor para verde.
// ========================================================

const float scoreF4F6 =
    calcularScoreMaiorMelhor(
        razaoF4F6,
        VERDE_F4F6_MUITO_FORTE,
        VERDE_F4F6_FRACO
    );


// ========================================================
// SCORE FINAL
// ========================================================
//
// Pesos:
//
// R/G   -> 40%
// G/B   -> 35%
// F4/F6 -> 25%
//
// R/G recebe um peso ligeiramente maior porque ajuda
// bastante a separar verde de branco.
// ========================================================

const float scoreVerde =
    (
        scoreRG * 0.40f
    ) +
    (
        scoreGB * 0.35f
    ) +
    (
        scoreF4F6 * 0.25f
    );


// ========================================================
// PROTEÇÃO CONTRA BRANCO
// ========================================================
//
// Se R/G já estiver muito alto e F4/F6 muito baixo,
// temos uma combinação típica de branco.
//
// Nesse caso rejeitamos mesmo que o score geral esteja
// próximo do limite.
// ========================================================

if(
    razaoRG >= LIMIAR_BRANCO_RG &&
    razaoF4F6 <= LIMIAR_BRANCO_F4F6
)
{
    return false;
}


// ========================================================
// CARACTERÍSTICA MÍNIMA DE VERDE
// ========================================================
//
// Evita que um único critério excepcionalmente bom
// transforme uma leitura completamente diferente em verde.
//
// Pelo menos duas das três características precisam
// possuir alguma contribuição significativa.
// ========================================================

int caracteristicasVerdes = 0;


if(scoreRG >= 0.30f)
{
    caracteristicasVerdes++;
}

if(scoreGB >= 0.30f)
{
    caracteristicasVerdes++;
}

if(scoreF4F6 >= 0.30f)
{
    caracteristicasVerdes++;
}


if(
    caracteristicasVerdes < 2
)
{
    return false;
}


// ========================================================
// DECISÃO FINAL
// ========================================================

if(
    scoreVerde >= SCORE_MINIMO_VERDE
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
// Vermelho real:
//
// R/G ≈ 6
// R/B ≈ 6
// R/Clear ≈ 0.63
//
// Mantemos a lógica atual.
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
// Intensidade mínima
// --------------------------------------------------------

if(
    clear < LIMIAR_MIN_VERMELHO_CLEAR
)
{
    return false;
}


// --------------------------------------------------------
// Valores válidos
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
// R/G
// --------------------------------------------------------

const float razaoRG =
    vermelho /
    verde;


// --------------------------------------------------------
// R/B
// --------------------------------------------------------

const float razaoRB =
    vermelho /
    azul;


// --------------------------------------------------------
// R/Clear
// --------------------------------------------------------

const float razaoRClear =
    vermelho /
    clear;


// --------------------------------------------------------
// Vermelho dominante
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

resultado.verdeDetectado =
    detectarVerde(dados);

resultado.vermelhoDetectado =
    detectarVermelho(dados);


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
