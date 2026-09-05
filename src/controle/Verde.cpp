#include "Verde.h"


// ============================================================
// CONSTRUTOR
// ============================================================

Verde::Verde(
    AS7341Analise& analise
)
    :
    analise(analise)
{
    reset();
}


// ============================================================
// RESET
// ============================================================

void Verde::reset()
{
    resultado.avaliando = false;

    resultado.finalizada = false;

    resultado.verdeEsquerda = false;

    resultado.verdeDireita = false;

    resultado.vermelhoEsquerda = false;

    resultado.vermelhoDireita = false;

    resultado.cinzaEsquerda = false;

    resultado.cinzaDireita = false;

    resultado.maiorQuantidadePretos = 0;

    resultado.encontrouAmbosLados = false;

    resultado.encontrouVermelho = false;

    resultado.encontrouCinza = false;
}


// ============================================================
// INICIAR AVALIAÇÃO
// ============================================================

void Verde::iniciar(
    int quantidadePretosInicial
)
{
    reset();

    resultado.avaliando = true;

    resultado.maiorQuantidadePretos =
        quantidadePretosInicial;
}


// ============================================================
// UPDATE
// ============================================================

void Verde::update(
    const AS7341Data& dadosEsquerda,
    const AS7341Data& dadosDireita,
    int quantidadePretos
)
{
    if(!resultado.avaliando)
    {
        return;
    }


    // ========================================================
    // ANALISA SENSOR ESQUERDO
    // ========================================================

    AS7341Resultado esquerda =
        analise.analisar(
            dadosEsquerda
        );


    // ========================================================
    // ANALISA SENSOR DIREITO
    // ========================================================

    AS7341Resultado direita =
        analise.analisar(
            dadosDireita
        );


    // ========================================================
    // VERDE ESQUERDO
    // ========================================================

    if(
        esquerda.valido &&
        esquerda.verdeDetectado
    )
    {
        resultado.verdeEsquerda = true;
    }


    // ========================================================
    // VERDE DIREITO
    // ========================================================

    if(
        direita.valido &&
        direita.verdeDetectado
    )
    {
        resultado.verdeDireita = true;
    }


    // ========================================================
    // VERDE NOS DOIS LADOS
    // ========================================================
    //
    // Não precisa ser simultâneo.
    //
    // Exemplo:
    //
    // instante 1 -> verde esquerda
    // instante 20 -> verde direita
    //
    // Resultado:
    //
    // encontrouAmbosLados = true
    //
    // A camada de decisão posteriormente interpreta isso
    // como uma curva de 180 graus.
    //

    if(
        resultado.verdeEsquerda &&
        resultado.verdeDireita
    )
    {
        resultado.encontrouAmbosLados = true;
    }


    // ========================================================
    // VERMELHO ESQUERDO
    // ========================================================

    if(
        esquerda.valido &&
        esquerda.vermelhoDetectado
    )
    {
        resultado.vermelhoEsquerda = true;

        resultado.encontrouVermelho = true;
    }


    // ========================================================
    // VERMELHO DIREITO
    // ========================================================

    if(
        direita.valido &&
        direita.vermelhoDetectado
    )
    {
        resultado.vermelhoDireita = true;

        resultado.encontrouVermelho = true;
    }


    // ========================================================
    // MAIOR QUANTIDADE DE PRETOS
    // ========================================================

    if(
        quantidadePretos >
        resultado.maiorQuantidadePretos
    )
    {
        resultado.maiorQuantidadePretos =
            quantidadePretos;
    }
}


// ============================================================
// FINALIZAR
// ============================================================

void Verde::finalizar()
{
    if(!resultado.avaliando)
    {
        return;
    }

    resultado.avaliando = false;

    resultado.finalizada = true;
}


// ============================================================
// ESTA AVALIANDO
// ============================================================

bool Verde::estaAvaliando() const
{
    return resultado.avaliando;
}


// ============================================================
// FINALIZADO
// ============================================================

bool Verde::finalizado() const
{
    return resultado.finalizada;
}


// ============================================================
// VERDE ESQUERDA
// ============================================================

bool Verde::detectouVerdeEsquerda() const
{
    return resultado.verdeEsquerda;
}


// ============================================================
// VERDE DIREITA
// ============================================================

bool Verde::detectouVerdeDireita() const
{
    return resultado.verdeDireita;
}


// ============================================================
// VERDE DOS DOIS LADOS
// ============================================================

bool Verde::detectouVerdeDosDoisLados() const
{
    return resultado.encontrouAmbosLados;
}


// ============================================================
// VERMELHO
// ============================================================

bool Verde::detectouVermelho() const
{
    return resultado.encontrouVermelho;
}


// ============================================================
// CINZA
// ============================================================

bool Verde::detectouCinza() const
{
    return resultado.encontrouCinza;
}


// ============================================================
// MAIOR QUANTIDADE DE PRETOS
// ============================================================

int Verde::getMaiorQuantidadePretos() const
{
    return resultado.maiorQuantidadePretos;
}


// ============================================================
// DATA COMPLETA
// ============================================================

AvaliacaoCorData Verde::getData() const
{
    return resultado;
}