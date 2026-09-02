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
    resultado.avaliando =
        false;

    resultado.finalizada =
        false;

    resultado.verdeEsquerda =
        false;

    resultado.verdeDireita =
        false;

    resultado.vermelhoEsquerda =
        false;

    resultado.vermelhoDireita =
        false;

    resultado.cinzaEsquerda =
        false;

    resultado.cinzaDireita =
        false;

    resultado.maiorQuantidadePretos =
        0;

    resultado.encontrouAmbosLados =
        false;

    resultado.encontrouVermelho =
        false;

    resultado.encontrouCinza =
        false;
}


// ============================================================
// INICIAR AVALIAÇÃO
// ============================================================

void Verde::iniciar(
    int quantidadePretosInicial
)
{
    reset();

    resultado.avaliando =
        true;

    resultado.maiorQuantidadePretos =
        quantidadePretosInicial;

    Serial.println(
        "INICIO DA AVALIACAO DE COR"
    );
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
    if(
        !resultado.avaliando
    )
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
        esquerda.verdeDetectado
    )
    {
        if(
            !resultado.verdeEsquerda
        )
        {
            Serial.println(
                "VERDE ESQUERDA"
            );
        }

        resultado.verdeEsquerda =
            true;
    }


    // ========================================================
    // VERDE DIREITO
    // ========================================================

    if(
        direita.verdeDetectado
    )
    {
        if(
            !resultado.verdeDireita
        )
        {
            Serial.println(
                "VERDE DIREITA"
            );
        }

        resultado.verdeDireita =
            true;
    }


    // ========================================================
    // VERMELHO ESQUERDO
    // ========================================================

    if(
        esquerda.vermelhoDetectado
    )
    {
        resultado.vermelhoEsquerda =
            true;

        resultado.encontrouVermelho =
            true;
    }


    // ========================================================
    // VERMELHO DIREITO
    // ========================================================

    if(
        direita.vermelhoDetectado
    )
    {
        resultado.vermelhoDireita =
            true;

        resultado.encontrouVermelho =
            true;
    }


    // ========================================================
    // CINZA ESQUERDO
    // ========================================================

    if(
        esquerda.cinzaDetectado
    )
    {
        resultado.cinzaEsquerda =
            true;

        resultado.encontrouCinza =
            true;
    }


    // ========================================================
    // CINZA DIREITO
    // ========================================================

    if(
        direita.cinzaDetectado
    )
    {
        resultado.cinzaDireita =
            true;

        resultado.encontrouCinza =
            true;
    }


    // ========================================================
    // AMBOS OS LADOS
    // ========================================================
    //
    // IMPORTANTE:
    //
    // Não precisa acontecer simultaneamente.
    //
    // Exemplo:
    //
    // leitura 1:
    // verde esquerda
    //
    // leitura 20:
    // verde direita
    //
    // resultado:
    //
    // encontrouAmbosLados = true
    //
    // Isso será interpretado depois como 180°.
    // ========================================================

    if(
        resultado.verdeEsquerda &&
        resultado.verdeDireita
    )
    {
        resultado.encontrouAmbosLados =
            true;
    }


    // ========================================================
    // GUARDA MAIOR QUANTIDADE DE PRETOS
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
    if(
        !resultado.avaliando
    )
    {
        return;
    }


    resultado.avaliando =
        false;

    resultado.finalizada =
        true;


    Serial.println(
        "AVALIACAO DE COR FINALIZADA"
    );


    Serial.print(
        "VERDE ESQUERDA: "
    );

    Serial.println(
        resultado.verdeEsquerda
    );


    Serial.print(
        "VERDE DIREITA: "
    );

    Serial.println(
        resultado.verdeDireita
    );


    Serial.print(
        "VERMELHO: "
    );

    Serial.println(
        resultado.encontrouVermelho
    );


    Serial.print(
        "CINZA: "
    );

    Serial.println(
        resultado.encontrouCinza
    );


    Serial.print(
        "MAIOR NUMERO DE PRETOS: "
    );

    Serial.println(
        resultado.maiorQuantidadePretos
    );
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