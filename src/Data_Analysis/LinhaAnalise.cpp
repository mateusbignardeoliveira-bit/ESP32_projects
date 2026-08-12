#include "LinhaAnalise.h"


LinhaAnalise::LinhaAnalise()
{
    resultado.linhaDetectada = false;

    resultado.posicao = 0.0f;

    resultado.erro = 0.0f;

    resultado.intensidade = 0.0f;

    resultado.largura = 0.0f;


    for(int i = 0; i < 8; i++)
    {
        resultado.sensores[i] = 0.0f;
    }
}


float LinhaAnalise::normalizarSensor(
    int valor,
    int indice
)
{
    int limiteBranco =
        branco[indice];

    int limitePreto =
        preto[indice];


    if(limitePreto <= limiteBranco)
    {
        return 0.0f;
    }


    float normalizado =
        (float)(valor - limiteBranco) /
        (float)(limitePreto - limiteBranco);


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


void LinhaAnalise::update(
    const ArrayData& dados
)
{
    int valores[8] =
    {
        dados.s1,
        dados.s2,
        dados.s3,
        dados.s4,
        dados.s5,
        dados.s6,
        dados.s7,
        dados.s8
    };


    float somaIntensidade = 0.0f;

    float somaPosicao = 0.0f;

    float maiorIntensidade = 0.0f;

    int sensoresAtivos = 0;


    for(int i = 0; i < 8; i++)
    {
        resultado.sensores[i] =
            normalizarSensor(
                valores[i],
                i
            );


        somaIntensidade +=
            resultado.sensores[i];


        somaPosicao +=
            resultado.sensores[i] *
            pesos[i];


        if(
            resultado.sensores[i] >
            maiorIntensidade
        )
        {
            maiorIntensidade =
                resultado.sensores[i];
        }


        if(
            resultado.sensores[i] >= 0.20f
        )
        {
            sensoresAtivos++;
        }
    }


    // Detecta a linha

    resultado.linhaDetectada =
        maiorIntensidade >= 0.08f;


    // Calcula posição

    if(somaIntensidade > 0.01f)
    {
        resultado.posicao =
            somaPosicao /
            somaIntensidade;
    }
    else
    {
        resultado.posicao = 0.0f;
    }


    // Erro utilizado pelo PID

    resultado.erro =
        resultado.posicao;


    // Intensidade total

    resultado.intensidade =
        somaIntensidade;


    // Quantidade de sensores ativos

    resultado.largura =
        (float)sensoresAtivos;
}


LinhaData LinhaAnalise::getData()
{
    return resultado;
}