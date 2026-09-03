#include "TOF200F.h"


TOF200F::TOF200F(
    TCA& multiplexador
)
:
tca(multiplexador)
{
    canal = 0;

    indiceLeitura = 0;

    totalSoma = 0;

    distanciaAtual = 0;

    ultimaLeitura = 0;

    novaLeitura = false;

    ultimaLeituraValida = false;


    for(
        int i = 0;
        i < NUM_LEITURAS;
        i++
    )
    {
        leituras[i] = 0;
    }
}


bool TOF200F::begin(
    uint8_t canalTCA
)
{
    canal = canalTCA;


    if(!tca.selecionarCanal(
        canal
    ))
    {
        return false;
    }


    if(!lox.begin(0x29))
    {
        return false;
    }


    lox.setMeasurementTimingBudgetMicroSeconds(
        20000
    );


    indiceLeitura = 0;

    totalSoma = 0;

    distanciaAtual = 0;

    ultimaLeitura = 0;

    novaLeitura = false;

    ultimaLeituraValida = false;


    for(
        int i = 0;
        i < NUM_LEITURAS;
        i++
    )
    {
        leituras[i] = 0;
    }


    lox.startRangeContinuous(
        20
    );


    return true;
}


void TOF200F::update()
{
    // --------------------------------------------------------
    // Por padrão não existe uma leitura nova nesta chamada.
    // --------------------------------------------------------

    novaLeitura = false;


    // --------------------------------------------------------
    // Seleciona o canal do TCA
    // --------------------------------------------------------

    if(!tca.selecionarCanal(
        canal
    ))
    {
        return;
    }


    // --------------------------------------------------------
    // Ainda não existe uma nova medição
    // --------------------------------------------------------

    if(!lox.isRangeComplete())
    {
        return;
    }


    // --------------------------------------------------------
    // Lê uma medição REAL do VL53L0X
    // --------------------------------------------------------

    uint16_t leituraBruta =
        lox.readRange();


    novaLeitura = true;


    // --------------------------------------------------------
    // Verifica erro
    // --------------------------------------------------------

    if(
        lox.readRangeStatus() == 4
    )
    {
        ultimaLeitura = 0;

        ultimaLeituraValida = false;

        return;
    }


    ultimaLeituraValida = true;


    // --------------------------------------------------------
    // Aplica offset
    // --------------------------------------------------------

    int leituraCalibrada =
        (int)leituraBruta +
        OFFSET_CORRECAO_MM;


    if(
        leituraCalibrada < 0
    )
    {
        leituraCalibrada = 0;
    }


    // --------------------------------------------------------
    // Guarda leitura individual
    // --------------------------------------------------------

    ultimaLeitura =
        leituraCalibrada;


    // --------------------------------------------------------
    // Filtro de 3 leituras para getDistance()
    // --------------------------------------------------------

    totalSoma -=
        leituras[indiceLeitura];


    leituras[indiceLeitura] =
        leituraCalibrada;


    totalSoma +=
        leituraCalibrada;


    indiceLeitura++;


    if(
        indiceLeitura >=
        NUM_LEITURAS
    )
    {
        indiceLeitura = 0;
    }


    distanciaAtual =
        totalSoma /
        NUM_LEITURAS;
}


int TOF200F::getDistance()
{
    return distanciaAtual;
}


int TOF200F::getUltimaLeitura()
{
    return ultimaLeitura;
}


bool TOF200F::temNovaLeitura()
{
    return novaLeitura;
}


bool TOF200F::ultimaLeituraValidaAgora()
{
    return ultimaLeituraValida;
}