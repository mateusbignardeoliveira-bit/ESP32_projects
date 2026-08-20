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


    // --------------------------------------------------------
    // 20 ms
    // --------------------------------------------------------

    lox.setMeasurementTimingBudgetMicroSeconds(
        20000
    );


    // --------------------------------------------------------
    // Inicializa filtro
    // --------------------------------------------------------

    indiceLeitura = 0;

    totalSoma = 0;

    distanciaAtual = 0;


    for(
        int i = 0;
        i < NUM_LEITURAS;
        i++
    )
    {
        leituras[i] = 0;
    }


    // --------------------------------------------------------
    // Modo contínuo
    // --------------------------------------------------------

    lox.startRangeContinuous(
        20
    );


    return true;
}


void TOF200F::update()
{
    // --------------------------------------------------------
    // Seleciona canal somente se necessário
    // --------------------------------------------------------

    if(!tca.selecionarCanal(
        canal
    ))
    {
        return;
    }


    // --------------------------------------------------------
    // Ainda não terminou?
    // Sai imediatamente.
    // --------------------------------------------------------

    if(!lox.isRangeComplete())
    {
        return;
    }


    uint16_t leituraBruta =
        lox.readRange();


    if(
        lox.readRangeStatus() == 4
    )
    {
        return;
    }


    int leituraCalibrada =
        (int)leituraBruta +
        OFFSET_CORRECAO_MM;


    if(
        leituraCalibrada < 0
    )
    {
        leituraCalibrada = 0;
    }


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