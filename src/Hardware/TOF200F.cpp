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

    quantidadeLeituras = 0;

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

    quantidadeLeituras = 0;

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
    // Por padrão não existe nova leitura nesta chamada.
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
    // Ainda não terminou uma nova medição
    // --------------------------------------------------------

    if(!lox.isRangeComplete())
    {
        return;
    }


    // --------------------------------------------------------
    // Lê a medição
    // --------------------------------------------------------

    uint16_t leituraBruta =
        lox.readRange();


    // A partir daqui existe uma nova medição,
    // mesmo que ela seja inválida.

    novaLeitura = true;


    // --------------------------------------------------------
    // Verifica o status REAL da medição
    // --------------------------------------------------------

    uint8_t status =
        lox.readRangeStatus();


    // --------------------------------------------------------
    // SOMENTE STATUS 0 É ACEITO
    //
    // Status 0 = Range Valid
    //
    // Status 2 = Signal Fail
    // Status 4 = Phase Fail
    //
    // Leituras com qualquer outro status não entram
    // no filtro e não podem confirmar obstáculo.
    // --------------------------------------------------------

    if(status != 0)
    {
        ultimaLeitura = 0;

        ultimaLeituraValida = false;

        return;
    }


    ultimaLeituraValida = true;


    // --------------------------------------------------------
    // Aplica correção de calibração
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
    // Guarda a última leitura individual
    // --------------------------------------------------------

    ultimaLeitura =
        leituraCalibrada;


    // --------------------------------------------------------
    // Filtro de média móvel
    //
    // IMPORTANTE:
    // somente leituras válidas entram no filtro.
    // --------------------------------------------------------

    if(
        quantidadeLeituras < NUM_LEITURAS
    )
    {
        leituras[indiceLeitura] =
            leituraCalibrada;

        totalSoma +=
            leituraCalibrada;

        quantidadeLeituras++;

        indiceLeitura++;

        if(
            indiceLeitura >= NUM_LEITURAS
        )
        {
            indiceLeitura = 0;
        }
    }
    else
    {
        totalSoma -=
            leituras[indiceLeitura];

        leituras[indiceLeitura] =
            leituraCalibrada;

        totalSoma +=
            leituraCalibrada;

        indiceLeitura++;

        if(
            indiceLeitura >= NUM_LEITURAS
        )
        {
            indiceLeitura = 0;
        }
    }


    // --------------------------------------------------------
    // Atualiza distância filtrada
    // --------------------------------------------------------

    if(
        quantidadeLeituras > 0
    )
    {
        distanciaAtual =
            totalSoma /
            quantidadeLeituras;
    }
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