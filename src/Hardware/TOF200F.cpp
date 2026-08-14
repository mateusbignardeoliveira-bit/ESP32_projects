#include "TOF200F.h"


// ============================================================
// CONSTRUTOR
// ============================================================

TOF200F::TOF200F(TCA& multiplexador)
:
tca(multiplexador)
{

    canal = 0;

    indiceLeitura = 0;

    totalSoma = 0;

    distanciaAtual = 0;


    for(int i = 0; i < NUM_LEITURAS; i++)
    {
        leituras[i] = 0;
    }

}


// ============================================================
// BEGIN
// ============================================================

bool TOF200F::begin(uint8_t canalTCA)
{

    canal = canalTCA;


    // --------------------------------------------------------
    // Seleciona canal do TCA
    // --------------------------------------------------------

    if(!tca.selecionarCanal(canal))
    {
        return false;
    }


    // --------------------------------------------------------
    // Inicializa VL53L0X
    // --------------------------------------------------------

    if(!lox.begin(0x29))
    {
        return false;
    }


    // --------------------------------------------------------
    // Timing
    //
    // 33 ms é muito mais adequado para o robô do que
    // os 200 ms anteriores.
    // --------------------------------------------------------

    lox.setMeasurementTimingBudgetMicroSeconds(33000);


    // --------------------------------------------------------
    // Inicializa filtro
    // --------------------------------------------------------

    indiceLeitura = 0;

    totalSoma = 0;

    distanciaAtual = 0;


    for(int i = 0; i < NUM_LEITURAS; i++)
    {
        leituras[i] = 0;
    }


    return true;

}


// ============================================================
// UPDATE
// ============================================================

void TOF200F::update()
{

    // --------------------------------------------------------
    // Seleciona o canal do TOF
    // --------------------------------------------------------

    if(!tca.selecionarCanal(canal))
    {
        return;
    }


    VL53L0X_RangingMeasurementData_t medida;


    // --------------------------------------------------------
    // Faz uma medição
    // --------------------------------------------------------

    lox.rangingTest(
        &medida,
        false
    );


    // --------------------------------------------------------
    // Verifica validade
    // --------------------------------------------------------

    if(medida.RangeStatus == 4)
    {
        // Não atualiza a distância.
        // Mantém a última leitura válida.

        return;
    }


    int leituraBruta =
        medida.RangeMilliMeter;


    // --------------------------------------------------------
    // Correção
    // --------------------------------------------------------

    int leituraCalibrada =
        leituraBruta + OFFSET_CORRECAO_MM;


    if(leituraCalibrada < 0)
    {
        leituraCalibrada = 0;
    }


    // --------------------------------------------------------
    // Filtro de média móvel
    // --------------------------------------------------------

    totalSoma -=
        leituras[indiceLeitura];


    leituras[indiceLeitura] =
        leituraCalibrada;


    totalSoma +=
        leituras[indiceLeitura];


    indiceLeitura++;


    if(indiceLeitura >= NUM_LEITURAS)
    {
        indiceLeitura = 0;
    }


    // --------------------------------------------------------
    // Média
    // --------------------------------------------------------

    distanciaAtual =
        totalSoma / NUM_LEITURAS;

}


// ============================================================
// GET DISTANCE
// ============================================================

int TOF200F::getDistance()
{

    return distanciaAtual;

}