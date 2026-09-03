#include "TOF200F.h"


// ============================================================
// CONSTRUTOR
// ============================================================

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

    leiturasInvalidas = 0;


    for(
        int i = 0;
        i < NUM_LEITURAS;
        i++
    )
    {
        leituras[i] = 0;
    }
}


// ============================================================
// LIMPA FILTRO
// ============================================================

void TOF200F::limparFiltro()
{
    indiceLeitura = 0;

    totalSoma = 0;

    quantidadeLeituras = 0;

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


// ============================================================
// BEGIN
// ============================================================

bool TOF200F::begin(
    uint8_t canalTCA
)
{
    canal = canalTCA;


    // --------------------------------------------------------
    // Seleciona o canal
    // --------------------------------------------------------

    if(
        !tca.selecionarCanal(
            canal
        )
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Inicializa VL53L0X
    // --------------------------------------------------------

    if(
        !lox.begin(0x29)
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Configuração do tempo de medição
    // --------------------------------------------------------

    if(
        !lox.setMeasurementTimingBudgetMicroSeconds(
            20000
        )
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Estado inicial
    // --------------------------------------------------------

    limparFiltro();

    ultimaLeitura = 0;

    novaLeitura = false;

    ultimaLeituraValida = false;

    leiturasInvalidas = 0;


    // --------------------------------------------------------
    // Modo CONTÍNUO
    // --------------------------------------------------------

    if(
        !lox.startRangeContinuous(
            PERIODO_MEDICAO_MS
        )
    )
    {
        return false;
    }


    return true;
}


// ============================================================
// REINICIA MEDIÇÃO
// ============================================================

bool TOF200F::reiniciarMedicao()
{
    // --------------------------------------------------------
    // Para o modo contínuo atual
    // --------------------------------------------------------

    lox.stopRangeContinuous();

    delay(5);


    // --------------------------------------------------------
    // Limpa estado do sensor
    // --------------------------------------------------------

    limparFiltro();

    ultimaLeitura = 0;

    novaLeitura = false;

    ultimaLeituraValida = false;

    leiturasInvalidas = 0;


    // --------------------------------------------------------
    // Reinicia modo contínuo
    // --------------------------------------------------------

    return lox.startRangeContinuous(
        PERIODO_MEDICAO_MS
    );
}


// ============================================================
// UPDATE
// ============================================================

void TOF200F::update()
{
    // --------------------------------------------------------
    // Por padrão não há nova leitura.
    // --------------------------------------------------------

    novaLeitura = false;


    // --------------------------------------------------------
    // Seleciona novamente o canal do TCA.
    // --------------------------------------------------------

    if(
        !tca.selecionarCanal(
            canal
        )
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Ainda não terminou uma medição.
    // --------------------------------------------------------

    if(
        !lox.isRangeComplete()
    )
    {
        return;
    }


    // ========================================================
    // LEITURA CONTÍNUA
    // ========================================================
    //
    // IMPORTANTE:
    //
    // NÃO usar readRange() aqui.
    //
    // readRange() = single shot
    //
    // readRangeResult() = resultado do modo contínuo
    //
    // ========================================================

    uint16_t leituraBruta =
        lox.readRangeResult();


    // --------------------------------------------------------
    // Depois de readRangeResult(), o status da medição
    // fica disponível através de readRangeStatus().
    // --------------------------------------------------------

    uint8_t status =
        lox.readRangeStatus();


    novaLeitura = true;


    // ========================================================
    // LEITURA INVÁLIDA
    // ========================================================

    if(
        status != 0
    )
    {
        ultimaLeituraValida = false;

        ultimaLeitura = 0;

        leiturasInvalidas++;


        // ----------------------------------------------------
        // Não altera o filtro.
        //
        // Não transforma uma leitura inválida em distância.
        // ----------------------------------------------------


        // ----------------------------------------------------
        // Se o sensor ficar preso em erro por muitas leituras,
        // reinicia somente a medição.
        //
        // NÃO altera a lógica de obstáculo.
        // ----------------------------------------------------

        if(
            leiturasInvalidas >=
            LIMITE_INVALIDAS_REINICIO
        )
        {
            reiniciarMedicao();
        }


        return;
    }


    // ========================================================
    // LEITURA VÁLIDA
    // ========================================================

    ultimaLeituraValida = true;

    leiturasInvalidas = 0;


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
    // Guarda última leitura individual
    // --------------------------------------------------------

    ultimaLeitura =
        leituraCalibrada;


    // ========================================================
    // FILTRO DE MÉDIA MÓVEL
    // ========================================================

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


// ============================================================
// DISTÂNCIA
// ============================================================

int TOF200F::getDistance()
{
    return distanciaAtual;
}


// ============================================================
// ÚLTIMA LEITURA
// ============================================================

int TOF200F::getUltimaLeitura()
{
    return ultimaLeitura;
}


// ============================================================
// NOVA LEITURA
// ============================================================

bool TOF200F::temNovaLeitura()
{
    return novaLeitura;
}


// ============================================================
// VALIDADE
// ============================================================

bool TOF200F::ultimaLeituraValidaAgora()
{
    return ultimaLeituraValida;
}


// ============================================================
// RESET
// ============================================================

void TOF200F::reset()
{
    limparFiltro();

    ultimaLeitura = 0;

    novaLeitura = false;

    ultimaLeituraValida = false;

    leiturasInvalidas = 0;
}