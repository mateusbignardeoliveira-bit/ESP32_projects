#include "AS7341.h"


// ============================================================
// CONSTRUTOR
// ============================================================

AS7341Sensores::AS7341Sensores(
    TCA& controladorTCA
)
:
tca(controladorTCA)
{

    dadosDireita.valido = false;
    dadosDireita.tempo = 0;


    dadosEsquerda.valido = false;
    dadosEsquerda.tempo = 0;

}



// ============================================================
// CONFIGURAR SENSOR
// ============================================================

bool AS7341Sensores::configurarSensor(
    Adafruit_AS7341& sensor
)
{

    // --------------------------------------------------------
    // Inicializa o sensor
    // --------------------------------------------------------

    if(!sensor.begin())
    {
        return false;
    }


    // --------------------------------------------------------
    // Tempo de integração
    // --------------------------------------------------------

    sensor.setATIME(ATIME);

    sensor.setASTEP(ASTEP);


    // --------------------------------------------------------
    // Ganho
    // --------------------------------------------------------

    sensor.setGain(
        AS7341_GAIN_32X
    );


    // --------------------------------------------------------
    // LED interno desligado
    // --------------------------------------------------------

    sensor.enableLED(false);


    return true;

}



// ============================================================
// BEGIN
// ============================================================

bool AS7341Sensores::begin()
{

    // ========================================================
    // SENSOR DIREITA
    // ========================================================

    if(!tca.selecionarCanal(CANAL_DIREITA))
    {
        return false;
    }


    if(!configurarSensor(sensorDireita))
    {
        return false;
    }


    // ========================================================
    // SENSOR ESQUERDA
    // ========================================================

    if(!tca.selecionarCanal(CANAL_ESQUERDA))
    {
        return false;
    }


    if(!configurarSensor(sensorEsquerda))
    {
        return false;
    }


    // --------------------------------------------------------
    // Nenhum sensor fica selecionado ao terminar
    // --------------------------------------------------------

    tca.desligarCanais();


    return true;

}



// ============================================================
// ARMAZENAR LEITURA
// ============================================================

void AS7341Sensores::armazenarLeitura(
    AS7341Data& destino,
    uint16_t* readings
)
{

    // --------------------------------------------------------
    // F1 - F4
    // --------------------------------------------------------

    destino.F1 = readings[0];
    destino.F2 = readings[1];
    destino.F3 = readings[2];
    destino.F4 = readings[3];


    // --------------------------------------------------------
    // F5 - F8
    //
    // A biblioteca possui os canais auxiliares nos índices
    // 4 e 5, portanto os canais espectrais que queremos
    // ficam em 6, 7, 8 e 9.
    // --------------------------------------------------------

    destino.F5 = readings[6];
    destino.F6 = readings[7];
    destino.F7 = readings[8];
    destino.F8 = readings[9];


    // --------------------------------------------------------
    // Clear / NIR
    // --------------------------------------------------------

    destino.clear = readings[10];
    destino.nir = readings[11];


    destino.valido = true;
    destino.tempo = millis();

}



// ============================================================
// UPDATE
// ============================================================

bool AS7341Sensores::update()
{

    uint16_t readings[12];


    // ========================================================
    // DIREITA
    // ========================================================

    if(!tca.selecionarCanal(CANAL_DIREITA))
    {
        return false;
    }


    if(!sensorDireita.readAllChannels(readings))
    {
        return false;
    }


    armazenarLeitura(
        dadosDireita,
        readings
    );


    // ========================================================
    // ESQUERDA
    // ========================================================

    if(!tca.selecionarCanal(CANAL_ESQUERDA))
    {
        return false;
    }


    if(!sensorEsquerda.readAllChannels(readings))
    {
        return false;
    }


    armazenarLeitura(
        dadosEsquerda,
        readings
    );


    // --------------------------------------------------------
    // Desliga os canais após a leitura
    // --------------------------------------------------------

    tca.desligarCanais();


    return true;

}



// ============================================================
// GET DIREITA
// ============================================================

AS7341Data AS7341Sensores::getDireita()
{

    return dadosDireita;

}



// ============================================================
// GET ESQUERDA
// ============================================================

AS7341Data AS7341Sensores::getEsquerda()
{

    return dadosEsquerda;

}