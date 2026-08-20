#include "AS7341.h"


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

    lendoDireita = false;
    lendoEsquerda = false;
}


bool AS7341Sensores::configurarSensor(
    Adafruit_AS7341& sensor
)
{
    if(!sensor.begin())
    {
        return false;
    }

    sensor.setATIME(
        ATIME
    );

    sensor.setASTEP(
        ASTEP
    );

    sensor.setGain(
        GANHO
    );

    sensor.enableLED(
        false
    );

    return true;
}


bool AS7341Sensores::begin()
{
    // --------------------------------------------------------
    // DIREITA
    // --------------------------------------------------------

    if(!tca.selecionarCanal(
        CANAL_DIREITA
    ))
    {
        return false;
    }

    if(!configurarSensor(
        sensorDireita
    ))
    {
        return false;
    }


    // --------------------------------------------------------
    // ESQUERDA
    // --------------------------------------------------------

    if(!tca.selecionarCanal(
        CANAL_ESQUERDA
    ))
    {
        return false;
    }

    if(!configurarSensor(
        sensorEsquerda
    ))
    {
        return false;
    }


    // --------------------------------------------------------
    // Começa imediatamente a primeira aquisição
    // --------------------------------------------------------

    if(!tca.selecionarCanal(
        CANAL_DIREITA
    ))
    {
        return false;
    }

    sensorDireita.startReading();

    lendoDireita = true;


    return true;
}


void AS7341Sensores::armazenarLeitura(
    AS7341Data& destino,
    uint16_t* readings
)
{
    destino.F1 = readings[0];
    destino.F2 = readings[1];
    destino.F3 = readings[2];
    destino.F4 = readings[3];

    destino.F5 = readings[6];
    destino.F6 = readings[7];
    destino.F7 = readings[8];
    destino.F8 = readings[9];

    destino.clear = readings[10];
    destino.nir = readings[11];

    destino.valido = true;
    destino.tempo = millis();
}


bool AS7341Sensores::update()
{
    uint16_t readings[12];

    bool houveLeitura = false;


    // ========================================================
    // DIREITA
    // ========================================================

    if(lendoDireita)
    {
        if(!tca.selecionarCanal(
            CANAL_DIREITA
        ))
        {
            return houveLeitura;
        }


        if(
            sensorDireita.checkReadingProgress()
        )
        {
            sensorDireita.getAllChannels(
                readings
            );

            armazenarLeitura(
                dadosDireita,
                readings
            );

            lendoDireita = false;

            houveLeitura = true;
        }
    }


    // ========================================================
    // ESQUERDA
    // ========================================================

    if(!lendoEsquerda)
    {
        if(!tca.selecionarCanal(
            CANAL_ESQUERDA
        ))
        {
            return houveLeitura;
        }

        sensorEsquerda.startReading();

        lendoEsquerda = true;
    }


    if(lendoEsquerda)
    {
        if(!tca.selecionarCanal(
            CANAL_ESQUERDA
        ))
        {
            return houveLeitura;
        }


        if(
            sensorEsquerda.checkReadingProgress()
        )
        {
            sensorEsquerda.getAllChannels(
                readings
            );

            armazenarLeitura(
                dadosEsquerda,
                readings
            );

            lendoEsquerda = false;

            houveLeitura = true;
        }
    }


    // ========================================================
    // Se terminou os dois, começa novamente o direito
    // ========================================================

    if(
        !lendoDireita &&
        !lendoEsquerda
    )
    {
        if(
            tca.selecionarCanal(
                CANAL_DIREITA
            )
        )
        {
            sensorDireita.startReading();

            lendoDireita = true;
        }
    }


    return houveLeitura;
}


AS7341Data AS7341Sensores::getDireita()
{
    return dadosDireita;
}


AS7341Data AS7341Sensores::getEsquerda()
{
    return dadosEsquerda;
}