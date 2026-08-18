#include <Arduino.h>

#include "hardware/TCA.h"
#include "hardware/AS7341.h"
#include "hardware/ArrayLinha.h"

#include "sensores/AS7341Analise.h"
#include "sensores/LinhaAnalise.h"

#include "decisao/DecisaoRobo.h"
#include "estados/EstadoRobo.h"

// ============================================================
// OBJETOS
// ============================================================

TCA tca;

AS7341Sensores sensoresAS7341(tca);

ArrayLinha arrayLinha(
    Serial2,
    5,
    18,
    115200
);

AS7341Analise as7341Analise;

LinhaAnalise linhaAnalise;

DecisaoRobo decisaoRobo;

EstadoRoboControl estadoRobo;

// ============================================================
// NOME DA DECISÃO DO VERDE
// ============================================================

const char* nomeAcaoVerde(
    TipoAcaoVerde acao
)
{
    switch(acao)
    {
        case VERDE_SEM_EVENTO:
            return "SEM EVENTO";

        case VERDE_PASSAGEM_RETA:
            return "PASSAGEM RETA";

        case VERDE_CURVA_ESQUERDA:
            return "CURVA ESQUERDA";

        case VERDE_CURVA_DIREITA:
            return "CURVA DIREITA";

        case VERDE_MEIA_VOLTA:
            return "MEIA VOLTA";

        default:
            return "DESCONHECIDA";
    }
}

// ============================================================
// NOME DO EVENTO DO ARRAY
// ============================================================

const char* nomeEventoArray(
    TipoEventoArray evento
)
{
    switch(evento)
    {
        case ARRAY_NORMAL:
            return "NORMAL";

        case ARRAY_CURVA_ESQUERDA:
            return "CURVA ESQUERDA";

        case ARRAY_CURVA_DIREITA:
            return "CURVA DIREITA";

        case ARRAY_INTERSECCAO:
            return "INTERSECCAO";

        default:
            return "DESCONHECIDO";
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" TESTE DECISAO + ESTADOS");
    Serial.println(" AS7341 + ARRAY");
    Serial.println("========================================");

    // --------------------------------------------------------
    // TCA
    // --------------------------------------------------------

    Serial.println("Inicializando TCA...");

    tca.begin();

    Serial.println("TCA OK");

    // --------------------------------------------------------
    // AS7341
    // --------------------------------------------------------

    Serial.println("Inicializando AS7341...");

    if(!sensoresAS7341.begin())
    {
        Serial.println("ERRO AS7341!");

        while(true)
        {
            delay(1000);
        }
    }

    Serial.println("AS7341 OK");

    // --------------------------------------------------------
    // ARRAY
    // --------------------------------------------------------

    Serial.println("Inicializando ArrayLinha...");

    arrayLinha.begin();

    Serial.println("Array OK");

    Serial.println();
    Serial.println("Sistema pronto.");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Atualiza array
    // --------------------------------------------------------

    arrayLinha.update();

    // --------------------------------------------------------
    // Atualiza AS7341
    // --------------------------------------------------------

    sensoresAS7341.update();

    // --------------------------------------------------------
    // Obtém dados brutos do AS7341
    // --------------------------------------------------------

    AS7341Data dadosEsquerda =
        sensoresAS7341.getEsquerda();

    AS7341Data dadosDireita =
        sensoresAS7341.getDireita();

    // --------------------------------------------------------
    // Obtém dados brutos do array
    // --------------------------------------------------------

    ArrayData dadosLinha =
        arrayLinha.getData();

    // --------------------------------------------------------
    // Analisa AS7341
    // --------------------------------------------------------

    AS7341Resultado esquerda =
        as7341Analise.analisar(
            dadosEsquerda
        );

    AS7341Resultado direita =
        as7341Analise.analisar(
            dadosDireita
        );

    // --------------------------------------------------------
    // Analisa linha
    // --------------------------------------------------------

    linhaAnalise.update(
        dadosLinha
    );

    LinhaData linha =
        linhaAnalise.getData();

    // --------------------------------------------------------
    // DECISÃO
    // --------------------------------------------------------

    decisaoRobo.update(
        esquerda,
        direita,
        linha
    );

    const DecisaoData& decisao =
        decisaoRobo.getData();

    // --------------------------------------------------------
    // ESTADO
    // --------------------------------------------------------

    estadoRobo.update(
        decisao
    );

    // ========================================================
    // SERIAL
    // ========================================================

    Serial.println();
    Serial.println("========================================");
    Serial.println(" DECISAO ATUAL");
    Serial.println("========================================");

    // --------------------------------------------------------
    // AS7341
    // --------------------------------------------------------

    Serial.print("Verde esquerda: ");

    if(decisao.verdeEsquerda)
    {
        Serial.println("SIM");
    }
    else
    {
        Serial.println("NAO");
    }

    Serial.print("Verde direita:  ");

    if(decisao.verdeDireita)
    {
        Serial.println("SIM");
    }
    else
    {
        Serial.println("NAO");
    }

    // --------------------------------------------------------
    // ARRAY
    // --------------------------------------------------------

    Serial.print("Sensores pretos: ");

    Serial.println(
        decisao.sensoresPretos
    );

    Serial.print("Posicao digital: ");

    Serial.println(
        decisao.posicaoDigital
    );

    Serial.print("Evento array: ");

    Serial.println(
        nomeEventoArray(
            decisao.eventoArray
        )
    );

    // --------------------------------------------------------
    // DECISÃO DO VERDE
    // --------------------------------------------------------

    Serial.print("Acao verde: ");

    Serial.println(
        nomeAcaoVerde(
            decisao.acaoVerde
        )
    );

    // --------------------------------------------------------
    // EVENTO ESPECIAL
    // --------------------------------------------------------

    Serial.print("Evento especial: ");

    if(decisao.eventoEspecial)
    {
        Serial.println("SIM");
    }
    else
    {
        Serial.println("NAO");
    }

    // --------------------------------------------------------
    // ESTADO DO ROBÔ
    // --------------------------------------------------------

    Serial.print("Estado robo: ");

    Serial.println(
        estadoRobo.getNomeEstado()
    );

    Serial.println("========================================");

    delay(200);
}