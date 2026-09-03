#include <Arduino.h>

#include "Hardware/TCA.h"
#include "Hardware/TOF200F.h"
#include "sensores/TOFAnalise.h"

// ============================================================
// CONFIGURAÇÃO
// ============================================================

#define TCA_ENDERECO 0x70
#define TOF_CANAL 0

#define I2C_SDA 21
#define I2C_SCL 22

// ============================================================
// OBJETOS
// ============================================================

TCA tca(TCA_ENDERECO);

TOF200F tof200f(tca);

TOFAnalise tofAnalise(tof200f);

// ============================================================
// CONTROLE DE PRINT
// ============================================================

unsigned long ultimoPrint = 0;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("           TESTE TOF200F + TOFANALISE");
    Serial.println("==============================================");
    Serial.println();

    // --------------------------------------------------------
    // Inicializa I2C / TCA
    // --------------------------------------------------------

    tca.begin(
        I2C_SDA,
        I2C_SCL,
        400000
    );

    Serial.println("TCA inicializado.");

    // --------------------------------------------------------
    // Inicializa TOF no canal 0
    // --------------------------------------------------------

    if(!tof200f.begin(TOF_CANAL))
    {
        Serial.println("ERRO: TOF200F NAO INICIALIZADO!");

        while(true)
        {
            delay(1000);
        }
    }

    Serial.println("TOF200F OK.");
    Serial.println();

    Serial.println("INSTRUCOES:");
    Serial.println();
    Serial.println("1. Deixe a frente LIVRE.");
    Serial.println("2. Coloque um obstaculo proximo.");
    Serial.println("3. Retire o obstaculo.");
    Serial.println();
    Serial.println("Observe principalmente:");
    Serial.println(" - LEITURA");
    Serial.println(" - DISTANCIA FILTRADA");
    Serial.println(" - VALIDO");
    Serial.println(" - BAIXAS");
    Serial.println(" - OBSTACULO");
    Serial.println();

    Serial.println("----------------------------------------------");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Atualiza o hardware do TOF
    // --------------------------------------------------------

    tof200f.update();

    // --------------------------------------------------------
    // Atualiza a análise
    // --------------------------------------------------------

    tofAnalise.update();

    // --------------------------------------------------------
    // Mostra somente quando existe nova medição
    // --------------------------------------------------------

    if(
        tof200f.temNovaLeitura()
    )
    {
        Serial.print("LEITURA: ");

        if(tof200f.ultimaLeituraValidaAgora())
        {
            Serial.print(tof200f.getUltimaLeitura());
            Serial.print(" mm");
        }
        else
        {
            Serial.print("INVALIDA");
        }

        Serial.print(" | DIST FILTRADA: ");
        Serial.print(tofAnalise.getDistancia());
        Serial.print(" mm");

        Serial.print(" | VALIDO: ");
        Serial.print(
            tofAnalise.isValido()
            ? "SIM"
            : "NAO"
        );

        Serial.print(" | BAIXAS: ");
        Serial.print(
            tofAnalise.getLeiturasBaixas()
        );

        Serial.print(" | OBSTACULO: ");

        if(tofAnalise.temObstaculo())
        {
            Serial.println("SIM");
        }
        else
        {
            Serial.println("NAO");
        }
    }
}