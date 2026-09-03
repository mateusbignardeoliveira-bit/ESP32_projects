#include <Arduino.h>
#include <Wire.h>

#include "Hardware/TCA.h"
#include "Hardware/AS7341.h"
#include "Hardware/ArrayLinha.h"
#include "Hardware/ICM20948.h"
#include "Hardware/MotorControlador.h"
#include "Hardware/TOF200F.h"
#include "Hardware/ARGB.h"

#include "sensores/LinhaAnalise.h"
#include "sensores/AS7341Analise.h"
#include "sensores/IMU.h"
#include "sensores/TOFAnalise.h"

#include "controle/PIDLinha.h"
#include "controle/ControleLinha.h"
#include "controle/ControleGiro.h"
#include "controle/ControleObstaculo.h"
#include "controle/Verde.h"

#include "estados/MaquinaEstados.h"


// ============================================================
// CONFIGURAÇÃO DE PINOS
// ============================================================

// ------------------------------------------------------------
// I2C do TCA9548A
//
// Usa o Wire principal do ESP32.
// O TCA fica neste barramento.
//
// Pinos padrão do Wire:
// SDA = GPIO21
// SCL = GPIO22
// ------------------------------------------------------------


// ------------------------------------------------------------
// I2C EXCLUSIVO DO ICM-20948
//
// O ICM NÃO passa pelo TCA.
//
// SDA = GPIO19
// SCL = GPIO23
// ------------------------------------------------------------

static constexpr int ICM_SDA = 19;
static constexpr int ICM_SCL = 23;


// ------------------------------------------------------------
// Botão / interruptor de segurança
//
// LOW = parar robô
//
// Quando acionado, o robô permanece parado até reiniciar.
// ------------------------------------------------------------

static constexpr int PINO_INTERRUPTOR = 32;


// ------------------------------------------------------------
// ARGB
// ------------------------------------------------------------

static constexpr int PINO_ARGB = 25;


// ============================================================
// UART
// ============================================================

// ------------------------------------------------------------
// Array de linha
// ------------------------------------------------------------

static constexpr int ARRAY_RX = 5;
static constexpr int ARRAY_TX = 18;


// ------------------------------------------------------------
// Controlador Yahboom
// ------------------------------------------------------------
//
// RX ESP32 = GPIO18
// TX ESP32 = GPIO5
// ------------------------------------------------------------

static constexpr int MOTOR_RX = 16;
static constexpr int MOTOR_TX = 17;


// ============================================================
// TCA9548A
// ============================================================

// ------------------------------------------------------------
// Canal 0 = ToF
// Canal 1 = AS7341 direita
// Canal 2 = AS7341 esquerda
// ------------------------------------------------------------

static constexpr uint8_t CANAL_TOF = 0;


// ============================================================
// PARÂMETROS GERAIS
// ============================================================

static constexpr int VELOCIDADE_LINHA = 200;


// ============================================================
// OBJETOS DE HARDWARE
// ============================================================

// ------------------------------------------------------------
// TCA
// ------------------------------------------------------------

TCA tca;


// ------------------------------------------------------------
// Sensores AS7341
// ------------------------------------------------------------

AS7341Sensores sensoresCor(
    tca
);


// ------------------------------------------------------------
// Array de linha
// ------------------------------------------------------------

ArrayLinha arrayLinha(
    Serial2,
    ARRAY_RX,
    ARRAY_TX,
    115200
);


// ------------------------------------------------------------
// Barramento I2C exclusivo do ICM
// ------------------------------------------------------------
//
// IMPORTANTE:
//
// O TCA usa o Wire principal.
//
// O ICM usa outro controlador I2C,
// permitindo trabalhar nos GPIO19/23
// sem alterar o barramento do TCA.
// ------------------------------------------------------------

TwoWire I2C_ICM(
    1
);


// ------------------------------------------------------------
// ICM-20948
// ------------------------------------------------------------

ICM20948 icm(
    I2C_ICM
);


// ------------------------------------------------------------
// Controlador de motores
// ------------------------------------------------------------

MotorControlador motores(
    Serial1,
    MOTOR_RX,
    MOTOR_TX,
    115200
);


// ------------------------------------------------------------
// ToF
// ------------------------------------------------------------

TOF200F tof(
    tca
);


// ------------------------------------------------------------
// ARGB
// ------------------------------------------------------------

ARGB leds;


// ============================================================
// OBJETOS DE ANÁLISE
// ============================================================

LinhaAnalise analiseLinha;

AS7341Analise analiseCorEsquerda;

AS7341Analise analiseCorDireita;

IMU imu(
    icm
);

TOFAnalise analiseTOF(
    tof
);


// ============================================================
// OBJETOS DE CONTROLE
// ============================================================

PIDLinha pidLinha;


ControleLinha controleLinha(
    pidLinha,
    motores
);


ControleGiro controleGiro(
    imu,
    motores
);


ControleObstaculo controleObstaculo(
    imu,
    motores
);


Verde verde(
    analiseCorEsquerda
);


// ============================================================
// MÁQUINA DE ESTADOS
// ============================================================

MaquinaEstados maquinaEstados(
    analiseLinha,
    analiseCorEsquerda,
    analiseCorDireita,
    verde,
    analiseTOF,
    imu,
    controleLinha,
    controleGiro,
    controleObstaculo,
    motores
);


// ============================================================
// DADOS DOS SENSORES
// ============================================================

ArrayData dadosArray;

LinhaData dadosLinha;

AS7341Data dadosCorEsquerda;

AS7341Data dadosCorDireita;


// ------------------------------------------------------------
// Resultados das análises espectrais
// ------------------------------------------------------------

AS7341Resultado resultadoCorEsquerda;

AS7341Resultado resultadoCorDireita;


// ============================================================
// ESTADO GERAL DO ROBÔ
// ============================================================

bool roboParadoPorInterruptor = false;


// ============================================================
// FUNÇÃO DE PARADA DE EMERGÊNCIA
// ============================================================

void pararRobo()
{
    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.cancelar();

    motores.stop();

    delay(100);

    motores.release();

    leds.apagar();
}


// ============================================================
// FALHA DE INICIALIZAÇÃO
// ============================================================
//
// Em caso de falha de um hardware essencial:
//
// - motores são parados
// - motores são liberados
// - LEDs ficam vermelhos
// - execução permanece parada
// ============================================================

void falhaInicializacao(
    const char* mensagem
)
{
    Serial.print(
        "ERRO: "
    );

    Serial.println(
        mensagem
    );

    motores.stop();

    delay(100);

    motores.release();

    leds.mostrarVermelho();

    while (true)
    {
        delay(1000);
    }
}


// ============================================================
// ATUALIZAÇÃO DOS LEDs
// ============================================================

void atualizarLEDs()
{
    // --------------------------------------------------------
    // PRIORIDADE 1
    // VERMELHO
    // --------------------------------------------------------

    if (
        resultadoCorEsquerda.vermelhoDetectado ||
        resultadoCorDireita.vermelhoDetectado
    )
    {
        leds.mostrarVermelho();

        return;
    }


    // --------------------------------------------------------
    // PRIORIDADE 2
    // CINZA
    // --------------------------------------------------------

    if (
        resultadoCorEsquerda.cinzaDetectado ||
        resultadoCorDireita.cinzaDetectado
    )
    {
        leds.mostrarCinza();

        return;
    }


    // --------------------------------------------------------
    // PRIORIDADE 3
    // OBSTÁCULO
    // --------------------------------------------------------

    if (
        analiseTOF.temObstaculo()
    )
    {
        leds.mostrarObstaculo();

        return;
    }


    // --------------------------------------------------------
    // PRIORIDADE 4
    // VERDE
    // --------------------------------------------------------

    if (
        resultadoCorEsquerda.verdeDetectado ||
        resultadoCorDireita.verdeDetectado
    )
    {
        leds.mostrarVerde(
            resultadoCorEsquerda.verdeDetectado,
            resultadoCorDireita.verdeDetectado
        );

        return;
    }


    // --------------------------------------------------------
    // PRIORIDADE 5
    // LINHA
    //
    // Cada LED representa um sensor.
    //
    // Preto = azul.
    // Branco = apagado.
    // --------------------------------------------------------

    leds.mostrarLinha(
        dadosLinha.sensores
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    // ========================================================
    // SERIAL DE DEBUG
    // ========================================================

    Serial.begin(
        115200
    );

    delay(100);


    // ========================================================
    // INTERRUPTOR
    // ========================================================

    pinMode(
        PINO_INTERRUPTOR,
        INPUT_PULLUP
    );


    // ========================================================
    // ARGB
    // ========================================================

    leds.begin();


    // ========================================================
    // INICIALIZA TCA
    // ========================================================
    //
    // O TCA usa o Wire principal.
    //
    // Como nenhum pino foi informado, a classe usa os pinos
    // padrão do ESP32:
    //
    // SDA = GPIO21
    // SCL = GPIO22
    // ========================================================

    tca.begin(
        -1,
        -1,
        400000
    );


    // ========================================================
    // INICIALIZA ICM
    // ========================================================
    //
    // O ICM usa o segundo barramento I2C:
    //
    // SDA = 19
    // SCL = 23
    // ========================================================

    if (
        !icm.begin(
            ICM_SDA,
            ICM_SCL,
            400000
        )
    )
    {
        falhaInicializacao(
            "ICM-20948"
        );
    }


    // ========================================================
    // INICIA CAMADA DE IMU
    // ========================================================

    if (
        !imu.begin()
    )
    {
        falhaInicializacao(
            "IMU"
        );
    }


    // ========================================================
    // CALIBRA GIROSCÓPIO
    // ========================================================
    //
    // O robô deve permanecer completamente parado.
    // ========================================================

    Serial.println(
        "Calibrando giroscopio..."
    );

    if (
        !imu.calibrar()
    )
    {
        falhaInicializacao(
            "Calibracao do giroscopio"
        );
    }

    imu.zerarHeading();


    // ========================================================
    // MOTORES
    // ========================================================

    motores.begin();


    // ========================================================
    // ARRAY DE LINHA
    // ========================================================

    arrayLinha.begin();


    // ========================================================
    // AS7341
    // ========================================================

    if (
        !sensoresCor.begin()
    )
    {
        falhaInicializacao(
            "AS7341"
        );
    }


    // ========================================================
    // TOF
    // ========================================================

    if (
        !tof.begin(
            CANAL_TOF
        )
    )
    {
        falhaInicializacao(
            "VL53L0X / ToF"
        );
    }


    // ========================================================
    // CONTROLE DE LINHA
    // ========================================================

    controleLinha.begin();

    controleLinha.setVelocidadeBase(
        VELOCIDADE_LINHA
    );


    // ========================================================
    // CONTROLE DE GIRO
    // ========================================================

    controleGiro.begin();


    // ========================================================
    // CONTROLE DE OBSTÁCULO
    // ========================================================

    controleObstaculo.begin();


    // ========================================================
    // MÁQUINA DE ESTADOS
    // ========================================================

    maquinaEstados.begin();


    // ========================================================
    // VERIFICA INTERRUPTOR
    // ========================================================

    if (
        digitalRead(
            PINO_INTERRUPTOR
        ) == LOW
    )
    {
        roboParadoPorInterruptor = true;

        pararRobo();

        Serial.println(
            "Robo parado pelo interruptor."
        );

        return;
    }


    // ========================================================
    // LEDS INICIAIS
    // ========================================================

    leds.apagar();


    // ========================================================
    // FINALIZA SETUP
    // ========================================================

    Serial.println(
        "Robo inicializado."
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
     // ========================================================
    // INTERRUPTOR
    // ========================================================

    if(digitalRead(PINO_INTERRUPTOR) == LOW)
    {
        if(!roboParadoPorInterruptor)
        {
            roboParadoPorInterruptor = true;

            pararRobo();

            Serial.println(
                "Robo parado pelo interruptor."
            );
        }

        return;
    }


    // ========================================================
    // INTERRUPTOR VOLTOU PARA HIGH
    // ========================================================

    if(roboParadoPorInterruptor)
    {
        Serial.println(
            "Novo ciclo iniciado."
        );

        // Limpa toda a lógica anterior.
        //
        // Não recalibra a IMU.
        // Não zera heading.
        maquinaEstados.resetExecucao();

        leds.apagar();

        roboParadoPorInterruptor = false;
    }


    // ========================================================
    // ATUALIZA ARRAY
    // ========================================================

    arrayLinha.update();

    dadosArray =
        arrayLinha.getData();


    analiseLinha.update(
        dadosArray
    );

    dadosLinha =
        analiseLinha.getData();


    // ========================================================
    // ATUALIZA AS7341
    // ========================================================

    sensoresCor.update();


    dadosCorDireita =
        sensoresCor.getDireita();


    dadosCorEsquerda =
        sensoresCor.getEsquerda();


    // ========================================================
    // ANALISA CORES
    // ========================================================

    resultadoCorDireita =
        analiseCorDireita.analisar(
            dadosCorDireita
        );


    resultadoCorEsquerda =
        analiseCorEsquerda.analisar(
            dadosCorEsquerda
        );


    // ========================================================
    // ATUALIZA TOF
    // ========================================================

    tof.update();

    analiseTOF.update();


    // ========================================================
    // MÁQUINA DE ESTADOS
    // ========================================================
    //
    // A máquina decide o que fazer.
    //
    // O Main NÃO decide curva, verde, obstáculo etc.
    // ========================================================

    maquinaEstados.update(
        dadosLinha,
        dadosCorEsquerda,
        dadosCorDireita
    );


    // ========================================================
    // ATUALIZA INDICAÇÃO VISUAL
    // ========================================================

    atualizarLEDs();
}