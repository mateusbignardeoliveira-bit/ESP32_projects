#include <Arduino.h>

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
// PINOS
// ============================================================

// I2C principal
static constexpr int PINO_SDA = 19;
static constexpr int PINO_SCL = 23;

// Interruptor geral
static constexpr int PINO_INTERRUPTOR = 32;

// ARGB
static constexpr int PINO_ARGB = 25;


// ============================================================
// UART
// ============================================================

// Array de linha
// RX = 16
// TX = 17
static constexpr int ARRAY_RX = 16;
static constexpr int ARRAY_TX = 17;

// Yahboom
// RX = 18
// TX = 5
static constexpr int MOTOR_RX = 18;
static constexpr int MOTOR_TX = 5;


// ============================================================
// TCA
// ============================================================

static constexpr uint8_t CANAL_TOF = 0;


// ============================================================
// VELOCIDADES DOS CONTROLES
// ============================================================

static constexpr int VELOCIDADE_LINHA = 200;


// ============================================================
// OBJETOS DE HARDWARE
// ============================================================

TCA tca;

AS7341Sensores sensoresCor(tca);

ArrayLinha arrayLinha(
    Serial2,
    ARRAY_RX,
    ARRAY_TX,
    115200
);

ICM20948 icm;

MotorControlador motores(
    Serial1,
    MOTOR_RX,
    MOTOR_TX,
    115200
);

TOF200F tof(
    tca
);

ARGB leds;


// ============================================================
// OBJETOS DE ANÁLISE
// ============================================================

LinhaAnalise analiseLinha;

AS7341Analise analiseCorEsquerda;
AS7341Analise analiseCorDireita;

IMU imu(icm);

TOFAnalise analiseTOF(tof);


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
// DADOS ATUAIS
// ============================================================

ArrayData dadosArray;

LinhaData dadosLinha;

AS7341Data dadosCorEsquerda;

AS7341Data dadosCorDireita;

AS7341Resultado resultadoCorEsquerda;

AS7341Resultado resultadoCorDireita;


// ============================================================
// CONTROLE DO INTERRUPTOR
// ============================================================

bool roboDesligado = false;


// ============================================================
// DESLIGA ROBÔ
// ============================================================

void desligarRobo()
{
    if (roboDesligado)
    {
        return;
    }

    roboDesligado = true;

    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.cancelar();

    motores.stop();

    motores.release();

    leds.apagar();
}


// ============================================================
// ATUALIZA INDICAÇÃO DOS LEDs
// ============================================================

void atualizarLEDs()
{
    // --------------------------------------------------------
    // Prioridade 1:
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
    // Prioridade 2:
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
    // Prioridade 3:
    // OBSTÁCULO
    // --------------------------------------------------------

    if (analiseTOF.temObstaculo())
    {
        leds.mostrarObstaculo();

        return;
    }


    // --------------------------------------------------------
    // Prioridade 4:
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
    // Prioridade 5:
    // LINHA
    //
    // Cada LED representa um sensor do array.
    // Preto = azul.
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
    // --------------------------------------------------------
    // Interruptor geral
    // LOW = desligado
    // --------------------------------------------------------

    pinMode(
        PINO_INTERRUPTOR,
        INPUT_PULLUP
    );


    // --------------------------------------------------------
    // Inicializa ARGB
    // --------------------------------------------------------

    leds.begin();


    // --------------------------------------------------------
    // Inicializa I2C
    //
    // TCA e ICM compartilham o mesmo barramento físico.
    // ICM não passa pelo TCA.
    // --------------------------------------------------------

    tca.begin(
        PINO_SDA,
        PINO_SCL,
        400000
    );


    // --------------------------------------------------------
    // ICM
    // --------------------------------------------------------

    icm.begin(
        PINO_SDA,
        PINO_SCL,
        400000
    );

    imu.begin();


    // --------------------------------------------------------
    // Motores
    // --------------------------------------------------------

    motores.begin();


    // --------------------------------------------------------
    // Array de linha
    // --------------------------------------------------------

    arrayLinha.begin();


    // --------------------------------------------------------
    // AS7341
    // --------------------------------------------------------

    sensoresCor.begin();


    // --------------------------------------------------------
    // ToF
    // --------------------------------------------------------

    tof.begin(
        CANAL_TOF
    );


    // --------------------------------------------------------
    // Controles
    // --------------------------------------------------------

    controleLinha.begin();

    controleLinha.setVelocidadeBase(
        VELOCIDADE_LINHA
    );

    controleGiro.begin();

    controleObstaculo.begin();


    // --------------------------------------------------------
    // Máquina de estados
    // --------------------------------------------------------

    maquinaEstados.begin();


    // --------------------------------------------------------
    // Calibração inicial do giroscópio
    //
    // Não imprime nada na serial.
    // O robô deve estar parado neste momento.
    // --------------------------------------------------------

    imu.calibrar();

    imu.zerarHeading();


    // --------------------------------------------------------
    // Estado inicial dos LEDs
    // --------------------------------------------------------

    leds.apagar();


    // --------------------------------------------------------
    // Se o interruptor já estiver desligado,
    // o robô permanece desligado.
    // --------------------------------------------------------

    if (digitalRead(PINO_INTERRUPTOR) == LOW)
    {
        desligarRobo();
    }
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Interruptor geral
    //
    // LOW = robô parado.
    // --------------------------------------------------------

    if (digitalRead(PINO_INTERRUPTOR) == LOW)
    {
        desligarRobo();

        return;
    }


    // --------------------------------------------------------
    // Interruptor voltou para HIGH.
    //
    // Neste momento permitimos novamente a execução.
    // --------------------------------------------------------

    if (roboDesligado)
    {
        roboDesligado = false;

        controles:
        controleLinha.reset();

        maquinaEstados.begin();
    }


    // --------------------------------------------------------
    // Atualiza array de linha
    // --------------------------------------------------------

    arrayLinha.update();

    dadosArray = arrayLinha.getData();

    analiseLinha.update(
        dadosArray
    );

    dadosLinha = analiseLinha.getData();


    // --------------------------------------------------------
    // Atualiza AS7341
    // --------------------------------------------------------

    sensoresCor.update();

    dadosCorDireita =
        sensoresCor.getDireita();

    dadosCorEsquerda =
        sensoresCor.getEsquerda();


    // --------------------------------------------------------
    // Analisa cores
    // --------------------------------------------------------

    resultadoCorDireita =
        analiseCorDireita.analisar(
            dadosCorDireita
        );

    resultadoCorEsquerda =
        analiseCorEsquerda.analisar(
            dadosCorEsquerda
        );


    // --------------------------------------------------------
    // Atualiza ToF
    // --------------------------------------------------------

    tof.update();

    analiseTOF.update();


    // --------------------------------------------------------
    // Atualiza máquina de estados
    // --------------------------------------------------------

    maquinaEstados.update(
        dadosLinha,
        dadosCorEsquerda,
        dadosCorDireita
    );


    // --------------------------------------------------------
    // Atualiza indicação visual
    // --------------------------------------------------------

    atualizarLEDs();
}