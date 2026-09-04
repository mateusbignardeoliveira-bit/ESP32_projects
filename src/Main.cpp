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
// PINOS
// ============================================================

static constexpr int ICM_SDA = 19;
static constexpr int ICM_SCL = 23;

static constexpr int PINO_INTERRUPTOR = 32;

static constexpr int PINO_ARGB = 25;


// ============================================================
// UART
// ============================================================

static constexpr int ARRAY_RX = 5;
static constexpr int ARRAY_TX = 18;

static constexpr int MOTOR_RX = 16;
static constexpr int MOTOR_TX = 17;


// ============================================================
// TCA
// ============================================================

static constexpr uint8_t CANAL_TOF = 0;


// ============================================================
// VELOCIDADE
// ============================================================

static constexpr int VELOCIDADE_LINHA = 200;


// ============================================================
// LADO DO OBSTÁCULO
// ============================================================
//
// -1 = esquerda
// +1 = direita
//
// Direita é a configuração inicial.
//

static constexpr int LADO_OBSTACULO = 1;


// ============================================================
// CONFIRMAÇÃO TEMPORAL DE CORES
// ============================================================
//
// Uma única leitura não é suficiente para parar o robô.
//
// A cor precisa aparecer durante várias leituras
// consecutivas.
//
// Isso evita que um ruído momentâneo do AS7341
// seja interpretado como uma marca real.
//

static constexpr int LEITURAS_VERMELHO_NECESSARIAS = 10;

static constexpr int LEITURAS_CINZA_NECESSARIAS = 10;


// ============================================================
// OBJETOS
// ============================================================

TCA tca;


AS7341Sensores sensoresCor(
    tca
);


ArrayLinha arrayLinha(
    Serial2,
    ARRAY_RX,
    ARRAY_TX,
    115200
);


TwoWire I2C_ICM(
    1
);


ICM20948 icm(
    I2C_ICM
);


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
// ANÁLISE
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
// CONTROLE
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
// MÁQUINA
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
// DADOS
// ============================================================

ArrayData dadosArray;

LinhaData dadosLinha;

AS7341Data dadosCorEsquerda;

AS7341Data dadosCorDireita;


AS7341Resultado resultadoCorEsquerda;

AS7341Resultado resultadoCorDireita;


// ============================================================
// INTERRUPTOR
// ============================================================

bool roboParadoPorInterruptor = false;


// ============================================================
// CONTADORES DE CONFIRMAÇÃO
// ============================================================

int contadorVermelho = 0;

int contadorCinza = 0;


// ============================================================
// PARADA
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
// RESET DA CONFIRMAÇÃO DE CORES
// ============================================================

void resetConfirmacaoCores()
{
    contadorVermelho = 0;

    contadorCinza = 0;
}


// ============================================================
// ATUALIZAR CONFIRMAÇÃO DE CORES
// ============================================================
//
// A lógica é:
//
// leitura vermelha
//     -> contador vermelho++
//
// leitura não vermelha
//     -> contador vermelho = 0
//
// leitura cinza
//     -> contador cinza++
//
// leitura não cinza
//     -> contador cinza = 0
//
// Assim, somente uma sequência realmente contínua
// consegue confirmar a marca.
//

void atualizarConfirmacaoCores()
{
    // ========================================================
    // VERMELHO
    // ========================================================

    if(
        resultadoCorEsquerda.vermelhoDetectado ||
        resultadoCorDireita.vermelhoDetectado
    )
    {
        contadorVermelho++;

        if(
            contadorVermelho >
            LEITURAS_VERMELHO_NECESSARIAS
        )
        {
            contadorVermelho =
                LEITURAS_VERMELHO_NECESSARIAS;
        }
    }
    else
    {
        contadorVermelho = 0;
    }


    // ========================================================
    // CINZA
    // ========================================================

    if(
        resultadoCorEsquerda.cinzaDetectado ||
        resultadoCorDireita.cinzaDetectado
    )
    {
        contadorCinza++;

        if(
            contadorCinza >
            LEITURAS_CINZA_NECESSARIAS
        )
        {
            contadorCinza =
                LEITURAS_CINZA_NECESSARIAS;
        }
    }
    else
    {
        contadorCinza = 0;
    }
}


// ============================================================
// FALHA
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


    while(true)
    {
        delay(1000);
    }
}


// ============================================================
// LEDS
// ============================================================

void atualizarLEDs()
{
    // --------------------------------------------------------
    // VERMELHO TRAVADO
    // --------------------------------------------------------

    if(
        maquinaEstados.paradoPorVermelho()
    )
    {
        leds.mostrarVermelho();

        return;
    }


    // --------------------------------------------------------
    // CINZA TRAVADO
    // --------------------------------------------------------

    if(
        maquinaEstados.paradoPorCinza()
    )
    {
        leds.mostrarCinza();

        return;
    }


    // --------------------------------------------------------
    // VERMELHO
    // --------------------------------------------------------

    if(
        resultadoCorEsquerda.vermelhoDetectado ||
        resultadoCorDireita.vermelhoDetectado
    )
    {
        leds.mostrarVermelho();

        return;
    }


    // --------------------------------------------------------
    // CINZA
    // --------------------------------------------------------

    if(
        resultadoCorEsquerda.cinzaDetectado ||
        resultadoCorDireita.cinzaDetectado
    )
    {
        leds.mostrarCinza();

        return;
    }


    // --------------------------------------------------------
    // OBSTÁCULO
    // --------------------------------------------------------

    if(
        analiseTOF.temObstaculo()
    )
    {
        leds.mostrarObstaculo();

        return;
    }


    // --------------------------------------------------------
    // VERDE
    // --------------------------------------------------------

    if(
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
    // LINHA
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
    // TCA
    // ========================================================

    tca.begin(
        -1,
        -1,
        400000
    );


    // ========================================================
    // ICM
    // ========================================================

    if(
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
    // IMU
    // ========================================================

    if(
        !imu.begin()
    )
    {
        falhaInicializacao(
            "IMU"
        );
    }


    // ========================================================
    // CALIBRAÇÃO
    // ========================================================

    Serial.println(
        "Calibrando giroscopio..."
    );


    if(
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
    // ARRAY
    // ========================================================

    arrayLinha.begin();


    // ========================================================
    // AS7341
    // ========================================================

    if(
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

    if(
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
    // MÁQUINA
    // ========================================================

    maquinaEstados.begin();


    // ========================================================
    // INTERRUPTOR INICIAL
    // ========================================================

    if(
        digitalRead(
            PINO_INTERRUPTOR
        ) == LOW
    )
    {
        roboParadoPorInterruptor = true;

        resetConfirmacaoCores();

        pararRobo();

        Serial.println(
            "Robo parado pelo interruptor."
        );

        return;
    }


    // ========================================================
    // ESTADO INICIAL DOS CONTADORES
    // ========================================================

    resetConfirmacaoCores();


    leds.apagar();


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
    // LEITURA DOS AS7341
    // ========================================================
    //
    // Fazemos a leitura antes do interruptor porque
    // vermelho possui prioridade absoluta.
    //

    sensoresCor.update();


    dadosCorDireita =
        sensoresCor.getDireita();


    dadosCorEsquerda =
        sensoresCor.getEsquerda();


    resultadoCorDireita =
        analiseCorDireita.analisar(
            dadosCorDireita
        );


    resultadoCorEsquerda =
        analiseCorEsquerda.analisar(
            dadosCorEsquerda
        );


    // ========================================================
    // ATUALIZA CONFIRMAÇÃO
    // ========================================================

    atualizarConfirmacaoCores();


    // ========================================================
    // VERMELHO CONFIRMADO
    // ========================================================
    //
    // Depois de 5 leituras consecutivas:
    //
    // -> STOP_VERMELHO
    //
    // Esse estado é absoluto.
    // O interruptor não consegue liberá-lo.
    //

    if(
        contadorVermelho >=
        LEITURAS_VERMELHO_NECESSARIAS
    )
    {
        maquinaEstados.pararPorVermelho();

        atualizarLEDs();

        return;
    }


    // ========================================================
    // VERMELHO JÁ TRAVADO
    // ========================================================
    //
    // Continua travado para sempre.
    //

    if(
        maquinaEstados.paradoPorVermelho()
    )
    {
        motores.stop();

        leds.mostrarVermelho();

        return;
    }


    // ========================================================
    // INTERRUPTOR LOW
    // ========================================================
    //
    // LOW = robô pausado.
    //
    // O contador de cores é limpo.
    //

    if(
        digitalRead(
            PINO_INTERRUPTOR
        ) == LOW
    )
    {
        if(
            !roboParadoPorInterruptor
        )
        {
            roboParadoPorInterruptor =
                true;

            resetConfirmacaoCores();

            pararRobo();

            Serial.println(
                "Robo pausado pelo interruptor."
            );
        }

        return;
    }


    // ========================================================
    // INTERRUPTOR VOLTOU PARA HIGH
    // ========================================================
    //
    // Isso significa:
    //
    // NOVO CICLO DE EXECUÇÃO
    //
    // Não recalibra a IMU.
    //
    // Não chama imu.calibrar().
    //
    // Não chama imu.zerarHeading().
    //
    // Apenas reinicia a máquina de estados.
    //

    if(
        roboParadoPorInterruptor
    )
    {
        Serial.println(
            "Novo ciclo iniciado."
        );


        maquinaEstados.resetExecucao();


        resetConfirmacaoCores();


        leds.apagar();


        roboParadoPorInterruptor =
            false;
    }


    // ========================================================
    // CINZA CONFIRMADO
    // ========================================================
    //
    // Cinza precisa aparecer em 5 leituras consecutivas.
    //
    // Diferentemente do vermelho, cinza NÃO é uma trava
    // absoluta.
    //
    // O interruptor acima consegue reiniciar a execução.
    //

    if(
        contadorCinza >=
        LEITURAS_CINZA_NECESSARIAS
    )
    {
        maquinaEstados.pararPorCinza();

        atualizarLEDs();

        return;
    }


    // ========================================================
    // ARRAY
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
    // TOF
    // ========================================================

    tof.update();

    analiseTOF.update();


    // ========================================================
    // MÁQUINA
    // ========================================================

    maquinaEstados.update(
        dadosLinha,
        dadosCorEsquerda,
        dadosCorDireita
    );


    // ========================================================
    // LED
    // ========================================================

    atualizarLEDs();
}