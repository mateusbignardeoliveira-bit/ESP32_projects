#include <Arduino.h>

#include "Hardware/ICM20948.h"
#include "Hardware/MotorControlador.h"

#include "sensores/IMU.h"

#include "controle/ControleGiro.h"


// ============================================================
// OBJETOS
// ============================================================

ICM20948 icm;

HardwareSerial SerialMotores(1);

MotorControlador motores(
    SerialMotores,
    16,     // RX
    17      // TX
);

IMU imu(icm);

ControleGiro controleGiro(
    imu,
    motores
);


// ============================================================
// CONTROLE DO TESTE
// ============================================================

bool testeFinalizado = false;

unsigned long inicioTeste = 0;
unsigned long ultimoPrint = 0;


// ============================================================
// SETUP
// ============================================================

void setup()
{

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println(" TESTE - CURVA 90 GRAUS");
    Serial.println("================================");
    Serial.println();


    // --------------------------------------------------------
    // ICM
    // --------------------------------------------------------

    Serial.println("Inicializando ICM...");

    if(!icm.begin(19, 23))
    {
        Serial.println("ERRO: ICM nao encontrado.");

        while(true)
        {
            delay(1000);
        }
    }

    Serial.println("ICM OK.");


    // --------------------------------------------------------
    // IMU
    // --------------------------------------------------------

    if(!imu.begin())
    {
        Serial.println("ERRO: IMU nao inicializada.");

        while(true)
        {
            delay(1000);
        }
    }

    Serial.println("IMU OK.");


    // --------------------------------------------------------
    // MOTORES
    // --------------------------------------------------------

    Serial.println("Inicializando controlador dos motores...");

    motores.begin();

    Serial.println("Motores OK.");


    // --------------------------------------------------------
    // CONTROLE DE GIRO
    // --------------------------------------------------------

    controleGiro.begin();

    Serial.println("Controle de giro OK.");


    // --------------------------------------------------------
    // CALIBRACAO DO GYRO
    // --------------------------------------------------------

    Serial.println();
    Serial.println("NAO MOVA O ROBO!");
    Serial.println("Calibrando gyro...");

    imu.calibrar();

    Serial.println("Calibracao OK.");


    // --------------------------------------------------------
    // ZERA HEADING
    // --------------------------------------------------------

    imu.zerarHeading();

    delay(500);

    Serial.print("Heading inicial: ");
    Serial.println(imu.getHeading());


    // --------------------------------------------------------
    // TEMPO PARA COLOCAR O ROBO NO CHAO
    // --------------------------------------------------------

    Serial.println();
    Serial.println("Prepare o robo...");

    for(int i = 3; i > 0; i--)
    {

        Serial.print(i);
        Serial.println("...");

        delay(1000);

    }


    Serial.println("CURVA 90 DIREITA!");

    Serial.println();


    // --------------------------------------------------------
    // INICIA CURVA
    // --------------------------------------------------------

    controleGiro.curva90Direita();

    inicioTeste = millis();

}



// ============================================================
// LOOP
// ============================================================

void loop()
{

    // --------------------------------------------------------
    // Atualiza controle de giro
    // --------------------------------------------------------

    if(!testeFinalizado)
    {

        controleGiro.update();


        // ----------------------------------------------------
        // DEBUG
        // ----------------------------------------------------

        if(millis() - ultimoPrint >= 100)
        {

            ultimoPrint = millis();

            Serial.print("Heading: ");
            Serial.print(imu.getHeading(), 2);

            Serial.print(" | Yaw: ");
            Serial.print(imu.getData().yaw, 2);

            Serial.print(" | GyroZ: ");
            Serial.print(imu.getData().gyroZ, 2);

            Serial.print(" | M1: ");
            Serial.print(motores.getM1());

            Serial.print(" M2: ");
            Serial.print(motores.getM2());

            Serial.print(" M3: ");
            Serial.print(motores.getM3());

            Serial.print(" M4: ");
            Serial.println(motores.getM4());

        }


        // ----------------------------------------------------
        // CURVA TERMINOU
        // ----------------------------------------------------

        if(controleGiro.terminou())
        {

            testeFinalizado = true;


            Serial.println();
            Serial.println("================================");
            Serial.println(" CURVA CONCLUIDA");
            Serial.println("================================");


            Serial.print("Heading final: ");
            Serial.println(imu.getHeading(), 2);


            // ------------------------------------------------
            // PRIMEIRO: ZERA VELOCIDADE PELO PID
            // ------------------------------------------------

            Serial.println();
            Serial.println("Enviando STOP...");
            
            motores.stop();


            // ------------------------------------------------
            // PEQUENO TEMPO PARA O DRIVER PROCESSAR
            // ------------------------------------------------

            delay(200);


            // ------------------------------------------------
            // DEPOIS: LIBERA O PID
            // ------------------------------------------------

            Serial.println("Enviando RELEASE...");

            motores.release();


            Serial.println("Motores liberados.");
            Serial.println();


            Serial.println("Agora nao deve haver torque tentando");
            Serial.println("girar as rodas.");

        }


        // ----------------------------------------------------
        // TIMEOUT DE SEGURANCA
        // ----------------------------------------------------

        if(millis() - inicioTeste > 10000)
        {

            Serial.println();
            Serial.println("================================");
            Serial.println(" TIMEOUT");
            Serial.println("================================");

            Serial.println("Cancelando controle...");

            controleGiro.cancelar();


            Serial.println("STOP...");

            motores.stop();

            delay(200);

            Serial.println("RELEASE...");

            motores.release();


            testeFinalizado = true;

        }

    }


    // --------------------------------------------------------
    // TESTE FINALIZADO
    // --------------------------------------------------------

    else
    {

        // Nada mais é enviado ao controlador.
        // Os motores permanecem liberados.

        delay(10);

    }

}