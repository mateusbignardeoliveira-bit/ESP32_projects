#include <Arduino.h>

#include "hardware/ICM20948.h"
#include "sensores/IMU.h"

// ============================================================
// ICM20948
// ============================================================

ICM20948 icm;
IMU imu(icm);

// ============================================================
// UART MOTOR
//
// RX = GPIO 18
// TX = GPIO 5
// ============================================================

HardwareSerial motores(2);

// ============================================================
// CONFIGURAÇÃO
// ============================================================

const int VELOCIDADE = 500;

// ============================================================
// MOTOR
// ============================================================

void setSpeed(int m1, int m2, int m3, int m4)
{
    String comando = "$spd:";

    comando += String(m1);
    comando += ",";
    comando += String(m2);
    comando += ",";
    comando += String(m3);
    comando += ",";
    comando += String(m4);
    comando += "#";

    motores.print(comando);
}

// ============================================================
// PARA
// ============================================================

void parar()
{
    setSpeed(0, 0, 0, 0);
}

// ============================================================
// NORMALIZA 0..360
// ============================================================

float normalizar(float angulo)
{
    while (angulo >= 360.0f)
        angulo -= 360.0f;

    while (angulo < 0.0f)
        angulo += 360.0f;

    return angulo;
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println(" TESTE SIMPLES - CURVA 90 GRAUS");
    Serial.println("================================");
    Serial.println();

    // --------------------------------------------------------
    // ICM
    // --------------------------------------------------------

    if (!icm.begin(19, 23, 400000))
    {
        Serial.println("ERRO ICM20948!");

        while (true)
            delay(1000);
    }

    Serial.println("ICM OK.");

    // --------------------------------------------------------
    // IMU
    // --------------------------------------------------------

    if (!imu.begin())
    {
        Serial.println("ERRO IMU!");

        while (true)
            delay(1000);
    }

    Serial.println("IMU OK.");

    // --------------------------------------------------------
    // UART motores
    // --------------------------------------------------------

    motores.begin(
        115200,
        SERIAL_8N1,
        16,
        17
    );

    parar();

    // --------------------------------------------------------
    // CALIBRAÇÃO
    // --------------------------------------------------------

    Serial.println();
    Serial.println("NAO MOVA O ROBO!");
    Serial.println("Calibrando gyro...");

    delay(2000);

    if (!imu.calibrar())
    {
        Serial.println("ERRO CALIBRACAO!");

        parar();

        while (true)
            delay(1000);
    }

    Serial.println("Calibracao OK.");

    // --------------------------------------------------------
    // Estabilização
    // --------------------------------------------------------

    for (int i = 0; i < 50; i++)
    {
        imu.update();
        delay(20);
    }

    // --------------------------------------------------------
    // Heading inicial
    // --------------------------------------------------------

    float inicial = imu.getHeading();
    float alvo = normalizar(inicial + 90.0f);

    Serial.println();

    Serial.print("Inicial: ");
    Serial.println(inicial, 2);

    Serial.print("Alvo:   ");
    Serial.println(alvo, 2);

    Serial.println();

    Serial.println("Comecando em 3...");

    delay(1000);

    Serial.println("2...");
    delay(1000);

    Serial.println("1...");
    delay(1000);

    Serial.println("GIRANDO!");

    // --------------------------------------------------------
    // Começa a girar DIREITA
    // --------------------------------------------------------

    setSpeed(
        VELOCIDADE,
        VELOCIDADE,
        -VELOCIDADE,
        -VELOCIDADE
    );
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    static bool terminou = false;

    if (terminou)
    {
        parar();
        delay(100);
        return;
    }

    // --------------------------------------------------------
    // Atualiza IMU
    // --------------------------------------------------------

    imu.update();

    float atual = imu.getHeading();

    // --------------------------------------------------------
    // Calcula quanto girou desde o início
    // --------------------------------------------------------

    static float inicial = 0.0f;
    static bool primeiraLeitura = true;

    if (primeiraLeitura)
    {
        inicial = atual;
        primeiraLeitura = false;
    }

    float girado = atual - inicial;

    if (girado < 0.0f)
        girado += 360.0f;

    // --------------------------------------------------------
    // DEBUG
    // --------------------------------------------------------

    Serial.print("Heading: ");
    Serial.print(atual, 2);

    Serial.print(" | Girado: ");
    Serial.println(girado, 2);

    // --------------------------------------------------------
    // PARADA
    //
    // Aqui NÃO existe correção para trás.
    // Apenas gira até passar de 88 graus.
    // --------------------------------------------------------

    if (girado >= 88.0f)
    {
        parar();

        delay(200);

        // Atualiza novamente para verificar posição final
        imu.update();

        Serial.println();
        Serial.println("==============================");
        Serial.println("       CURVA PARADA");
        Serial.println("==============================");

        Serial.print("Heading final: ");
        Serial.println(imu.getHeading(), 2);

        Serial.print("Total girado:  ");
        Serial.println(girado, 2);

        terminou = true;

        return;
    }

    delay(20);
}
