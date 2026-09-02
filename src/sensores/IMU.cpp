#include "IMU.h"

#include <Arduino.h>
#include <math.h>

// =============================================================
// CONSTRUTOR
// =============================================================

IMU::IMU(ICM20948& hardware)
    : sensor(hardware),
      dados{},

      gyroBiasX(0.0f),
      gyroBiasY(0.0f),
      gyroBiasZ(0.0f),

      magOffsetX(0.0f),
      magOffsetY(0.0f),
      magOffsetZ(0.0f),

      magMinX(0.0f),
      magMinY(0.0f),
      magMinZ(0.0f),

      magMaxX(0.0f),
      magMaxY(0.0f),
      magMaxZ(0.0f),

      calibrandoMagnetometro(false),

      yawIntegrado(0.0f),
      headingOffset(0.0f),

      ultimoTempo(0),

      accelFiltradoX(0.0f),
      accelFiltradoY(0.0f),
      accelFiltradoZ(0.0f),

      gyroFiltradoX(0.0f),
      gyroFiltradoY(0.0f),
      gyroFiltradoZ(0.0f),

      magFiltradoX(0.0f),
      magFiltradoY(0.0f),
      magFiltradoZ(0.0f),

      primeiraAtualizacao(true)
{
}

// =============================================================
// BEGIN
// =============================================================

bool IMU::begin()
{
    dados = {};

    gyroBiasX = 0.0f;
    gyroBiasY = 0.0f;
    gyroBiasZ = 0.0f;

    magOffsetX = 0.0f;
    magOffsetY = 0.0f;
    magOffsetZ = 0.0f;

    yawIntegrado = 0.0f;
    headingOffset = 0.0f;

    accelFiltradoX = 0.0f;
    accelFiltradoY = 0.0f;
    accelFiltradoZ = 0.0f;

    gyroFiltradoX = 0.0f;
    gyroFiltradoY = 0.0f;
    gyroFiltradoZ = 0.0f;

    magFiltradoX = 0.0f;
    magFiltradoY = 0.0f;
    magFiltradoZ = 0.0f;

    calibrandoMagnetometro = false;

    primeiraAtualizacao = true;

    ultimoTempo = micros();

    dados.calibrada = false;

    return sensor.isConnected();
}

// =============================================================
// UPDATE
// =============================================================

bool IMU::update()
{
    if (!sensor.update())
        return false;

    const ICM20948Data& bruto =
        sensor.getData();

    // ---------------------------------------------------------
    // Primeira leitura
    // ---------------------------------------------------------

    if (primeiraAtualizacao)
    {
        accelFiltradoX = bruto.accelX;
        accelFiltradoY = bruto.accelY;
        accelFiltradoZ = bruto.accelZ;

        gyroFiltradoX =
            bruto.gyroX - gyroBiasX;

        gyroFiltradoY =
            bruto.gyroY - gyroBiasY;

        gyroFiltradoZ =
            bruto.gyroZ - gyroBiasZ;

        magFiltradoX =
            bruto.magX - magOffsetX;

        magFiltradoY =
            bruto.magY - magOffsetY;

        magFiltradoZ =
            bruto.magZ - magOffsetZ;

        calcularRollPitch();

        // Começamos o yaw em zero.
        yawIntegrado = 0.0f;

        headingOffset = 0.0f;

        ultimoTempo = micros();

        primeiraAtualizacao = false;

        dados.accelX = accelFiltradoX;
        dados.accelY = accelFiltradoY;
        dados.accelZ = accelFiltradoZ;

        dados.gyroX = gyroFiltradoX;
        dados.gyroY = gyroFiltradoY;
        dados.gyroZ = gyroFiltradoZ;

        dados.magX = magFiltradoX;
        dados.magY = magFiltradoY;
        dados.magZ = magFiltradoZ;

        dados.temperatura =
            bruto.temperature;

        dados.yaw = 0.0f;
        dados.heading = 0.0f;

        return true;
    }

    // ---------------------------------------------------------
    // DT
    // ---------------------------------------------------------

    uint32_t agora = micros();

    float dt =
        (agora - ultimoTempo) /
        1000000.0f;

    ultimoTempo = agora;

    if (
        dt <= 0.0f ||
        dt > 0.1f
    )
    {
        dt = 0.01f;
    }

    // ---------------------------------------------------------
    // ACELERÔMETRO
    // ---------------------------------------------------------

    accelFiltradoX =
        filtrar(
            accelFiltradoX,
            bruto.accelX,
            ALPHA_ACELEROMETRO
        );

    accelFiltradoY =
        filtrar(
            accelFiltradoY,
            bruto.accelY,
            ALPHA_ACELEROMETRO
        );

    accelFiltradoZ =
        filtrar(
            accelFiltradoZ,
            bruto.accelZ,
            ALPHA_ACELEROMETRO
        );

    // ---------------------------------------------------------
    // GIROSCÓPIO
    // ---------------------------------------------------------

    float gyroX =
        bruto.gyroX - gyroBiasX;

    float gyroY =
        bruto.gyroY - gyroBiasY;

    float gyroZ =
        bruto.gyroZ - gyroBiasZ;

    gyroFiltradoX =
        filtrar(
            gyroFiltradoX,
            gyroX,
            ALPHA_GIROSCOPIO
        );

    gyroFiltradoY =
        filtrar(
            gyroFiltradoY,
            gyroY,
            ALPHA_GIROSCOPIO
        );

    gyroFiltradoZ =
        filtrar(
            gyroFiltradoZ,
            gyroZ,
            ALPHA_GIROSCOPIO
        );

    // ---------------------------------------------------------
    // MAGNETÔMETRO
    // ---------------------------------------------------------

    if (bruto.magDataReady)
    {
        float magX =
            bruto.magX - magOffsetX;

        float magY =
            bruto.magY - magOffsetY;

        float magZ =
            bruto.magZ - magOffsetZ;

        magFiltradoX =
            filtrar(
                magFiltradoX,
                magX,
                ALPHA_MAGNETOMETRO
            );

        magFiltradoY =
            filtrar(
                magFiltradoY,
                magY,
                ALPHA_MAGNETOMETRO
            );

        magFiltradoZ =
            filtrar(
                magFiltradoZ,
                magZ,
                ALPHA_MAGNETOMETRO
            );
    }

    // ---------------------------------------------------------
    // Calibração magnética
    // ---------------------------------------------------------

    if (calibrandoMagnetometro)
    {
        atualizarCalibracaoMagnetometro();
    }

    // ---------------------------------------------------------
    // ORIENTAÇÃO
    // ---------------------------------------------------------

    calcularRollPitch();

    calcularYaw(dt);

    // ---------------------------------------------------------
    // SAÍDA
    // ---------------------------------------------------------

    dados.accelX = accelFiltradoX;
    dados.accelY = accelFiltradoY;
    dados.accelZ = accelFiltradoZ;

    dados.gyroX = gyroFiltradoX;
    dados.gyroY = gyroFiltradoY;
    dados.gyroZ = gyroFiltradoZ;

    dados.magX = magFiltradoX;
    dados.magY = magFiltradoY;
    dados.magZ = magFiltradoZ;

    dados.temperatura =
        bruto.temperature;

    dados.yaw = yawIntegrado;

    dados.heading =
        normalizarAngulo(
            yawIntegrado +
            headingOffset
        );

    dados.calibrada = true;

    return true;
}

// =============================================================
// CALIBRAR GIRO
// =============================================================

bool IMU::calibrar()
{
    constexpr int AMOSTRAS = 500;

    float somaX = 0.0f;
    float somaY = 0.0f;
    float somaZ = 0.0f;

    int validas = 0;

    Serial.println();
    Serial.println("NAO MOVA O ROBO!");
    Serial.println("Calibrando gyro...");

    delay(300);

    for (int i = 0; i < AMOSTRAS; i++)
    {
        if (sensor.update())
        {
            const ICM20948Data& bruto =
                sensor.getData();

            somaX += bruto.gyroX;
            somaY += bruto.gyroY;
            somaZ += bruto.gyroZ;

            validas++;
        }

        delay(5);
    }

    if (validas < 100)
    {
        Serial.println("ERRO: poucas amostras do gyro.");
        return false;
    }

    gyroBiasX =
        somaX / validas;

    gyroBiasY =
        somaY / validas;

    gyroBiasZ =
        somaZ / validas;

    gyroFiltradoX = 0.0f;
    gyroFiltradoY = 0.0f;
    gyroFiltradoZ = 0.0f;

    yawIntegrado = 0.0f;
    headingOffset = 0.0f;

    ultimoTempo = micros();

    dados.calibrada = true;

    Serial.println("Calibracao OK.");

    Serial.print("Bias X: ");
    Serial.println(gyroBiasX, 4);

    Serial.print("Bias Y: ");
    Serial.println(gyroBiasY, 4);

    Serial.print("Bias Z: ");
    Serial.println(gyroBiasZ, 4);

    return true;
}

// =============================================================
// CALIBRAÇÃO MAGNÉTICA
// =============================================================

void IMU::iniciarCalibracaoMagnetometro()
{
    calibrandoMagnetometro = true;

    magMinX = 1000000.0f;
    magMinY = 1000000.0f;
    magMinZ = 1000000.0f;

    magMaxX = -1000000.0f;
    magMaxY = -1000000.0f;
    magMaxZ = -1000000.0f;
}

bool IMU::atualizarCalibracaoMagnetometro()
{
    if (!calibrandoMagnetometro)
        return false;

    if (magFiltradoX < magMinX)
        magMinX = magFiltradoX;

    if (magFiltradoY < magMinY)
        magMinY = magFiltradoY;

    if (magFiltradoZ < magMinZ)
        magMinZ = magFiltradoZ;

    if (magFiltradoX > magMaxX)
        magMaxX = magFiltradoX;

    if (magFiltradoY > magMaxY)
        magMaxY = magFiltradoY;

    if (magFiltradoZ > magMaxZ)
        magMaxZ = magFiltradoZ;

    return true;
}

bool IMU::finalizarCalibracaoMagnetometro()
{
    if (!calibrandoMagnetometro)
        return false;

    calibrandoMagnetometro = false;

    float amplitudeX =
        magMaxX - magMinX;

    float amplitudeY =
        magMaxY - magMinY;

    float amplitudeZ =
        magMaxZ - magMinZ;

    if (
        amplitudeX < 1.0f &&
        amplitudeY < 1.0f &&
        amplitudeZ < 1.0f
    )
    {
        return false;
    }

    magOffsetX =
        (magMaxX + magMinX) *
        0.5f;

    magOffsetY =
        (magMaxY + magMinY) *
        0.5f;

    magOffsetZ =
        (magMaxZ + magMinZ) *
        0.5f;

    magFiltradoX = 0.0f;
    magFiltradoY = 0.0f;
    magFiltradoZ = 0.0f;

    return true;
}

// =============================================================
// ZERAR HEADING
// =============================================================

void IMU::zerarHeading()
{
    yawIntegrado = 0.0f;

    headingOffset = 0.0f;

    dados.yaw = 0.0f;
    dados.heading = 0.0f;

    ultimoTempo = micros();
}

// =============================================================
// GET DATA
// =============================================================

const IMUData& IMU::getData() const
{
    return dados;
}

// =============================================================
// GET YAW
// =============================================================

float IMU::getYaw() const
{
    return dados.yaw;
}

// =============================================================
// GET HEADING
// =============================================================

float IMU::getHeading() const
{
    return dados.heading;
}

// =============================================================
// GET ROLL
// =============================================================

float IMU::getRoll() const
{
    return dados.roll;
}

// =============================================================
// GET PITCH
// =============================================================

float IMU::getPitch() const
{
    return dados.pitch;
}

// =============================================================
// DIFERENÇA ANGULAR
// =============================================================

float IMU::diferencaAngular(
    float atual,
    float alvo
)
{
    float diferenca =
        alvo - atual;

    while (diferenca > 180.0f)
        diferenca -= 360.0f;

    while (diferenca < -180.0f)
        diferenca += 360.0f;

    return diferenca;
}

// =============================================================
// NORMALIZAR
// =============================================================

float IMU::normalizarAngulo(
    float angulo
) const
{
    while (angulo >= 360.0f)
        angulo -= 360.0f;

    while (angulo < 0.0f)
        angulo += 360.0f;

    return angulo;
}

// =============================================================
// FILTRO
// =============================================================

float IMU::filtrar(
    float anterior,
    float atual,
    float alpha
)
{
    return anterior +
           alpha *
           (atual - anterior);
}

// =============================================================
// ROLL / PITCH
// =============================================================

void IMU::calcularRollPitch()
{
    dados.roll =
        atan2f(
            accelFiltradoY,
            accelFiltradoZ
        ) *
        GRAUS_POR_RADIANO;

    dados.pitch =
        atan2f(
            -accelFiltradoX,
            sqrtf(
                accelFiltradoY *
                accelFiltradoY +

                accelFiltradoZ *
                accelFiltradoZ
            )
        ) *
        GRAUS_POR_RADIANO;
}

// =============================================================
// YAW
// =============================================================

void IMU::calcularYaw(float dt)
{
    /*
     * Para navegação do robô usamos principalmente o
     * giroscópio Z.
     *
     * Não aplicamos correção magnética contínua aqui.
     *
     * Isso é intencional:
     *
     * - curvas precisam responder rapidamente;
     * - o magnetômetro pode sofrer interferência;
     * - o controle de 90°/180° será baseado no gyro;
     * - a calibração do gyro remove o drift inicial.
     */

    yawIntegrado +=
        gyroFiltradoZ *
        SINAL_YAW *
        dt;

    // Mantém yaw em uma faixa confortável.
    while (yawIntegrado >= 360.0f)
        yawIntegrado -= 360.0f;

    while (yawIntegrado < 0.0f)
        yawIntegrado += 360.0f;
}

// =============================================================
// HEADING MAGNÉTICO
// =============================================================

float IMU::calcularHeadingMagnetometro() const
{
    float heading =
        atan2f(
            magFiltradoY,
            magFiltradoX
        ) *
        GRAUS_POR_RADIANO;

    return normalizarAngulo(heading);
}

// =============================================================
// HEADING MAGNÉTICO COMPENSADO
// =============================================================

float IMU::calcularHeadingMagnetometroCompensado() const
{
    float roll =
        dados.roll /
        GRAUS_POR_RADIANO;

    float pitch =
        dados.pitch /
        GRAUS_POR_RADIANO;

    float sr = sinf(roll);
    float cr = cosf(roll);

    float sp = sinf(pitch);
    float cp = cosf(pitch);

    float Xh =
        magFiltradoX * cp +
        magFiltradoZ * sp;

    float Yh =
        magFiltradoX * sr * sp +
        magFiltradoY * cr -
        magFiltradoZ * sr * cp;

    float heading =
        atan2f(
            Yh,
            Xh
        ) *
        GRAUS_POR_RADIANO;

    return normalizarAngulo(heading);
}