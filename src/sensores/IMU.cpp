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

    // ---------------------------------------------------------
    // ZERA CALIBRAÇÃO
    // ---------------------------------------------------------

    gyroBiasX = 0.0f;
    gyroBiasY = 0.0f;
    gyroBiasZ = 0.0f;

    // Os offsets magnéticos serão obtidos pela calibração.
    magOffsetX = 0.0f;
    magOffsetY = 0.0f;
    magOffsetZ = 0.0f;

    // ---------------------------------------------------------
    // ORIENTAÇÃO
    // ---------------------------------------------------------

    yawIntegrado = 0.0f;

    headingOffset = 0.0f;

    // ---------------------------------------------------------
    // FILTROS
    // ---------------------------------------------------------

    accelFiltradoX = 0.0f;
    accelFiltradoY = 0.0f;
    accelFiltradoZ = 0.0f;

    gyroFiltradoX = 0.0f;
    gyroFiltradoY = 0.0f;
    gyroFiltradoZ = 0.0f;

    magFiltradoX = 0.0f;
    magFiltradoY = 0.0f;
    magFiltradoZ = 0.0f;

    // ---------------------------------------------------------
    // ESTADO
    // ---------------------------------------------------------

    primeiraAtualizacao = true;

    calibrandoMagnetometro = false;

    ultimoTempo = micros();

    dados.calibrada = false;

    return sensor.isConnected();
}

// =============================================================
// UPDATE
// =============================================================

bool IMU::update()
{
    // ---------------------------------------------------------
    // LEITURA DO HARDWARE
    // ---------------------------------------------------------

    if (!sensor.update()) {
        return false;
    }

    const ICM20948Data& bruto = sensor.getData();

    // =========================================================
    // PRIMEIRA LEITURA
    // =========================================================

    if (primeiraAtualizacao)
    {
        // -----------------------------------------------------
        // ACELERÔMETRO
        // -----------------------------------------------------

        accelFiltradoX = bruto.accelX;
        accelFiltradoY = bruto.accelY;
        accelFiltradoZ = bruto.accelZ;

        // -----------------------------------------------------
        // GIROSCÓPIO
        // -----------------------------------------------------

        gyroFiltradoX = bruto.gyroX - gyroBiasX;
        gyroFiltradoY = bruto.gyroY - gyroBiasY;
        gyroFiltradoZ = bruto.gyroZ - gyroBiasZ;

        // -----------------------------------------------------
        // MAGNETÔMETRO
        // -----------------------------------------------------

        magFiltradoX = bruto.magX - magOffsetX;
        magFiltradoY = bruto.magY - magOffsetY;
        magFiltradoZ = bruto.magZ - magOffsetZ;

        // -----------------------------------------------------
        // ORIENTAÇÃO INICIAL
        // -----------------------------------------------------

        calcularRollPitch();

        float headingMag =
            calcularHeadingMagnetometroCompensado();

        yawIntegrado = headingMag;

        headingOffset = 0.0f;

        // -----------------------------------------------------
        // TEMPO
        // -----------------------------------------------------

        ultimoTempo = micros();

        primeiraAtualizacao = false;

        // -----------------------------------------------------
        // SAÍDA
        // -----------------------------------------------------

        dados.accelX = accelFiltradoX;
        dados.accelY = accelFiltradoY;
        dados.accelZ = accelFiltradoZ;

        dados.gyroX = gyroFiltradoX;
        dados.gyroY = gyroFiltradoY;
        dados.gyroZ = gyroFiltradoZ;

        dados.magX = magFiltradoX;
        dados.magY = magFiltradoY;
        dados.magZ = magFiltradoZ;

        dados.temperatura = bruto.temperature;

        dados.yaw = yawIntegrado;

        dados.heading =
            normalizarAngulo(
                yawIntegrado + headingOffset
            );

        return true;
    }

    // =========================================================
    // TEMPO
    // =========================================================

    uint32_t agora = micros();

    float dt =
        (agora - ultimoTempo) / 1000000.0f;

    ultimoTempo = agora;

    // Proteção contra interrupções/bloqueios.
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }

    // =========================================================
    // ACELERÔMETRO
    // =========================================================

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

    // =========================================================
    // GIROSCÓPIO
    // =========================================================

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

    // =========================================================
    // MAGNETÔMETRO
    // =========================================================

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

    // =========================================================
    // CALIBRAÇÃO MAGNÉTICA
    // =========================================================

    if (calibrandoMagnetometro) {
        atualizarCalibracaoMagnetometro();
    }

    // =========================================================
    // ORIENTAÇÃO
    // =========================================================

    calcularRollPitch();

    calcularYaw(dt);

    // =========================================================
    // SAÍDA
    // =========================================================

    dados.accelX = accelFiltradoX;
    dados.accelY = accelFiltradoY;
    dados.accelZ = accelFiltradoZ;

    dados.gyroX = gyroFiltradoX;
    dados.gyroY = gyroFiltradoY;
    dados.gyroZ = gyroFiltradoZ;

    dados.magX = magFiltradoX;
    dados.magY = magFiltradoY;
    dados.magZ = magFiltradoZ;

    dados.temperatura = bruto.temperature;

    dados.yaw = yawIntegrado;

    dados.heading =
        normalizarAngulo(
            yawIntegrado + headingOffset
        );

    dados.calibrada =
        (fabsf(gyroBiasX) > 0.0f ||
         fabsf(gyroBiasY) > 0.0f ||
         fabsf(gyroBiasZ) > 0.0f);

    return true;
}

// =============================================================
// CALIBRAÇÃO DO GIROSCÓPIO
// =============================================================

bool IMU::calibrar()
{
    constexpr int AMOSTRAS = 500;

    float somaX = 0.0f;
    float somaY = 0.0f;
    float somaZ = 0.0f;

    int amostrasValidas = 0;

    /*
     * O robô precisa permanecer completamente parado.
     */

    for (int i = 0; i < AMOSTRAS; i++)
    {
        if (!sensor.update()) {
            delay(5);
            continue;
        }

        const ICM20948Data& bruto =
            sensor.getData();

        somaX += bruto.gyroX;
        somaY += bruto.gyroY;
        somaZ += bruto.gyroZ;

        amostrasValidas++;

        delay(5);
    }

    if (amostrasValidas < 100) {
        return false;
    }

    // ---------------------------------------------------------
    // BIAS
    // ---------------------------------------------------------

    gyroBiasX =
        somaX / amostrasValidas;

    gyroBiasY =
        somaY / amostrasValidas;

    gyroBiasZ =
        somaZ / amostrasValidas;

    // ---------------------------------------------------------
    // REINICIA FILTRO
    // ---------------------------------------------------------

    gyroFiltradoX = 0.0f;
    gyroFiltradoY = 0.0f;
    gyroFiltradoZ = 0.0f;

    // ---------------------------------------------------------
    // REINICIA TEMPO
    // ---------------------------------------------------------

    ultimoTempo = micros();

    dados.calibrada = true;

    return true;
}

// =============================================================
// INICIAR CALIBRAÇÃO DO MAGNETÔMETRO
// =============================================================

void IMU::iniciarCalibracaoMagnetometro()
{
    calibrandoMagnetometro = true;

    // Começa com extremos impossíveis.
    magMinX =  1000000.0f;
    magMinY =  1000000.0f;
    magMinZ =  1000000.0f;

    magMaxX = -1000000.0f;
    magMaxY = -1000000.0f;
    magMaxZ = -1000000.0f;
}

// =============================================================
// ATUALIZAR CALIBRAÇÃO DO MAGNETÔMETRO
// =============================================================

bool IMU::atualizarCalibracaoMagnetometro()
{
    if (!calibrandoMagnetometro) {
        return false;
    }

    /*
     * Utilizamos os dados já filtrados.
     *
     * Durante a calibração o robô deve ser movimentado
     * lentamente em várias direções, permitindo que o
     * magnetômetro observe diferentes orientações.
     */

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

// =============================================================
// FINALIZAR CALIBRAÇÃO DO MAGNETÔMETRO
// =============================================================

bool IMU::finalizarCalibracaoMagnetometro()
{
    if (!calibrandoMagnetometro) {
        return false;
    }

    calibrandoMagnetometro = false;

    // ---------------------------------------------------------
    // VERIFICA SE HOUVE MOVIMENTO SUFICIENTE
    // ---------------------------------------------------------

    float amplitudeX =
        magMaxX - magMinX;

    float amplitudeY =
        magMaxY - magMinY;

    float amplitudeZ =
        magMaxZ - magMinZ;

    /*
     * Se praticamente não houve variação, não temos
     * informação suficiente para calcular o offset.
     */

    if (amplitudeX < 1.0f &&
        amplitudeY < 1.0f &&
        amplitudeZ < 1.0f)
    {
        return false;
    }

    // ---------------------------------------------------------
    // HARD-IRON OFFSET
    // ---------------------------------------------------------

    magOffsetX =
        (magMaxX + magMinX) * 0.5f;

    magOffsetY =
        (magMaxY + magMinY) * 0.5f;

    magOffsetZ =
        (magMaxZ + magMinZ) * 0.5f;

    // ---------------------------------------------------------
    // REINICIA FILTRO MAGNÉTICO
    // ---------------------------------------------------------

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
    /*
     * Queremos que a orientação atual passe a ser
     * exatamente 0 graus.
     */

    float atual =
        normalizarAngulo(yawIntegrado);

    headingOffset -= atual;

    headingOffset =
        normalizarAngulo(headingOffset);

    yawIntegrado = 0.0f;

    dados.yaw = 0.0f;
    dados.heading = 0.0f;
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

    while (diferenca > 180.0f) {
        diferenca -= 360.0f;
    }

    while (diferenca < -180.0f) {
        diferenca += 360.0f;
    }

    return diferenca;
}

// =============================================================
// NORMALIZAR ÂNGULO
// =============================================================

float IMU::normalizarAngulo(float angulo) const
{
    while (angulo >= 360.0f) {
        angulo -= 360.0f;
    }

    while (angulo < 0.0f) {
        angulo += 360.0f;
    }

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
           alpha * (atual - anterior);
}

// =============================================================
// ROLL / PITCH
// =============================================================

void IMU::calcularRollPitch()
{
    /*
     * A gravidade é utilizada como referência de inclinação.
     */

    dados.roll =
        atan2f(
            accelFiltradoY,
            accelFiltradoZ
        ) * GRAUS_POR_RADIANO;

    dados.pitch =
        atan2f(
            -accelFiltradoX,
            sqrtf(
                accelFiltradoY * accelFiltradoY +
                accelFiltradoZ * accelFiltradoZ
            )
        ) * GRAUS_POR_RADIANO;
}

// =============================================================
// YAW
// =============================================================

void IMU::calcularYaw(float dt)
{
    // ---------------------------------------------------------
    // 1. INTEGRA GIROSCÓPIO
    // ---------------------------------------------------------

    float yawGyro =
        yawIntegrado +
        gyroFiltradoZ * dt;

    yawGyro =
        normalizarAngulo(yawGyro);

    // ---------------------------------------------------------
    // 2. HEADING MAGNÉTICO
    // ---------------------------------------------------------

    float headingMag =
        calcularHeadingMagnetometroCompensado();

    // ---------------------------------------------------------
    // 3. DIFERENÇA ENTRE GYRO E MAG
    // ---------------------------------------------------------

    float erroMag =
        diferencaAngular(
            yawGyro,
            headingMag
        );

    /*
     * O magnetômetro corrige lentamente o drift do gyro.
     *
     * Não simplesmente substituímos o yaw pelo magnetômetro,
     * pois isso deixaria as curvas muito ruidosas.
     */

    yawIntegrado =
        yawGyro +
        ALPHA_YAW_MAG * erroMag;

    yawIntegrado =
        normalizarAngulo(yawIntegrado);
}

// =============================================================
// HEADING MAGNÉTICO BÁSICO
// =============================================================

float IMU::calcularHeadingMagnetometro() const
{
    float heading =
        atan2f(
            magFiltradoY,
            magFiltradoX
        ) * GRAUS_POR_RADIANO;

    return normalizarAngulo(heading);
}

// =============================================================
// HEADING MAGNÉTICO COMPENSADO PELA INCLINAÇÃO
// =============================================================

float IMU::calcularHeadingMagnetometroCompensado() const
{
    /*
     * Compensação de inclinação.
     *
     * O magnetômetro não deve simplesmente usar X/Y se o
     * sensor estiver inclinado. Roll e pitch são utilizados
     * para projetar o campo magnético no plano horizontal.
     */

    float roll =
        dados.roll / GRAUS_POR_RADIANO;

    float pitch =
        dados.pitch / GRAUS_POR_RADIANO;

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
        ) * GRAUS_POR_RADIANO;

    return normalizarAngulo(
        heading
    );
}
