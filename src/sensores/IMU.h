#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "hardware/ICM20948.h"

struct IMUData
{
    // ---------------------------------------------------------
    // ACELERÔMETRO
    // ---------------------------------------------------------

    float accelX;
    float accelY;
    float accelZ;

    // ---------------------------------------------------------
    // GIROSCÓPIO
    // ---------------------------------------------------------

    float gyroX;
    float gyroY;
    float gyroZ;

    // ---------------------------------------------------------
    // MAGNETÔMETRO
    // ---------------------------------------------------------

    float magX;
    float magY;
    float magZ;

    // ---------------------------------------------------------
    // ORIENTAÇÃO
    // ---------------------------------------------------------

    float roll;
    float pitch;
    float yaw;

    // Direção absoluta/magnética normalizada [0, 360)
    float heading;

    // ---------------------------------------------------------
    // OUTROS
    // ---------------------------------------------------------

    float temperatura;

    bool calibrada;
};

class IMU
{
public:

    explicit IMU(ICM20948& hardware);

    // Inicializa o processamento da IMU
    bool begin();

    // Atualiza sensores, filtros e orientação
    bool update();

    // ---------------------------------------------------------
    // CALIBRAÇÃO
    // ---------------------------------------------------------

    // Calibra o bias do giroscópio.
    // O robô deve permanecer completamente parado.
    bool calibrar();

    // Inicia calibração do magnetômetro.
    // O robô deverá ser girado em várias direções.
    void iniciarCalibracaoMagnetometro();

    // Atualiza a calibração magnética durante a movimentação.
    bool atualizarCalibracaoMagnetometro();

    // Finaliza a calibração magnética.
    bool finalizarCalibracaoMagnetometro();

    // ---------------------------------------------------------
    // REFERÊNCIA DE ÂNGULO
    // ---------------------------------------------------------

    // Define a direção atual como 0 graus.
    void zerarHeading();

    // ---------------------------------------------------------
    // ACESSO
    // ---------------------------------------------------------

    const IMUData& getData() const;

    float getYaw() const;

    float getHeading() const;

    float getRoll() const;

    float getPitch() const;

    // Diferença angular mais curta entre atual e alvo.
    static float diferencaAngular(float atual, float alvo);

private:

    ICM20948& sensor;

    IMUData dados;

    // ---------------------------------------------------------
    // BIAS DO GIROSCÓPIO
    // ---------------------------------------------------------

    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

    // ---------------------------------------------------------
    // CALIBRAÇÃO MAGNÉTICA
    // ---------------------------------------------------------

    float magOffsetX;
    float magOffsetY;
    float magOffsetZ;

    float magMinX;
    float magMinY;
    float magMinZ;

    float magMaxX;
    float magMaxY;
    float magMaxZ;

    bool calibrandoMagnetometro;

    // ---------------------------------------------------------
    // ORIENTAÇÃO
    // ---------------------------------------------------------

    float yawIntegrado;

    // Offset utilizado para transformar o heading magnético
    // atual na referência escolhida pelo robô.
    float headingOffset;

    // ---------------------------------------------------------
    // TEMPO
    // ---------------------------------------------------------

    uint32_t ultimoTempo;

    // ---------------------------------------------------------
    // FILTROS
    // ---------------------------------------------------------

    float accelFiltradoX;
    float accelFiltradoY;
    float accelFiltradoZ;

    float gyroFiltradoX;
    float gyroFiltradoY;
    float gyroFiltradoZ;

    float magFiltradoX;
    float magFiltradoY;
    float magFiltradoZ;

    bool primeiraAtualizacao;

    // ---------------------------------------------------------
    // CONSTANTES
    // ---------------------------------------------------------

    static constexpr float ALPHA_ACELEROMETRO = 0.15f;

    static constexpr float ALPHA_GIROSCOPIO = 0.25f;

    static constexpr float ALPHA_MAGNETOMETRO = 0.15f;

    // Quanto maior, mais o yaw acompanha o magnetômetro.
    //
    // 0.02 = correção muito lenta
    // 0.05 = correção moderada
    // 0.10 = correção mais forte
    //
    // Começaremos com 0.04.
    static constexpr float ALPHA_YAW_MAG = 0.04f;

    static constexpr float GRAUS_POR_RADIANO = 57.2957795f;

    // ---------------------------------------------------------
    // PROCESSAMENTO
    // ---------------------------------------------------------

    float filtrar(
        float anterior,
        float atual,
        float alpha
    );

    float normalizarAngulo(float angulo) const;

    void calcularRollPitch();

    void calcularYaw(float dt);

    float calcularHeadingMagnetometro() const;

    float calcularHeadingMagnetometroCompensado() const;
};

#endif