#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "hardware/ICM20948.h"

struct IMUData
{
    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float magX;
    float magY;
    float magZ;

    float roll;
    float pitch;

    // Ângulo integrado pelo giroscópio.
    float yaw;

    // Heading relativo do robô [0, 360).
    float heading;

    float temperatura;

    bool calibrada;
};

class IMU
{
public:

    explicit IMU(ICM20948& hardware);

    bool begin();

    bool update();

    // Calibração do giroscópio.
    // Robô deve permanecer completamente parado.
    bool calibrar();

    // Calibração do magnetômetro.
    void iniciarCalibracaoMagnetometro();

    bool atualizarCalibracaoMagnetometro();

    bool finalizarCalibracaoMagnetometro();

    // Define a direção atual como 0°.
    void zerarHeading();

    const IMUData& getData() const;

    float getYaw() const;

    float getHeading() const;

    float getRoll() const;

    float getPitch() const;

    // Retorna o menor erro angular:
    //
    // positivo = alvo está à direita
    // negativo = alvo está à esquerda
    //
    static float diferencaAngular(
        float atual,
        float alvo
    );

private:

    ICM20948& sensor;

    IMUData dados;

    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

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

    float yawIntegrado;

    float headingOffset;

    uint32_t ultimoTempo;

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

    static constexpr float ALPHA_ACELEROMETRO = 0.15f;

    static constexpr float ALPHA_GIROSCOPIO = 0.35f;

    static constexpr float ALPHA_MAGNETOMETRO = 0.15f;

    static constexpr float GRAUS_POR_RADIANO =
        57.2957795f;

    // ---------------------------------------------------------
    // IMPORTANTE
    //
    // Se o giro físico para a direita fizer o heading diminuir,
    // trocamos este valor para -1.
    // ---------------------------------------------------------

    static constexpr float SINAL_YAW = -1.0f;

    float filtrar(
        float anterior,
        float atual,
        float alpha
    );

    float normalizarAngulo(
        float angulo
    ) const;

    void calcularRollPitch();

    void calcularYaw(
        float dt
    );

    float calcularHeadingMagnetometro() const;

    float calcularHeadingMagnetometroCompensado() const;
};

#endif