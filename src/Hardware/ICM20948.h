#ifndef ICM20948_H
#define ICM20948_H

#include <Arduino.h>
#include <Wire.h>

struct ICM20948Data
{
    int16_t accelRawX;
    int16_t accelRawY;
    int16_t accelRawZ;

    float accelX;
    float accelY;
    float accelZ;

    int16_t gyroRawX;
    int16_t gyroRawY;
    int16_t gyroRawZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    int16_t magRawX;
    int16_t magRawY;
    int16_t magRawZ;

    float magX;
    float magY;
    float magZ;

    int16_t tempRaw;
    float temperature;

    bool magDataReady;
    bool magOverflow;
};

class ICM20948
{
public:

    ICM20948(
        TwoWire& wire = Wire,
        uint8_t endereco = 0x68
    );

    bool begin(
        int sda = 19,
        int scl = 23,
        uint32_t frequencia = 400000
    );

    bool update();

    const ICM20948Data& getData() const;

    bool isConnected() const;

private:

    TwoWire& _wire;

    uint8_t _enderecoICM;
    uint8_t _enderecoMag;

    ICM20948Data _dados;

    bool _conectado;

    bool escrever(
        uint8_t endereco,
        uint8_t reg,
        uint8_t valor
    );

    uint8_t ler(
        uint8_t endereco,
        uint8_t reg
    );

    bool lerBytes(
        uint8_t endereco,
        uint8_t reg,
        uint8_t* dados,
        uint8_t quantidade
    );

    int16_t montarInt16(
        uint8_t alto,
        uint8_t baixo
    );

    bool inicializarICM();

    bool inicializarMagnetometro();

    bool lerAcelerometro();

    bool lerGiroscopio();

    bool lerTemperatura();

    bool lerMagnetometro();
};

#endif