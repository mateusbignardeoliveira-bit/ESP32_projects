#include "ICM20948.h"

#include <Arduino.h>

// =============================================================
// ICM-20948
// =============================================================

#define ICM_WHO_AM_I       0x00
#define ICM_USER_CTRL      0x03
#define ICM_LP_CONFIG      0x05
#define ICM_PWR_MGMT_1     0x06
#define ICM_PWR_MGMT_2     0x07
#define ICM_INT_PIN_CFG    0x0F

#define ICM_ACCEL_XOUT_H   0x2D
#define ICM_GYRO_XOUT_H    0x33
#define ICM_TEMP_OUT_H     0x39

// Registradores de configuração - banco 2
#define ICM_REG_BANK_SEL   0x7F

#define ICM_GYRO_SMPLRT_DIV    0x00
#define ICM_GYRO_CONFIG_1      0x01
#define ICM_ACCEL_SMPLRT_DIV_1 0x10
#define ICM_ACCEL_SMPLRT_DIV_2 0x11
#define ICM_ACCEL_CONFIG        0x14

// =============================================================
// AK09916
// =============================================================

#define AK_WIA2            0x01

#define AK_ST1             0x10

#define AK_HXL             0x11
#define AK_HXH             0x12
#define AK_HYL             0x13
#define AK_HYH             0x14
#define AK_HZL             0x15
#define AK_HZH             0x16

#define AK_ST2             0x18

#define AK_CNTL2           0x31
#define AK_CNTL3           0x32

// =============================================================
// Conversões
// =============================================================

// Acelerômetro ±2g
static constexpr float ACCEL_SENSITIVITY = 16384.0f;

// Giroscópio ±250 °/s
static constexpr float GYRO_SENSITIVITY = 131.0f;

// AK09916
static constexpr float MAG_SENSITIVITY = 0.15f;

// Temperatura
static constexpr float TEMP_SENSITIVITY = 333.87f;
static constexpr float TEMP_OFFSET = 21.0f;

// =============================================================
// Construtor
// =============================================================

ICM20948::ICM20948(
    TwoWire& wire,
    uint8_t endereco
)
    : _wire(wire),
      _enderecoICM(endereco),
      _enderecoMag(0x0C),
      _dados{},
      _conectado(false)
{
}

// =============================================================
// BEGIN
// =============================================================

bool ICM20948::begin(
    int sda,
    int scl,
    uint32_t frequencia
)
{
    _conectado = false;
    _dados = {};

    _wire.begin(
        sda,
        scl
    );

    _wire.setClock(
        frequencia
    );

    delay(100);

    if (!inicializarICM())
        return false;

    if (!inicializarMagnetometro())
        return false;

    _conectado = true;

    return true;
}

// =============================================================
// INICIALIZA ICM
// =============================================================

bool ICM20948::inicializarICM()
{
    uint8_t who = ler(
        _enderecoICM,
        ICM_WHO_AM_I
    );

    if (who != 0xEA)
        return false;

    // ---------------------------------------------------------
    // Acorda sensor
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_PWR_MGMT_1,
        0x01
    ))
    {
        return false;
    }

    delay(100);

    // ---------------------------------------------------------
    // Liga acelerômetro e giroscópio
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_PWR_MGMT_2,
        0x00
    ))
    {
        return false;
    }

    // ---------------------------------------------------------
    // Desabilita I2C Master interno
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_USER_CTRL,
        0x00
    ))
    {
        return false;
    }

    delay(10);

    // ---------------------------------------------------------
    // BYPASS para acessar AK09916 diretamente
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_INT_PIN_CFG,
        0x02
    ))
    {
        return false;
    }

    delay(10);

    // ---------------------------------------------------------
    // Banco 2
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_REG_BANK_SEL,
        0x20
    ))
    {
        return false;
    }

    // ---------------------------------------------------------
    // Gyro:
    //
    // FS = ±250 °/s
    // DLPF habilitado
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_GYRO_CONFIG_1,
        0x01
    ))
    {
        return false;
    }

    // ---------------------------------------------------------
    // Taxa do gyro
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_GYRO_SMPLRT_DIV,
        0x04
    ))
    {
        return false;
    }

    // ---------------------------------------------------------
    // Acelerômetro:
    //
    // FS = ±2g
    // DLPF habilitado
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_ACCEL_CONFIG,
        0x01
    ))
    {
        return false;
    }

    // ---------------------------------------------------------
    // Taxa do acelerômetro
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_ACCEL_SMPLRT_DIV_1,
        0x00
    ))
    {
        return false;
    }

    if (!escrever(
        _enderecoICM,
        ICM_ACCEL_SMPLRT_DIV_2,
        0x04
    ))
    {
        return false;
    }

    // ---------------------------------------------------------
    // Volta banco 0
    // ---------------------------------------------------------

    if (!escrever(
        _enderecoICM,
        ICM_REG_BANK_SEL,
        0x00
    ))
    {
        return false;
    }

    delay(10);

    return true;
}

// =============================================================
// INICIALIZA MAGNETÔMETRO
// =============================================================

bool ICM20948::inicializarMagnetometro()
{
    uint8_t wia2 = ler(
        _enderecoMag,
        AK_WIA2
    );

    if (wia2 != 0x09)
        return false;

    // Reset
    if (!escrever(
        _enderecoMag,
        AK_CNTL3,
        0x01
    ))
    {
        return false;
    }

    delay(100);

    // 100 Hz
    if (!escrever(
        _enderecoMag,
        AK_CNTL2,
        0x08
    ))
    {
        return false;
    }

    delay(10);

    return true;
}

// =============================================================
// UPDATE
// =============================================================

bool ICM20948::update()
{
    if (!_conectado)
        return false;

    bool sucesso = true;

    if (!lerAcelerometro())
        sucesso = false;

    if (!lerGiroscopio())
        sucesso = false;

    if (!lerTemperatura())
        sucesso = false;

    if (!lerMagnetometro())
        sucesso = false;

    return sucesso;
}

// =============================================================
// GET DATA
// =============================================================

const ICM20948Data& ICM20948::getData() const
{
    return _dados;
}

// =============================================================
// CONNECTION
// =============================================================

bool ICM20948::isConnected() const
{
    return _conectado;
}

// =============================================================
// ESCREVER
// =============================================================

bool ICM20948::escrever(
    uint8_t endereco,
    uint8_t reg,
    uint8_t valor
)
{
    _wire.beginTransmission(endereco);

    _wire.write(reg);
    _wire.write(valor);

    return _wire.endTransmission() == 0;
}

// =============================================================
// LER
// =============================================================

uint8_t ICM20948::ler(
    uint8_t endereco,
    uint8_t reg
)
{
    _wire.beginTransmission(endereco);

    _wire.write(reg);

    if (_wire.endTransmission(false) != 0)
        return 0xFF;

    uint8_t recebidos =
        _wire.requestFrom(
            endereco,
            (uint8_t)1
        );

    if (
        recebidos == 1 &&
        _wire.available()
    )
    {
        return _wire.read();
    }

    return 0xFF;
}

// =============================================================
// LER BYTES
// =============================================================

bool ICM20948::lerBytes(
    uint8_t endereco,
    uint8_t reg,
    uint8_t* dados,
    uint8_t quantidade
)
{
    _wire.beginTransmission(endereco);

    _wire.write(reg);

    if (_wire.endTransmission(false) != 0)
        return false;

    uint8_t recebidos =
        _wire.requestFrom(
            endereco,
            quantidade
        );

    if (recebidos != quantidade)
        return false;

    for (uint8_t i = 0; i < quantidade; i++)
    {
        if (!_wire.available())
            return false;

        dados[i] = _wire.read();
    }

    return true;
}

// =============================================================
// INT16
// =============================================================

int16_t ICM20948::montarInt16(
    uint8_t alto,
    uint8_t baixo
)
{
    return (int16_t)(
        ((uint16_t)alto << 8) |
        baixo
    );
}

// =============================================================
// ACELERÔMETRO
// =============================================================

bool ICM20948::lerAcelerometro()
{
    uint8_t buffer[6];

    if (!lerBytes(
        _enderecoICM,
        ICM_ACCEL_XOUT_H,
        buffer,
        6
    ))
    {
        return false;
    }

    _dados.accelRawX =
        montarInt16(
            buffer[0],
            buffer[1]
        );

    _dados.accelRawY =
        montarInt16(
            buffer[2],
            buffer[3]
        );

    _dados.accelRawZ =
        montarInt16(
            buffer[4],
            buffer[5]
        );

    _dados.accelX =
        _dados.accelRawX /
        ACCEL_SENSITIVITY;

    _dados.accelY =
        _dados.accelRawY /
        ACCEL_SENSITIVITY;

    _dados.accelZ =
        _dados.accelRawZ /
        ACCEL_SENSITIVITY;

    return true;
}

// =============================================================
// GIROSCÓPIO
// =============================================================

bool ICM20948::lerGiroscopio()
{
    uint8_t buffer[6];

    if (!lerBytes(
        _enderecoICM,
        ICM_GYRO_XOUT_H,
        buffer,
        6
    ))
    {
        return false;
    }

    _dados.gyroRawX =
        montarInt16(
            buffer[0],
            buffer[1]
        );

    _dados.gyroRawY =
        montarInt16(
            buffer[2],
            buffer[3]
        );

    _dados.gyroRawZ =
        montarInt16(
            buffer[4],
            buffer[5]
        );

    _dados.gyroX =
        _dados.gyroRawX /
        GYRO_SENSITIVITY;

    _dados.gyroY =
        _dados.gyroRawY /
        GYRO_SENSITIVITY;

    _dados.gyroZ =
        _dados.gyroRawZ /
        GYRO_SENSITIVITY;

    return true;
}

// =============================================================
// TEMPERATURA
// =============================================================

bool ICM20948::lerTemperatura()
{
    uint8_t buffer[2];

    if (!lerBytes(
        _enderecoICM,
        ICM_TEMP_OUT_H,
        buffer,
        2
    ))
    {
        return false;
    }

    _dados.tempRaw =
        montarInt16(
            buffer[0],
            buffer[1]
        );

    _dados.temperature =
        (
            (float)_dados.tempRaw /
            TEMP_SENSITIVITY
        ) +
        TEMP_OFFSET;

    return true;
}

// =============================================================
// MAGNETÔMETRO
// =============================================================

bool ICM20948::lerMagnetometro()
{
    uint8_t buffer[9];

    if (!lerBytes(
        _enderecoMag,
        AK_ST1,
        buffer,
        9
    ))
    {
        _dados.magDataReady = false;
        return false;
    }

    _dados.magDataReady =
        (buffer[0] & 0x01);

    if (!_dados.magDataReady)
        return true;

    _dados.magRawX =
        montarInt16(
            buffer[2],
            buffer[1]
        );

    _dados.magRawY =
        montarInt16(
            buffer[4],
            buffer[3]
        );

    _dados.magRawZ =
        montarInt16(
            buffer[6],
            buffer[5]
        );

    _dados.magOverflow =
        (buffer[8] & 0x08);

    if (_dados.magOverflow)
        return true;

    _dados.magX =
        _dados.magRawX *
        MAG_SENSITIVITY;

    _dados.magY =
        _dados.magRawY *
        MAG_SENSITIVITY;

    _dados.magZ =
        _dados.magRawZ *
        MAG_SENSITIVITY;

    return true;
}