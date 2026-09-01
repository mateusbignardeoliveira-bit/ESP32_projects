#include <arduino.h>
#include <Wire.h>

#define SDA_PIN 19
#define SCL_PIN 23

#define ICM_ADDR 0x68
#define AK_ADDR  0x0C

// ============================================================
// ICM-20948 - Banco 0
// ============================================================

#define REG_BANK_SEL    0x7F

#define WHO_AM_I        0x00
#define USER_CTRL       0x03
#define PWR_MGMT_1      0x06
#define INT_PIN_CFG     0x0F

#define ACCEL_XOUT_H    0x2D
#define GYRO_XOUT_H     0x33
#define TEMP_OUT_H      0x39


// ============================================================
// AK09916
// ============================================================

#define AK_WIA2         0x01

#define AK_ST1          0x10

#define AK_HXL          0x11
#define AK_HXH          0x12
#define AK_HYL          0x13
#define AK_HYH          0x14
#define AK_HZL          0x15
#define AK_HZH          0x16

#define AK_ST2          0x18

#define AK_CNTL2        0x31
#define AK_CNTL3        0x32


// ============================================================
// Constantes
// ============================================================

// Acelerômetro configurado no padrão ±2g
#define ACCEL_SENSITIVITY 16384.0f

// Giroscópio configurado no padrão ±250 °/s
#define GYRO_SENSITIVITY 131.0f

// AK09916
#define MAG_SENSITIVITY 0.15f

// Temperatura do ICM-20948
#define TEMP_SENSITIVITY 333.87f
#define TEMP_OFFSET      21.0f


// ============================================================
// Estruturas
// ============================================================

struct DadosIMU
{
    // RAW
    int16_t accelRawX;
    int16_t accelRawY;
    int16_t accelRawZ;

    int16_t gyroRawX;
    int16_t gyroRawY;
    int16_t gyroRawZ;

    int16_t tempRaw;

    int16_t magRawX;
    int16_t magRawY;
    int16_t magRawZ;

    // Acelerômetro
    float accelX_g;
    float accelY_g;
    float accelZ_g;

    float accelX_ms2;
    float accelY_ms2;
    float accelZ_ms2;

    // Giroscópio
    float gyroX_dps;
    float gyroY_dps;
    float gyroZ_dps;

    float gyroX_rad;
    float gyroY_rad;
    float gyroZ_rad;

    // Temperatura
    float temperatura;

    // Magnetômetro
    float magX_uT;
    float magY_uT;
    float magZ_uT;

    // Status
    bool magDataReady;
    bool magOverflow;
};


// ============================================================
// Escreve registrador
// ============================================================

void escrever(
    uint8_t endereco,
    uint8_t reg,
    uint8_t valor
)
{
    Wire.beginTransmission(endereco);

    Wire.write(reg);
    Wire.write(valor);

    Wire.endTransmission();
}


// ============================================================
// Lê um registrador
// ============================================================

uint8_t ler(
    uint8_t endereco,
    uint8_t reg
)
{
    Wire.beginTransmission(endereco);

    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return 0xFF;

    uint8_t recebidos =
        Wire.requestFrom(endereco, (uint8_t)1);

    if (recebidos == 1 && Wire.available())
        return Wire.read();

    return 0xFF;
}


// ============================================================
// Lê vários bytes
// ============================================================

bool lerBytes(
    uint8_t endereco,
    uint8_t reg,
    uint8_t *dados,
    uint8_t quantidade
)
{
    Wire.beginTransmission(endereco);

    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return false;

    uint8_t recebidos =
        Wire.requestFrom(endereco, quantidade);

    if (recebidos != quantidade)
        return false;

    for (uint8_t i = 0; i < quantidade; i++)
    {
        dados[i] = Wire.read();
    }

    return true;
}


// ============================================================
// Converte dois bytes BIG ENDIAN em int16
// ============================================================

int16_t montarInt16(
    uint8_t alto,
    uint8_t baixo
)
{
    return (int16_t)(
        ((uint16_t)alto << 8) |
        baixo
    );
}


// ============================================================
// Inicializa ICM-20948
// ============================================================

bool iniciarICM()
{
    // --------------------------------------------------------
    // Verifica WHO_AM_I
    // --------------------------------------------------------

    uint8_t who = ler(ICM_ADDR, WHO_AM_I);

    Serial.print("ICM WHO_AM_I: 0x");
    Serial.println(who, HEX);

    if (who != 0xEA)
    {
        Serial.println(
            "ERRO: ICM-20948 nao encontrado!"
        );

        return false;
    }


    // --------------------------------------------------------
    // Acorda o ICM
    // --------------------------------------------------------

    escrever(
        ICM_ADDR,
        PWR_MGMT_1,
        0x01
    );

    delay(100);


    // --------------------------------------------------------
    // Desativa I2C Master interno
    // --------------------------------------------------------

    escrever(
        ICM_ADDR,
        USER_CTRL,
        0x00
    );

    delay(10);


    // --------------------------------------------------------
    // Habilita I2C BYPASS
    // --------------------------------------------------------

    escrever(
        ICM_ADDR,
        INT_PIN_CFG,
        0x02
    );

    delay(10);

    return true;
}


// ============================================================
// Inicializa AK09916
// ============================================================

bool iniciarMagnetometro()
{
    uint8_t wia2 =
        ler(AK_ADDR, AK_WIA2);

    Serial.print("AK09916 WIA2: 0x");
    Serial.println(wia2, HEX);

    if (wia2 != 0x09)
    {
        Serial.println(
            "ERRO: AK09916 nao encontrado!"
        );

        return false;
    }

    Serial.println("AK09916 encontrado!");


    // --------------------------------------------------------
    // Reset
    // --------------------------------------------------------

    escrever(
        AK_ADDR,
        AK_CNTL3,
        0x01
    );

    delay(100);


    // --------------------------------------------------------
    // Modo contínuo 100 Hz
    // CNTL2 = 0x08
    // --------------------------------------------------------

    escrever(
        AK_ADDR,
        AK_CNTL2,
        0x08
    );

    delay(10);

    return true;
}


// ============================================================
// Lê acelerômetro
// ============================================================

bool lerAcelerometro(DadosIMU &dados)
{
    uint8_t buffer[6];

    if (!lerBytes(
            ICM_ADDR,
            ACCEL_XOUT_H,
            buffer,
            6))
    {
        return false;
    }


    dados.accelRawX =
        montarInt16(buffer[0], buffer[1]);

    dados.accelRawY =
        montarInt16(buffer[2], buffer[3]);

    dados.accelRawZ =
        montarInt16(buffer[4], buffer[5]);


    // --------------------------------------------------------
    // RAW -> g
    // --------------------------------------------------------

    dados.accelX_g =
        dados.accelRawX / ACCEL_SENSITIVITY;

    dados.accelY_g =
        dados.accelRawY / ACCEL_SENSITIVITY;

    dados.accelZ_g =
        dados.accelRawZ / ACCEL_SENSITIVITY;


    // --------------------------------------------------------
    // g -> m/s²
    // --------------------------------------------------------

    dados.accelX_ms2 =
        dados.accelX_g * 9.80665f;

    dados.accelY_ms2 =
        dados.accelY_g * 9.80665f;

    dados.accelZ_ms2 =
        dados.accelZ_g * 9.80665f;

    return true;
}


// ============================================================
// Lê giroscópio
// ============================================================

bool lerGiroscopio(DadosIMU &dados)
{
    uint8_t buffer[6];

    if (!lerBytes(
            ICM_ADDR,
            GYRO_XOUT_H,
            buffer,
            6))
    {
        return false;
    }


    dados.gyroRawX =
        montarInt16(buffer[0], buffer[1]);

    dados.gyroRawY =
        montarInt16(buffer[2], buffer[3]);

    dados.gyroRawZ =
        montarInt16(buffer[4], buffer[5]);


    // --------------------------------------------------------
    // RAW -> °/s
    // --------------------------------------------------------

    dados.gyroX_dps =
        dados.gyroRawX / GYRO_SENSITIVITY;

    dados.gyroY_dps =
        dados.gyroRawY / GYRO_SENSITIVITY;

    dados.gyroZ_dps =
        dados.gyroRawZ / GYRO_SENSITIVITY;


    // --------------------------------------------------------
    // °/s -> rad/s
    // --------------------------------------------------------

    dados.gyroX_rad =
        dados.gyroX_dps * DEG_TO_RAD;

    dados.gyroY_rad =
        dados.gyroY_dps * DEG_TO_RAD;

    dados.gyroZ_rad =
        dados.gyroZ_dps * DEG_TO_RAD;

    return true;
}


// ============================================================
// Lê temperatura
// ============================================================

bool lerTemperatura(DadosIMU &dados)
{
    uint8_t buffer[2];

    if (!lerBytes(
            ICM_ADDR,
            TEMP_OUT_H,
            buffer,
            2))
    {
        return false;
    }


    dados.tempRaw =
        montarInt16(buffer[0], buffer[1]);


    // --------------------------------------------------------
    // Conversão conforme sensor interno do ICM-20948
    // --------------------------------------------------------

    dados.temperatura =
        ((float)dados.tempRaw / TEMP_SENSITIVITY)
        + TEMP_OFFSET;

    return true;
}


// ============================================================
// Lê magnetômetro
// ============================================================

bool lerMagnetometro(DadosIMU &dados)
{
    uint8_t buffer[9];

    /*
        buffer:

        0 = ST1
        1 = HXL
        2 = HXH
        3 = HYL
        4 = HYH
        5 = HZL
        6 = HZH
        7 = reservado
        8 = ST2
    */

    if (!lerBytes(
            AK_ADDR,
            AK_ST1,
            buffer,
            9))
    {
        dados.magDataReady = false;

        return false;
    }


    // --------------------------------------------------------
    // ST1
    // --------------------------------------------------------

    uint8_t st1 = buffer[0];

    dados.magDataReady =
        (st1 & 0x01);


    if (!dados.magDataReady)
        return true;


    // --------------------------------------------------------
    // Dados magnéticos
    // --------------------------------------------------------

    dados.magRawX =
        montarInt16(buffer[2], buffer[1]);

    dados.magRawY =
        montarInt16(buffer[4], buffer[3]);

    dados.magRawZ =
        montarInt16(buffer[6], buffer[5]);


    // --------------------------------------------------------
    // ST2
    // --------------------------------------------------------

    uint8_t st2 = buffer[8];

    dados.magOverflow =
        (st2 & 0x08);


    if (dados.magOverflow)
        return true;


    // --------------------------------------------------------
    // RAW -> µT
    // --------------------------------------------------------

    dados.magX_uT =
        dados.magRawX * MAG_SENSITIVITY;

    dados.magY_uT =
        dados.magRawY * MAG_SENSITIVITY;

    dados.magZ_uT =
        dados.magRawZ * MAG_SENSITIVITY;

    return true;
}


// ============================================================
// Lê todos os sensores
// ============================================================

bool atualizar(DadosIMU &dados)
{
    bool sucesso = true;


    if (!lerAcelerometro(dados))
        sucesso = false;


    if (!lerGiroscopio(dados))
        sucesso = false;


    if (!lerTemperatura(dados))
        sucesso = false;


    if (!lerMagnetometro(dados))
        sucesso = false;


    return sucesso;
}


// ============================================================
// Mostra dados
// ============================================================

void imprimirDados(const DadosIMU &dados)
{
    Serial.println(
        "----------------------------------------"
    );


    // ========================================================
    // ACELERÔMETRO
    // ========================================================

    Serial.println("ACELEROMETRO");

    Serial.print("RAW X: ");
    Serial.print(dados.accelRawX);

    Serial.print(" | Y: ");
    Serial.print(dados.accelRawY);

    Serial.print(" | Z: ");
    Serial.println(dados.accelRawZ);


    Serial.print("g    X: ");
    Serial.print(dados.accelX_g, 4);

    Serial.print(" | Y: ");
    Serial.print(dados.accelY_g, 4);

    Serial.print(" | Z: ");
    Serial.println(dados.accelZ_g, 4);


    Serial.print("m/s2 X: ");
    Serial.print(dados.accelX_ms2, 4);

    Serial.print(" | Y: ");
    Serial.print(dados.accelY_ms2, 4);

    Serial.print(" | Z: ");
    Serial.println(dados.accelZ_ms2, 4);


    // ========================================================
    // GIROSCÓPIO
    // ========================================================

    Serial.println();
    Serial.println("GIROSCOPIO");

    Serial.print("RAW X: ");
    Serial.print(dados.gyroRawX);

    Serial.print(" | Y: ");
    Serial.print(dados.gyroRawY);

    Serial.print(" | Z: ");
    Serial.println(dados.gyroRawZ);


    Serial.print("deg/s X: ");
    Serial.print(dados.gyroX_dps, 3);

    Serial.print(" | Y: ");
    Serial.print(dados.gyroY_dps, 3);

    Serial.print(" | Z: ");
    Serial.println(dados.gyroZ_dps, 3);


    Serial.print("rad/s X: ");
    Serial.print(dados.gyroX_rad, 3);

    Serial.print(" | Y: ");
    Serial.print(dados.gyroY_rad, 3);

    Serial.print(" | Z: ");
    Serial.println(dados.gyroZ_rad, 3);


    // ========================================================
    // TEMPERATURA
    // ========================================================

    Serial.println();
    Serial.println("TEMPERATURA");

    Serial.print("RAW: ");
    Serial.println(dados.tempRaw);

    Serial.print("Temperatura: ");
    Serial.print(dados.temperatura, 2);
    Serial.println(" °C");


    // ========================================================
    // MAGNETÔMETRO
    // ========================================================

    Serial.println();
    Serial.println("MAGNETOMETRO AK09916");

    Serial.print("DRDY: ");
    Serial.println(
        dados.magDataReady ? "SIM" : "NAO"
    );


    if (dados.magDataReady)
    {
        Serial.print("RAW X: ");
        Serial.print(dados.magRawX);

        Serial.print(" | Y: ");
        Serial.print(dados.magRawY);

        Serial.print(" | Z: ");
        Serial.println(dados.magRawZ);


        Serial.print("uT X: ");
        Serial.print(dados.magX_uT, 2);

        Serial.print(" | Y: ");
        Serial.print(dados.magY_uT, 2);

        Serial.print(" | Z: ");
        Serial.println(dados.magZ_uT, 2);


        Serial.print("Overflow: ");

        if (dados.magOverflow)
            Serial.println("SIM");
        else
            Serial.println("NAO");
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );

    Wire.setClock(400000);

    delay(500);


    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "       ICM-20948 COMPLETO"
    );

    Serial.println(
        "========================================"
    );

    Serial.println();


    // --------------------------------------------------------
    // ICM
    // --------------------------------------------------------

    if (!iniciarICM())
    {
        while (true)
        {
            delay(1000);
        }
    }


    // --------------------------------------------------------
    // Magnetômetro
    // --------------------------------------------------------

    if (!iniciarMagnetometro())
    {
        while (true)
        {
            delay(1000);
        }
    }


    Serial.println();
    Serial.println(
        "Todos os sensores inicializados!"
    );

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    DadosIMU dados;


    if (atualizar(dados))
    {
        imprimirDados(dados);
    }
    else
    {
        Serial.println(
            "Erro lendo ICM-20948"
        );
    }


    delay(200);
}
