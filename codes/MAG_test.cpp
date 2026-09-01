#include <arduino.h>
#include <Wire.h>

#define SDA_PIN 19
#define SCL_PIN 23

#define ICM_ADDR 0x68
#define AK_ADDR  0x0C

// ============================================================
// ICM-20948 - Banco 0
// ============================================================

#define REG_BANK_SEL  0x7F
#define WHO_AM_I      0x00
#define USER_CTRL     0x03
#define PWR_MGMT_1    0x06
#define INT_PIN_CFG   0x0F

// ============================================================
// AK09916
// ============================================================

#define AK_WIA2       0x01

#define AK_ST1        0x10
#define AK_HXL        0x11
#define AK_HXH        0x12
#define AK_HYL        0x13
#define AK_HYH        0x14
#define AK_HZL        0x15
#define AK_HZH        0x16
#define AK_ST2        0x18

#define AK_CNTL2      0x31
#define AK_CNTL3      0x32


// ============================================================
// Escreve um registrador I2C
// ============================================================

void escrever(uint8_t endereco, uint8_t reg, uint8_t valor)
{
    Wire.beginTransmission(endereco);
    Wire.write(reg);
    Wire.write(valor);
    Wire.endTransmission();
}


// ============================================================
// Lê um registrador I2C
// ============================================================

uint8_t ler(uint8_t endereco, uint8_t reg)
{
    Wire.beginTransmission(endereco);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return 0xFF;

    Wire.requestFrom(endereco, (uint8_t)1);

    if (Wire.available())
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

    uint8_t recebidos = Wire.requestFrom(
        endereco,
        quantidade
    );

    if (recebidos != quantidade)
        return false;

    for (uint8_t i = 0; i < quantidade; i++)
    {
        dados[i] = Wire.read();
    }

    return true;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    delay(500);

    Serial.println();
    Serial.println("ICM-20948 + AK09916");
    Serial.println("===================");


    // ========================================================
    // Verifica ICM-20948
    // ========================================================

    uint8_t who = ler(ICM_ADDR, WHO_AM_I);

    Serial.print("ICM WHO_AM_I: 0x");
    Serial.println(who, HEX);

    if (who != 0xEA)
    {
        Serial.println("ERRO: ICM-20948 nao encontrado!");

        while (true)
        {
            delay(1000);
        }
    }


    // ========================================================
    // Acorda o ICM-20948
    // ========================================================

    // CLKSEL = 1
    // SLEEP = 0

    escrever(ICM_ADDR, PWR_MGMT_1, 0x01);

    delay(100);


    // ========================================================
    // Desativa o I2C Master interno
    // ========================================================

    escrever(ICM_ADDR, USER_CTRL, 0x00);

    delay(10);


    // ========================================================
    // Habilita I2C BYPASS
    //
    // Permite que o ESP32 acesse diretamente o AK09916
    // em 0x0C.
    // ========================================================

    escrever(ICM_ADDR, INT_PIN_CFG, 0x02);

    delay(10);


    // ========================================================
    // Verifica AK09916
    // ========================================================

    uint8_t wia2 = ler(AK_ADDR, AK_WIA2);

    Serial.print("AK09916 WIA2: 0x");
    Serial.println(wia2, HEX);

    if (wia2 != 0x09)
    {
        Serial.println("ERRO: AK09916 nao encontrado!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("AK09916 encontrado!");


    // ========================================================
    // Reset do AK09916
    // ========================================================

    escrever(AK_ADDR, AK_CNTL3, 0x01);

    delay(100);


    // ========================================================
    // AK09916 em modo continuo 100 Hz
    // ========================================================

    escrever(AK_ADDR, AK_CNTL2, 0x08);

    delay(10);

    Serial.println("Magnetometro iniciado.");
    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // 9 bytes:
    //
    // 0 = ST1
    // 1 = HXL
    // 2 = HXH
    // 3 = HYL
    // 4 = HYH
    // 5 = HZL
    // 6 = HZH
    // 7 = reservado
    // 8 = ST2
    //
    // ========================================================

    uint8_t dados[9];


    if (!lerBytes(AK_ADDR, AK_ST1, dados, 9))
    {
        Serial.println("Erro lendo magnetometro");
        delay(100);
        return;
    }


    // ========================================================
    // ST1
    // Bit 0 = DRDY
    // ========================================================

    uint8_t st1 = dados[0];


    if (!(st1 & 0x01))
    {
        // Dados ainda não estão prontos
        delay(10);
        return;
    }


    // ========================================================
    // Monta valores de 16 bits
    //
    // AK09916 envia LOW byte primeiro
    // ========================================================

    int16_t rawX =
        (int16_t)((dados[2] << 8) | dados[1]);

    int16_t rawY =
        (int16_t)((dados[4] << 8) | dados[3]);

    int16_t rawZ =
        (int16_t)((dados[6] << 8) | dados[5]);


    // ========================================================
    // ST2
    // ========================================================

    uint8_t st2 = dados[8];


    // Bit 3 = HOFL
    // Overflow do magnetômetro

    if (st2 & 0x08)
    {
        Serial.println("Magnetometro: OVERFLOW!");
        delay(100);
        return;
    }


    // ========================================================
    // Conversão para microtesla
    //
    // AK09916:
    // 0.15 uT / LSB
    // ========================================================

    float mx = rawX * 0.15f;
    float my = rawY * 0.15f;
    float mz = rawZ * 0.15f;


    // ========================================================
    // Mostra no Serial
    // ========================================================

    Serial.print("X: ");
    Serial.print(mx, 2);

    Serial.print(" uT | Y: ");
    Serial.print(my, 2);

    Serial.print(" uT | Z: ");
    Serial.print(mz, 2);

    Serial.println(" uT");


    delay(50);
}