#include "TCA.h"


TCA::TCA(uint8_t enderecoTCA)
{
    endereco = enderecoTCA;
    canalAtual = 255;

    pinoSDA = -1;
    pinoSCL = -1;
}


void TCA::begin(
    int SDA,
    int SCL,
    uint32_t frequencia
)
{
    pinoSDA = SDA;
    pinoSCL = SCL;

    if(
        pinoSDA >= 0 &&
        pinoSCL >= 0
    )
    {
        Wire.begin(
            pinoSDA,
            pinoSCL
        );
    }
    else
    {
        Wire.begin();
    }

    Wire.setClock(
        frequencia
    );

    desligarCanais();
}


bool TCA::selecionarCanal(
    uint8_t canal
)
{
    if(canal > 7)
    {
        return false;
    }

    // --------------------------------------------------------
    // Já está nesse canal.
    // Não faz outra transmissão I2C.
    // --------------------------------------------------------

    if(canalAtual == canal)
    {
        return true;
    }


    Wire.beginTransmission(
        endereco
    );

    Wire.write(
        (uint8_t)(1 << canal)
    );

    uint8_t erro =
        Wire.endTransmission();


    if(erro != 0)
    {
        return false;
    }

    canalAtual =
        canal;

    return true;
}


bool TCA::desligarCanais()
{
    if(canalAtual == 255)
    {
        return true;
    }

    Wire.beginTransmission(
        endereco
    );

    Wire.write(0x00);

    uint8_t erro =
        Wire.endTransmission();


    if(erro != 0)
    {
        return false;
    }

    canalAtual = 255;

    return true;
}


uint8_t TCA::getCanalAtual()
{
    return canalAtual;
}


uint8_t TCA::getEndereco()
{
    return endereco;
}