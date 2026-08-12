#include "TCA.h"


TCA::TCA(uint8_t enderecoTCA)
{
    endereco = enderecoTCA;
}



void TCA::begin()
{
    Wire.begin();
}



bool TCA::selecionarCanal(uint8_t canal)
{

    // O TCA9548A possui canais 0 até 7
    if(canal > 7)
    {
        return false;
    }


    Wire.beginTransmission(endereco);


    // Liga apenas o canal escolhido
    Wire.write(1 << canal);


    uint8_t erro = Wire.endTransmission();


    return erro == 0;
}



uint8_t TCA::getEndereco()
{
    return endereco;
}