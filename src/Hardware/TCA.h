#ifndef TCA_H
#define TCA_H

#include <Arduino.h>
#include <Wire.h>


class TCA
{

private:

    uint8_t endereco;


public:

    // Construtor
    TCA(uint8_t enderecoTCA = 0x70);


    // Inicia o I2C
    void begin();


    // Seleciona um canal do TCA
    bool selecionarCanal(uint8_t canal);


    // Retorna o endereço do TCA
    uint8_t getEndereco();

};


#endif