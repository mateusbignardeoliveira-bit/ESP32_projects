#ifndef TCA_H
#define TCA_H

#include <Arduino.h>
#include <Wire.h>


class TCA
{

private:

    // ========================================================
    // Endereço I2C do TCA9548A
    // ========================================================

    uint8_t endereco;


    // ========================================================
    // Estado do canal atualmente selecionado
    //
    // 0 a 7 = canal ativo
    // 255   = nenhum canal
    // ========================================================

    uint8_t canalAtual;


    // ========================================================
    // Pinos I2C
    // ========================================================

    int pinoSDA;
    int pinoSCL;


public:

    // ========================================================
    // Construtor
    //
    // Endereço padrão do TCA9548A:
    // 0x70
    // ========================================================

    TCA(
        uint8_t enderecoTCA = 0x70
    );


    // ========================================================
    // Inicializa o barramento I2C
    //
    // Se SDA/SCL não forem informados, usa os padrões.
    // ========================================================

    void begin(
        int SDA = -1,
        int SCL = -1,
        uint32_t frequencia = 400000    );


    // ========================================================
    // Seleciona um dos 8 canais
    //
    // canal = 0 até 7
    //
    // Retorna:
    // true  = sucesso
    // false = erro
    // ========================================================

    bool selecionarCanal(
        uint8_t canal
    );


    // ========================================================
    // Desliga todos os canais
    // ========================================================

    bool desligarCanais();


    // ========================================================
    // Retorna o canal atualmente selecionado
    //
    // 0 a 7 = canal
    // 255   = nenhum
    // ========================================================

    uint8_t getCanalAtual();


    // ========================================================
    // Retorna o endereço I2C do TCA
    // ========================================================

    uint8_t getEndereco();

};

#endif