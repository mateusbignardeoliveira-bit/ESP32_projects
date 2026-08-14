#include "TCA.h"


// ============================================================
// CONSTRUTOR
// ============================================================

TCA::TCA(uint8_t enderecoTCA)
{

    endereco = enderecoTCA;

    canalAtual = 255;

    pinoSDA = -1;
    pinoSCL = -1;

}



// ============================================================
// BEGIN
// ============================================================

void TCA::begin(
    int SDA,
    int SCL,
    uint32_t frequencia
)
{

    pinoSDA = SDA;
    pinoSCL = SCL;


    // --------------------------------------------------------
    // Inicialização do barramento
    //
    // Se os pinos foram especificados, utiliza-os.
    // Caso contrário, usa a configuração padrão da placa.
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // Configura frequência I2C
    // --------------------------------------------------------

    Wire.setClock(frequencia);


    // --------------------------------------------------------
    // Por segurança, inicia com todos os canais desligados
    // --------------------------------------------------------

    desligarCanais();

}



// ============================================================
// SELECIONAR CANAL
// ============================================================

bool TCA::selecionarCanal(
    uint8_t canal
)
{

    // --------------------------------------------------------
    // O TCA9548A possui canais 0 até 7
    // --------------------------------------------------------

    if(canal > 7)
    {
        return false;
    }


    // --------------------------------------------------------
    // Inicia transmissão para o TCA
    // --------------------------------------------------------

    Wire.beginTransmission(endereco);


    // --------------------------------------------------------
    // Ativa somente o canal solicitado
    //
    // Canal 0 → 00000001
    // Canal 1 → 00000010
    // Canal 2 → 00000100
    // ...
    // Canal 7 → 10000000
    // --------------------------------------------------------

    Wire.write(
        (uint8_t)(1 << canal)
    );


    uint8_t erro =
        Wire.endTransmission();


    // --------------------------------------------------------
    // Verifica resultado
    // --------------------------------------------------------

    if(erro != 0)
    {
        return false;
    }


    // --------------------------------------------------------
    // Guarda canal ativo
    // --------------------------------------------------------

    canalAtual = canal;


    return true;

}



// ============================================================
// DESLIGAR TODOS OS CANAIS
// ============================================================

bool TCA::desligarCanais()
{

    Wire.beginTransmission(endereco);


    // --------------------------------------------------------
    // 0x00 = nenhum canal ativo
    // --------------------------------------------------------

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



// ============================================================
// GET CANAL ATUAL
// ============================================================

uint8_t TCA::getCanalAtual()
{

    return canalAtual;

}



// ============================================================
// GET ENDEREÇO
// ============================================================

uint8_t TCA::getEndereco()
{

    return endereco;

}