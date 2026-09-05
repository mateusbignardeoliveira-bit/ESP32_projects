#ifndef MAQUINA_ESTADOS_H
#define MAQUINA_ESTADOS_H

#include <Arduino.h>

#include "../sensores/LinhaAnalise.h"
#include "../sensores/AS7341Analise.h"
#include "../sensores/TOFAnalise.h"
#include "../sensores/IMU.h"

#include "../controle/ControleLinha.h"
#include "../controle/ControleGiro.h"
#include "../controle/ControleObstaculo.h"
#include "../controle/Verde.h"

#include "../Hardware/MotorControlador.h"


class MaquinaEstados
{
public:

    enum Estado
    {
        INICIO,

        SEGUINDO_LINHA,

        AVALIANDO_MARCA,

        // ----------------------------------------------------
        // Giro normal
        // ----------------------------------------------------

        EXECUTANDO_GIRO,

        // ----------------------------------------------------
        // Curva verde
        // ----------------------------------------------------

        GIRO_VERDE_45,

        CURVA_VERDE_CONTINUA,

        CORRECAO_CURVA_VERDE,

        // ----------------------------------------------------
        // Curva preta
        // ----------------------------------------------------

        AVANCO_CURVA_PRETA,

        CURVA_PRETA,

        BUSCANDO_LINHA,

        // ----------------------------------------------------
        // Obstáculo
        // ----------------------------------------------------

        OBSTACULO_GIRO_1,

        OBSTACULO_RETO_INICIAL,

        OBSTACULO_GIRO_2,

        OBSTACULO_ORBITA_RETO,

        OBSTACULO_ORBITA_GIRO,

        OBSTACULO_GIRO_FINAL,

        STOP_VERMELHO
    };


    enum AcaoGiro
    {
        NENHUM_GIRO,

        GIRO_ESQUERDA,

        GIRO_DIREITA,

        GIRO_180
    };


    MaquinaEstados(
        LinhaAnalise& linha,
        AS7341Analise& corEsquerda,
        AS7341Analise& corDireita,
        Verde& verde,
        TOFAnalise& tof,
        IMU& imu,
        ControleLinha& controleLinha,
        ControleGiro& controleGiro,
        ControleObstaculo& controleObstaculo,
        MotorControlador& motores
    );


    void begin();


    void resetExecucao();


    void update(
        const LinhaData& dadosLinha,
        const AS7341Data& dadosCorEsquerda,
        const AS7341Data& dadosCorDireita
    );


    Estado getEstado() const;


    bool parado() const;

    bool paradoPorVermelho() const;


    // --------------------------------------------------------
    // Vermelho
    // --------------------------------------------------------

    void pararPorVermelho();


private:

    LinhaAnalise& analiseLinha;

    AS7341Analise& sensorCorEsquerda;

    AS7341Analise& sensorCorDireita;

    Verde& analiseVerde;

    TOFAnalise& analiseTOF;

    IMU& sensorIMU;


    ControleLinha& controleLinha;

    ControleGiro& controleGiro;

    ControleObstaculo& controleObstaculo;

    MotorControlador& motores;


    Estado estado;

    AcaoGiro acaoGiro;


    // --------------------------------------------------------
    // Tendência antes da avaliação
    // --------------------------------------------------------

    float tendenciaAntes;


    // --------------------------------------------------------
    // Avaliação
    // --------------------------------------------------------

    unsigned long inicioAvaliacao;

    unsigned long tempoMinimoAvaliacao;

    unsigned long tempoMaximoAvaliacao;


    // --------------------------------------------------------
    // Busca normal
    // --------------------------------------------------------

    unsigned long inicioBusca;

    unsigned long tempoMaximoBusca;


    // --------------------------------------------------------
    // Curva verde
    // --------------------------------------------------------

    AcaoGiro direcaoCurvaVerde;

    unsigned long inicioCurvaVerde;

    unsigned long inicioCorrecaoCurvaVerde;


    // --------------------------------------------------------
    // Curva preta
    // --------------------------------------------------------

    AcaoGiro direcaoCurvaPreta;

    unsigned long inicioAvancoCurvaPreta;


    // --------------------------------------------------------
    // Obstáculo
    // --------------------------------------------------------

    unsigned long inicioTrechoObstaculo;

    unsigned long tempoRetoObstaculoInicial;

    unsigned long tempoRetoObstaculoOrbita;

    int leiturasPretasObstaculo;


    // --------------------------------------------------------
    // Funções internas
    // --------------------------------------------------------

    void entrarEstado(
        Estado novoEstado
    );


    void processarSeguindoLinha(
        const LinhaData& linha
    );


    void iniciarAvaliacao(
        const LinhaData& linha
    );


    void processarAvaliacao(
        const LinhaData& linha,
        const AS7341Data& dadosCorEsquerda,
        const AS7341Data& dadosCorDireita
    );


    void finalizarAvaliacao(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Giro normal
    // --------------------------------------------------------

    void iniciarGiro(
        AcaoGiro acao
    );


    void processarGiro(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Curva verde
    // --------------------------------------------------------

    void iniciarGiroVerde(
        AcaoGiro acao
    );


    void processarGiroVerde45(
        const LinhaData& linha
    );


    void processarCurvaVerdeContinua(
        const LinhaData& linha
    );


    void iniciarCorrecaoCurvaVerde(
        const LinhaData& linha
    );


    void processarCorrecaoCurvaVerde(
        const LinhaData& linha
    );


    bool sensorCentralEncontrouPreto(
        const LinhaData& linha
    );


    bool sensorCentroNaLinha(
        const LinhaData& linha
    );


    AcaoGiro determinarDirecaoCorrecao(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Curva preta
    // --------------------------------------------------------

    void iniciarCurvaPreta();

    void processarAvancoCurvaPreta();

    void iniciarRotacaoCurvaPreta();

    void processarCurvaPreta(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Obstáculo
    // --------------------------------------------------------

    void iniciarObstaculo();


    void processarObstaculo(
        const LinhaData& linha
    );


    void iniciarRetoObstaculo(
        unsigned long duracao
    );


    bool linhaPretaConfirmada(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Busca
    // --------------------------------------------------------

    void iniciarBuscaLinha();


    bool linhaEncontrada(
        const LinhaData& linha
    );


    bool todosBrancos(
        const LinhaData& linha
    );


    int contarSensoresPretos(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Ré antes de curvas normais
    // --------------------------------------------------------

    void reAntesDaCurva();
};

#endif