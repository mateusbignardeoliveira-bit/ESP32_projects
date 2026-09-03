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

        EXECUTANDO_GIRO,

        BUSCANDO_LINHA,

        OBSTACULO_GIRO_1,
        OBSTACULO_RETO_1,
        OBSTACULO_GIRO_2,
        OBSTACULO_RETO_2,
        OBSTACULO_GIRO_3,
        OBSTACULO_RETO_3,
        OBSTACULO_GIRO_4,
        OBSTACULO_BUSCANDO_LINHA,

        STOP_CINZA,

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

    bool paradoPorCinza() const;


    // --------------------------------------------------------
    // Vermelho
    // --------------------------------------------------------

    void pararPorVermelho();


    // --------------------------------------------------------
    // Configuração do lado do obstáculo
    //
    // -1 = esquerda
    // +1 = direita
    // --------------------------------------------------------

    void setLadoObstaculo(int lado);

    void pararPorCinza();


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
    // Busca
    // --------------------------------------------------------

    unsigned long inicioBusca;

    unsigned long tempoMaximoBusca;


    // --------------------------------------------------------
    // Obstáculo
    // --------------------------------------------------------

    int ladoObstaculo;

    bool ladoObstaculoDefinido;

    unsigned long inicioTrechoObstaculo;

    unsigned long tempoRetoObstaculo1;

    unsigned long tempoRetoObstaculo2;

    unsigned long tempoMaximoRetoObstaculo3;


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


    void iniciarGiro(
        AcaoGiro acao
    );


    void processarGiro(
        const LinhaData& linha
    );


    void iniciarObstaculo();


    void processarObstaculo(
        const LinhaData& linha
    );


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

    void reAntesDaCurva();


};

#endif