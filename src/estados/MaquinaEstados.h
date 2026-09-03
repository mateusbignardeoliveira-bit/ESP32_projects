#ifndef MAQUINA_ESTADOS_H
#define MAQUINA_ESTADOS_H

#include <Arduino.h>

#include "../Hardware/MotorControlador.h"

#include "../sensores/LinhaAnalise.h"
#include "../sensores/AS7341Analise.h"
#include "../sensores/TOFAnalise.h"
#include "../sensores/IMU.h"

#include "../controle/ControleLinha.h"
#include "../controle/ControleGiro.h"
#include "../controle/ControleObstaculo.h"
#include "../controle/Verde.h"


class MaquinaEstados
{
public:

    // ========================================================
    // ESTADOS
    // ========================================================

    enum Estado
    {
        INICIO,

        SEGUINDO_LINHA,

        AVALIANDO_MARCA,

        EXECUTANDO_GIRO,

        BUSCANDO_LINHA,

        // ----------------------------------------------------
        // OBSTÁCULO
        // ----------------------------------------------------

        OBSTACULO_GIRO_1,

        OBSTACULO_RETO_INICIAL,

        OBSTACULO_GIRO_2,

        OBSTACULO_ORBITA_RETO,

        OBSTACULO_ORBITA_GIRO,

        OBSTACULO_GIRO_FINAL,

        // ----------------------------------------------------
        // STOPS
        // ----------------------------------------------------

        STOP_CINZA,

        STOP_VERMELHO
    };


    // ========================================================
    // AÇÕES DE GIRO
    // ========================================================

    enum AcaoGiro
    {
        NENHUM_GIRO,

        GIRO_ESQUERDA,

        GIRO_DIREITA,

        GIRO_180
    };


    // ========================================================
    // CONSTRUTOR
    // ========================================================

    MaquinaEstados(
        LinhaAnalise& linha,
        AS7341Analise& corEsq,
        AS7341Analise& corDir,
        Verde& verde,
        TOFAnalise& tof,
        IMU& imu,
        ControleLinha& controleLinhaRef,
        ControleGiro& controleGiroRef,
        ControleObstaculo& controleObstaculoRef,
        MotorControlador& motoresRef
    );


    // ========================================================
    // CONTROLE
    // ========================================================

    void begin();

    void update(
        const LinhaData& linha,
        const AS7341Data& dadosCorEsquerda,
        const AS7341Data& dadosCorDireita
    );

    void resetExecucao();


    // ========================================================
    // ESTADO
    // ========================================================

    Estado getEstado() const;

    bool parado() const;

    bool paradoPorVermelho() const;

    bool paradoPorCinza() const;


    // ========================================================
    // STOPS
    // ========================================================

    void pararPorVermelho();

    void pararPorCinza();


private:

    // ========================================================
    // REFERÊNCIAS
    // ========================================================

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


    // ========================================================
    // ESTADO ATUAL
    // ========================================================

    Estado estado;

    AcaoGiro acaoGiro;


    // ========================================================
    // AVALIAÇÃO
    // ========================================================

    unsigned long inicioAvaliacao;

    unsigned long tempoMinimoAvaliacao;

    unsigned long tempoMaximoAvaliacao;

    float tendenciaAntes;


    // ========================================================
    // BUSCA
    // ========================================================

    unsigned long inicioBusca;

    unsigned long tempoMaximoBusca;


    // ========================================================
    // OBSTÁCULO
    // ========================================================

    unsigned long inicioTrechoObstaculo;

    unsigned long tempoRetoObstaculoInicial;

    unsigned long tempoRetoObstaculoOrbita;


    // ========================================================
    // CONFIRMAÇÃO DE LINHA NO OBSTÁCULO
    // ========================================================

    int leiturasPretasObstaculo;


    // ========================================================
    // TRAVA APÓS FINALIZAR OBSTÁCULO
    // ========================================================
    //
    // Depois que a linha é encontrada e o robô termina o
    // giro final, o ToF pode continuar detectando o mesmo
    // obstáculo por algum tempo.
    //
    // Essa trava impede que a máquina imediatamente
    // reinicie uma nova manobra de obstáculo.
    //
    // A trava só é liberada quando o ToF deixar de
    // detectar o obstáculo.
    //

    bool bloquearNovoObstaculo;


    // ========================================================
    // MÉTODOS
    // ========================================================

    void entrarEstado(
        Estado novoEstado
    );


    // --------------------------------------------------------
    // Linha
    // --------------------------------------------------------

    void processarSeguindoLinha(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Avaliação
    // --------------------------------------------------------

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
    // Giros normais
    // --------------------------------------------------------

    void iniciarGiro(
        AcaoGiro acao
    );

    void processarGiro(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Ré antes de curvas
    // --------------------------------------------------------

    void reAntesDaCurva();


    // --------------------------------------------------------
    // Obstáculo
    // --------------------------------------------------------

    void iniciarObstaculo();

    void iniciarRetoObstaculo(
        unsigned long duracao
    );

    void processarObstaculo(
        const LinhaData& linha
    );

    bool linhaPretaConfirmada(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Busca normal
    // --------------------------------------------------------

    void iniciarBuscaLinha();

    bool linhaEncontrada(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Sensores de linha
    // --------------------------------------------------------

    bool todosBrancos(
        const LinhaData& linha
    );

    int contarSensoresPretos(
        const LinhaData& linha
    );
};

#endif