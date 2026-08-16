#ifndef ESTADO_ROBO_H
#define ESTADO_ROBO_H

#include <Arduino.h>

#include "../sensores/LinhaAnalise.h"
#include "../sensores/TOFAnalise.h"


// ============================================================
// ESTADOS DO ROBÔ
// ============================================================

enum EstadoRobo
{
    ESTADO_SEGUINDO_LINHA,

    ESTADO_CURVA_ESQUERDA,
    ESTADO_CURVA_DIREITA,

    ESTADO_DESALINHADO,

    ESTADO_OBSTACULO,

    ESTADO_RECUPERACAO
};


// ============================================================
// CLASSE
// ============================================================

class EstadoRoboControl
{

private:

    EstadoRobo estadoAtual;

    // --------------------------------------------------------
    // Memória do último lado conhecido da linha
    // --------------------------------------------------------

    int ultimoLado;

    // -1 = esquerda
    //  0 = centro
    // +1 = direita


    // --------------------------------------------------------
    // Tempo sem detectar linha
    // --------------------------------------------------------

    unsigned long inicioPerdaLinha;


    // --------------------------------------------------------
    // Parâmetros de decisão
    // --------------------------------------------------------

    static constexpr float LIMITE_CURVA =
        4.0f;

    static constexpr float LIMITE_DESALINHAMENTO =
        6.0f;

    static constexpr unsigned long TEMPO_RECUPERACAO =
        300;


    // --------------------------------------------------------
    // Determina o lado da linha
    // --------------------------------------------------------

    int determinarLado(
        const LinhaData& linha
    );


    // --------------------------------------------------------
    // Atualiza memória de lado
    // --------------------------------------------------------

    void atualizarMemoriaLinha(
        const LinhaData& linha
    );


public:

    // ========================================================
    // CONSTRUTOR
    // ========================================================

    EstadoRoboControl();


    // ========================================================
    // ATUALIZA ESTADO
    // ========================================================

    void update(
        const LinhaData& linha,
        const TOFAnalise& tof
    );


    // ========================================================
    // RETORNA ESTADO
    // ========================================================

    EstadoRobo getEstado();


    // ========================================================
    // DEFINE MANUALMENTE
    // ========================================================

    void definirEstado(
        EstadoRobo novoEstado
    );


    // ========================================================
    // TEXTO DO ESTADO
    // ========================================================

    const char* getNomeEstado();

};

#endif