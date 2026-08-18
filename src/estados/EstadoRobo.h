#ifndef ESTADO_ROBO_H
#define ESTADO_ROBO_H

#include <Arduino.h>

#include "../sensores/LinhaAnalise.h"
#include "../sensores/TOFAnalise.h"
#include "../decisao/DecisaoRobo.h"

// ============================================================
// ESTADOS DO ROBÔ
// ============================================================

enum EstadoRobo
{
    ESTADO_SEGUINDO_LINHA,

    ESTADO_CURVA_ESQUERDA,

    ESTADO_CURVA_DIREITA,

    ESTADO_INTERSECCAO,

    ESTADO_PASSAGEM_RETA,

    ESTADO_MEIA_VOLTA,

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
    // Memória do último lado conhecido
    // --------------------------------------------------------

    int ultimoLado;

    // --------------------------------------------------------
    // Tempo sem detectar linha
    // --------------------------------------------------------

    unsigned long inicioPerdaLinha;

    // --------------------------------------------------------
    // Parâmetros antigos mantidos por compatibilidade
    // --------------------------------------------------------

    static constexpr float LIMITE_CURVA =
        4.0f;

    static constexpr float LIMITE_DESALINHAMENTO =
        6.0f;

    static constexpr unsigned long TEMPO_RECUPERACAO =
        300;

    // --------------------------------------------------------
    // Determina lado da linha
    // --------------------------------------------------------

    int determinarLado(
        const LinhaData& linha
    );

    // --------------------------------------------------------
    // Atualiza memória da linha
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
    // UPDATE PRINCIPAL
    //
    // Recebe a decisão já processada.
    // ========================================================

    void update(
        const DecisaoData& decisao
    );

    // ========================================================
    // UPDATE ANTIGO COM TOF
    //
    // Mantido para compatibilidade.
    // ========================================================

    void update(
        const LinhaData& linha,
        const TOFAnalise& tof
    );

    // ========================================================
    // GET
    // ========================================================

    EstadoRobo getEstado();

    // ========================================================
    // DEFINE MANUALMENTE
    // ========================================================

    void definirEstado(
        EstadoRobo novoEstado
    );

    // ========================================================
    // NOME DO ESTADO
    // ========================================================

    const char* getNomeEstado();
};

#endif