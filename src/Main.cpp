#include <Arduino.h>

#include "hardware/ArrayLinha.h"
#include "hardware/MotorControlador.h"
#include "hardware/TCA.h"
#include "hardware/AS7341.h"
#include "hardware/TOF200F.h"

#include "sensores/LinhaAnalise.h"
#include "sensores/AS7341Analise.h"

#include "controle/PIDLinha.h"
#include "controle/ControleDirecao.h"
#include "controle/CurvaAnalise.h"
#include "controle/Verde.h"

#include "estados/EstadoRobo.h"


// ============================================================
// PINOS
// ============================================================

const int PINO_BOTAO = 32;


// ============================================================
// ARRAY DE LINHA
//
// RX = 5
// TX = 18
// ============================================================

ArrayLinha arrayLinha(
    Serial2,
    5,
    18
);


// ============================================================
// CONTROLADOR DOS MOTORES
//
// RX = 16
// TX = 17
// ============================================================

MotorControlador motores(
    Serial1,
    16,
    17
);


// ============================================================
// TCA9548A
// ============================================================

TCA tca;


// ============================================================
// SENSORES
// ============================================================

AS7341Sensores sensoresCor(
    tca
);


TOF200F tof(
    tca
);


// ============================================================
// ANÁLISE DA LINHA
// ============================================================

LinhaAnalise linha;


// ============================================================
// ANÁLISE DE COR
// ============================================================

AS7341Analise analiseCor;


// ============================================================
// PID
// ============================================================

PIDLinha pid(
    80.0f,
    0.0f,
    20.0f
);


// ============================================================
// CONTROLE DE DIREÇÃO
// ============================================================

ControleDirecao direcao(
    200,
    400
);


// ============================================================
// ANÁLISE DE CURVA / CRUZAMENTO
// ============================================================

CurvaAnalise curva;


// ============================================================
// MÓDULO DE VERDE
// ============================================================

Verde verde(
    motores,
    analiseCor
);


// ============================================================
// ESTADOS PRINCIPAIS
// ============================================================

EstadoRoboControl estado;


// ============================================================
// VELOCIDADE DA CURVA
// ============================================================

const int VELOCIDADE_CURVA =
    400;


// ============================================================
// CANCELAMENTO DE INÉRCIA
// ============================================================

const int VELOCIDADE_FREIO =
    -1000;


const unsigned long TEMPO_FREIO_CURVA =
    20;


// ============================================================
// PRÉ-CURVA
// ============================================================

const int VELOCIDADE_PRE_CURVA =
    60;


const unsigned long TEMPO_PRE_CURVA =
    180;


// ============================================================
// LIMIAR DE PRETO
// ============================================================

const float LIMIAR_PRETO_CURVA =
    0.35f;


// ============================================================
// MÍNIMO DE PRETOS PARA CONFIRMAR CURVA
// ============================================================

const int MINIMO_PRETOS_CURVA =
    4;


// ============================================================
// MÍNIMO DE PRETOS PARA ENCONTRAR NOVA LINHA
// ============================================================

const int MINIMO_PRETOS_NOVA_LINHA =
    3;


// ============================================================
// LIMITE DE SEGURANÇA DA CURVA
// ============================================================

const unsigned long TEMPO_MAXIMO_CURVA =
    700;


// ============================================================
// AVANÇO APÓS ENCONTRAR NOVA LINHA
// ============================================================

const unsigned long TEMPO_ALINHAMENTO_CURVA =
    200;


// ============================================================
// AVANÇO APÓS VERDE QUE NÃO ERA CURVA
// ============================================================
//
// Quando verde é detectado mas existem mais de 3 sensores
// pretos:
//
//      NÃO faz curva
//      NÃO continua indefinidamente
//
// O robô avança por este tempo e depois volta ao PID.
//
// ============================================================

const int VELOCIDADE_POS_VERDE =
    200;


const unsigned long TEMPO_POS_VERDE =
    400;


// ============================================================
// CRUZAMENTO
// ============================================================

const unsigned long TEMPO_CRUZAMENTO =
    300;


const unsigned long BLOQUEIO_CRUZAMENTO =
    500;


// ============================================================
// OBSTÁCULO
// ============================================================

const int DISTANCIA_OBSTACULO_MM =
    100;


// ============================================================
// TEMPOS
// ============================================================

unsigned long tempoAnteriorPID =
    0;


unsigned long inicioFreioCurva =
    0;


unsigned long inicioCruzamento =
    0;


unsigned long fimBloqueioCruzamento =
    0;


// ============================================================
// CONTROLE INTERNO DO AVANÇO PÓS-VERDE
// ============================================================

bool avancandoPosVerde =
    false;


unsigned long inicioAvancoPosVerde =
    0;


// ============================================================
// CONTROLE INTERNO DA CURVA
// ============================================================

enum FaseCurva
{
    CURVA_FASE_AVANCANDO,

    CURVA_FASE_FREANDO,

    CURVA_FASE_GIRANDO,

    CURVA_FASE_ALINHANDO
};


FaseCurva faseCurva =
    CURVA_FASE_AVANCANDO;


// ============================================================
// DIREÇÃO DA CURVA
//
// TRUE  = esquerda
// FALSE = direita
// ============================================================

bool curvaAtualEsquerda =
    false;


// ============================================================
// TEMPOS INTERNOS DA CURVA
// ============================================================

unsigned long inicioPreCurva =
    0;


unsigned long inicioGiroCurva =
    0;


unsigned long inicioAlinhamentoCurva =
    0;


// ============================================================
// BOTÃO
// ============================================================

bool botaoAnterior =
    HIGH;


// ============================================================
// GIRO PARA ESQUERDA
// ============================================================

void girarEsquerda()
{
    motores.setSpeed(
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA
    );
}


// ============================================================
// GIRO PARA DIREITA
// ============================================================

void girarDireita()
{
    motores.setSpeed(
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// CANCELA INÉRCIA
// ============================================================

void cancelarInercia()
{
    motores.setSpeed(
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO
    );
}


// ============================================================
// PARA ROBÔ
// ============================================================

void pararRobo()
{
    motores.stop();
}


// ============================================================
// ANDA DEVAGAR DURANTE PRÉ-CURVA
// ============================================================

void avancarPreCurva()
{
    motores.setSpeed(
        VELOCIDADE_PRE_CURVA,
        VELOCIDADE_PRE_CURVA,
        VELOCIDADE_PRE_CURVA,
        VELOCIDADE_PRE_CURVA
    );
}


// ============================================================
// AVANÇO APÓS VERDE FALSO
// ============================================================

void avancarPosVerde()
{
    motores.setSpeed(
        VELOCIDADE_POS_VERDE,
        VELOCIDADE_POS_VERDE,
        VELOCIDADE_POS_VERDE,
        VELOCIDADE_POS_VERDE
    );
}


// ============================================================
// AVANÇA RETO
// ============================================================

void avancarReto()
{
    direcao.update(
        0.0f
    );


    MotoresData dadosMotores =
        direcao.getData();


    motores.setSpeed(
        dadosMotores.m1,
        dadosMotores.m2,
        dadosMotores.m3,
        dadosMotores.m4
    );
}


// ============================================================
// CONTA QUANTOS SENSORES ESTÃO PRETOS
// ============================================================

int quantidadeSensoresPretos(
    const LinhaData& dadosLinha
)
{
    int quantidade =
        0;


    for(int i = 0; i < 8; i++)
    {
        if(
            dadosLinha.sensores[i] >=
            LIMIAR_PRETO_CURVA
        )
        {
            quantidade++;
        }
    }


    return quantidade;
}


// ============================================================
// VERIFICA SE TODOS OS SENSORES ESTÃO BRANCOS
// ============================================================

bool arrayTodoBranco(
    const LinhaData& dadosLinha
)
{
    return (
        quantidadeSensoresPretos(
            dadosLinha
        ) == 0
    );
}


// ============================================================
// VERIFICA SE EXISTE PRETO NA REGIÃO CENTRAL
//
// S3 S4 S5 S6
// ============================================================

bool pretoNaRegiaoCentral(
    const LinhaData& dadosLinha
)
{
    for(int i = 2; i <= 5; i++)
    {
        if(
            dadosLinha.sensores[i] >=
            LIMIAR_PRETO_CURVA
        )
        {
            return true;
        }
    }


    return false;
}


// ============================================================
// VERIFICA CURVA FALSA
//
// Menos de 4 pretos.
//
// E todos os pretos estão somente em:
//
// S3 S4 S5 S6
//
// ============================================================

bool curvaFalsa(
    const LinhaData& dadosLinha
)
{
    int quantidadePretos =
        quantidadeSensoresPretos(
            dadosLinha
        );


    if(
        quantidadePretos >=
        MINIMO_PRETOS_CURVA
    )
    {
        return false;
    }


    if(
        !pretoNaRegiaoCentral(
            dadosLinha
        )
    )
    {
        return false;
    }


    for(int i = 0; i < 8; i++)
    {
        bool preto =
            dadosLinha.sensores[i] >=
            LIMIAR_PRETO_CURVA;


        if(
            preto &&
            (i < 2 || i > 5)
        )
        {
            return false;
        }
    }


    return true;
}


// ============================================================
// PROCURA NOVA LINHA À ESQUERDA
//
// S1 S2 S3 S4
// ============================================================

bool encontrouNovaLinhaEsquerda(
    const LinhaData& dadosLinha
)
{
    int quantidadePretos =
        0;


    for(int i = 0; i < 4; i++)
    {
        if(
            dadosLinha.sensores[i] >=
            LIMIAR_PRETO_CURVA
        )
        {
            quantidadePretos++;
        }
    }


    return (
        quantidadePretos >=
        MINIMO_PRETOS_NOVA_LINHA
    );
}


// ============================================================
// PROCURA NOVA LINHA À DIREITA
//
// S5 S6 S7 S8
// ============================================================

bool encontrouNovaLinhaDireita(
    const LinhaData& dadosLinha
)
{
    int quantidadePretos =
        0;


    for(int i = 4; i < 8; i++)
    {
        if(
            dadosLinha.sensores[i] >=
            LIMIAR_PRETO_CURVA
        )
        {
            quantidadePretos++;
        }
    }


    return (
        quantidadePretos >=
        MINIMO_PRETOS_NOVA_LINHA
    );
}


// ============================================================
// RESETA CONTROLE INTERNO DA CURVA
// ============================================================

void resetControleCurva()
{
    faseCurva =
        CURVA_FASE_AVANCANDO;


    curvaAtualEsquerda =
        false;


    inicioPreCurva =
        0;


    inicioFreioCurva =
        0;


    inicioGiroCurva =
        0;


    inicioAlinhamentoCurva =
        0;
}


// ============================================================
// REINICIA ROBÔ
// ============================================================

void reiniciarRobo()
{
    pararRobo();


    estado.definirEstado(
        ESTADO_SEGUINDO_LINHA
    );


    pid.reset();


    verde.reset();


    direcao.update(
        0.0f
    );


    inicioFreioCurva =
        0;


    inicioCruzamento =
        0;


    fimBloqueioCruzamento =
        0;


    avancandoPosVerde =
        false;


    inicioAvancoPosVerde =
        0;


    resetControleCurva();


    tempoAnteriorPID =
        micros();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );


    delay(500);


    // ========================================================
    // BOTÃO
    // ========================================================

    pinMode(
        PINO_BOTAO,
        INPUT
    );


    // ========================================================
    // TCA
    // ========================================================

    tca.begin(
        -1,
        -1,
        400000
    );


    // ========================================================
    // ARRAY
    // ========================================================

    arrayLinha.begin();


    // ========================================================
    // MOTORES
    // ========================================================

    motores.begin();


    // ========================================================
    // TOF
    // ========================================================

    tof.begin(
        0
    );


    // ========================================================
    // AS7341
    // ========================================================

    sensoresCor.begin();


    // ========================================================
    // SEGURANÇA
    // ========================================================

    pararRobo();


    // ========================================================
    // ESTADO INICIAL
    // ========================================================

    estado.definirEstado(
        ESTADO_SEGUINDO_LINHA
    );


    tempoAnteriorPID =
        micros();


    botaoAnterior =
        digitalRead(
            PINO_BOTAO
        );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // BOTÃO
    // ========================================================

    bool botaoAtual =
        digitalRead(
            PINO_BOTAO
        );


    // --------------------------------------------------------
    // LOW = PARADO
    // --------------------------------------------------------

    if(
        botaoAtual == LOW
    )
    {
        pararRobo();


        botaoAnterior =
            LOW;


        return;
    }


    // --------------------------------------------------------
    // LOW -> HIGH
    // --------------------------------------------------------

    if(
        botaoAnterior == LOW &&
        botaoAtual == HIGH
    )
    {
        reiniciarRobo();
    }


    botaoAnterior =
        botaoAtual;


    // ========================================================
    // AQUISIÇÃO DOS SENSORES I2C
    // ========================================================

    tof.update();

    sensoresCor.update();


    // ========================================================
    // OBSTÁCULO
    // ========================================================

    int distancia =
        tof.getDistance();


    if(
        distancia > 0 &&
        distancia <= DISTANCIA_OBSTACULO_MM
    )
    {
        pararRobo();

        return;
    }


    // ========================================================
    // ARRAY DE LINHA
    // ========================================================

    arrayLinha.update();


    ArrayData dadosArray =
        arrayLinha.getData();


    // ========================================================
    // ANÁLISE DA LINHA
    // ========================================================

    linha.update(
        dadosArray
    );


    LinhaData dadosLinha =
        linha.getData();


    // ========================================================
    // QUANTIDADE DE PRETOS
    // ========================================================

    int quantidadePretos =
        quantidadeSensoresPretos(
            dadosLinha
        );


    // ========================================================
    // ANÁLISE DE CURVA / CRUZAMENTO
    // ========================================================

    curva.update(
        dadosLinha
    );


    CurvaData dadosCurva =
        curva.getData();


    // ========================================================
    // ESTADO ATUAL
    // ========================================================

    EstadoRobo estadoAtual =
        estado.getEstado();


    // ========================================================
    // AS7341
    // ========================================================

    AS7341Data dadosEsquerda =
        sensoresCor.getEsquerda();


    AS7341Data dadosDireita =
        sensoresCor.getDireita();


    // ========================================================
    // VERDE
    // ========================================================
    //
    // IMPORTANTE:
    //
    // O Verde precisa ser atualizado SEMPRE quando o robô
    // estiver seguindo linha.
    //
    // NÃO podemos bloquear o update quando existem mais
    // de 3 pretos.
    //
    // O terceiro argumento informa a quantidade de pretos
    // para o módulo Verde decidir:
    //
    // <= 3 pretos:
    //     verde pode resultar em manobra.
    //
    // > 3 pretos:
    //     verde não deve gerar curva.
    //
    // Neste segundo caso, fazemos um pequeno avanço e
    // depois devolvemos o controle ao PID.
    //
    // ========================================================

    bool verdeEstavaExecutando =
        verde.estaExecutando();


    bool verdeTerminou =
        verde.finalizado();


    if(
        estadoAtual ==
        ESTADO_SEGUINDO_LINHA
        ||
        verdeEstavaExecutando
        ||
        verdeTerminou
    )
    {
        verde.update(
            dadosEsquerda,
            dadosDireita,
            quantidadePretos
        );


        // ----------------------------------------------------
        // VERDE EXECUTANDO
        // ----------------------------------------------------

        if(
            verde.estaExecutando()
        )
        {
            return;
        }


        // ----------------------------------------------------
        // VERDE TERMINOU
        // ----------------------------------------------------

        if(
            verde.finalizado()
        )
        {
            // ------------------------------------------------
            // Se o Verde executou uma manobra real,
            // o comportamento continua sendo o anterior.
            // ------------------------------------------------

            verde.reset();


            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            pid.reset();


            fimBloqueioCruzamento =
                millis() +
                BLOQUEIO_CRUZAMENTO;


            resetControleCurva();


            tempoAnteriorPID =
                micros();


            return;
        }
    }


    // ========================================================
    // ATUALIZA ESTADO
    // ========================================================

    estadoAtual =
        estado.getEstado();


    // ========================================================
    // AVANÇO PÓS-VERDE
    // ========================================================
    //
    // Este bloco é usado quando uma detecção de verde NÃO
    // resultou em uma curva.
    //
    // O robô:
    //
    // 1. avança um pequeno trecho;
    // 2. não usa PID durante esse pequeno trecho;
    // 3. depois volta ao PID.
    //
    // ========================================================

    if(
        avancandoPosVerde
    )
    {
        avancarPosVerde();


        if(
            millis() -
            inicioAvancoPosVerde >=
            TEMPO_POS_VERDE
        )
        {
            avancandoPosVerde =
                false;


            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            pid.reset();


            tempoAnteriorPID =
                micros();


            direcao.update(
                0.0f
            );
        }


        return;
    }


    // ========================================================
    // SEGUINDO LINHA
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_SEGUINDO_LINHA
    )
    {
        bool cruzamentoLiberado =
            millis() >=
            fimBloqueioCruzamento;


        // ====================================================
        // CRUZAMENTO
        // ====================================================

        if(
            dadosCurva.cruzamento &&
            cruzamentoLiberado
        )
        {
            estado.definirEstado(
                ESTADO_CRUZAMENTO
            );


            inicioCruzamento =
                millis();


            pid.reset();


            return;
        }


        // ====================================================
        // POSSÍVEL CURVA 90°
        // ====================================================
        //
        // Só entra se houver pelo menos 4 sensores pretos.
        //
        // 3 ou menos:
        //     não entra na rotina de curva.
        //
        // 4 ou mais:
        //     pode entrar na pré-curva.
        //
        // ====================================================

        if(
            dadosCurva.curva90 &&
            quantidadePretos >=
            MINIMO_PRETOS_CURVA
        )
        {
            // ------------------------------------------------
            // ESQUERDA
            // ------------------------------------------------

            if(
                dadosCurva.direcao ==
                CURVA_ESQUERDA
            )
            {
                estado.definirEstado(
                    ESTADO_CURVA_ESQUERDA
                );


                curvaAtualEsquerda =
                    true;


                faseCurva =
                    CURVA_FASE_AVANCANDO;


                inicioPreCurva =
                    millis();


                inicioFreioCurva =
                    0;


                inicioGiroCurva =
                    0;


                inicioAlinhamentoCurva =
                    0;


                pid.reset();


                avancarPreCurva();


                return;
            }


            // ------------------------------------------------
            // DIREITA
            // ------------------------------------------------

            if(
                dadosCurva.direcao ==
                CURVA_DIREITA
            )
            {
                estado.definirEstado(
                    ESTADO_CURVA_DIREITA
                );


                curvaAtualEsquerda =
                    false;


                faseCurva =
                    CURVA_FASE_AVANCANDO;


                inicioPreCurva =
                    millis();


                inicioFreioCurva =
                    0;


                inicioGiroCurva =
                    0;


                inicioAlinhamentoCurva =
                    0;


                pid.reset();


                avancarPreCurva();


                return;
            }
        }
    }


    // ========================================================
    // ATUALIZA ESTADO NOVAMENTE
    // ========================================================

    estadoAtual =
        estado.getEstado();


    // ========================================================
    // CRUZAMENTO
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_CRUZAMENTO
    )
    {
        unsigned long tempo =
            millis() -
            inicioCruzamento;


        avancarReto();


        if(
            tempo >=
            TEMPO_CRUZAMENTO
        )
        {
            estado.definirEstado(
                ESTADO_SEGUINDO_LINHA
            );


            fimBloqueioCruzamento =
                millis() +
                BLOQUEIO_CRUZAMENTO;


            pid.reset();


            tempoAnteriorPID =
                micros();
        }


        return;
    }


    // ========================================================
    // CURVAS 90°
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_CURVA_ESQUERDA
        ||
        estadoAtual ==
        ESTADO_CURVA_DIREITA
    )
    {
        // ====================================================
        // FASE 1
        //
        // AVANÇA DEVAGAR PARA CONFIRMAR
        // ====================================================

        if(
            faseCurva ==
            CURVA_FASE_AVANCANDO
        )
        {
            avancarPreCurva();


            unsigned long tempoPreCurva =
                millis() -
                inicioPreCurva;


            // ------------------------------------------------
            // CURVA FALSA
            //
            // Menos de 4 pretos.
            //
            // E os pretos estão somente entre S3-S6.
            //
            // Volta imediatamente para o PID.
            // ------------------------------------------------

            if(
                curvaFalsa(
                    dadosLinha
                )
            )
            {
                estado.definirEstado(
                    ESTADO_SEGUINDO_LINHA
                );


                resetControleCurva();


                pid.reset();


                tempoAnteriorPID =
                    micros();


                return;
            }


            // ------------------------------------------------
            // TODOS BRANCOS
            //
            // A linha desapareceu completamente.
            //
            // Curva real confirmada.
            // ------------------------------------------------

            if(
                arrayTodoBranco(
                    dadosLinha
                )
            )
            {
                faseCurva =
                    CURVA_FASE_FREANDO;


                inicioFreioCurva =
                    millis();


                cancelarInercia();


                return;
            }


            // ------------------------------------------------
            // SEGURANÇA DA PRÉ-CURVA
            // ------------------------------------------------

            if(
                tempoPreCurva >=
                TEMPO_PRE_CURVA
            )
            {
                estado.definirEstado(
                    ESTADO_SEGUINDO_LINHA
                );


                resetControleCurva();


                pid.reset();


                tempoAnteriorPID =
                    micros();


                return;
            }


            return;
        }


        // ====================================================
        // FASE 2
        //
        // CANCELAMENTO DE INÉRCIA
        // ====================================================

        if(
            faseCurva ==
            CURVA_FASE_FREANDO
        )
        {
            cancelarInercia();


            if(
                millis() -
                inicioFreioCurva >=
                TEMPO_FREIO_CURVA
            )
            {
                faseCurva =
                    CURVA_FASE_GIRANDO;


                inicioGiroCurva =
                    millis();
            }


            return;
        }


        // ====================================================
        // FASE 3
        //
        // GIRO NO PRÓPRIO EIXO
        // ====================================================

        if(
            faseCurva ==
            CURVA_FASE_GIRANDO
        )
        {
            unsigned long tempoGiro =
                millis() -
                inicioGiroCurva;


            // ------------------------------------------------
            // LIMITE DE SEGURANÇA
            // ------------------------------------------------

            if(
                tempoGiro >=
                TEMPO_MAXIMO_CURVA
            )
            {
                pararRobo();


                estado.definirEstado(
                    ESTADO_SEGUINDO_LINHA
                );


                resetControleCurva();


                pid.reset();


                tempoAnteriorPID =
                    micros();


                return;
            }


            // ------------------------------------------------
            // CURVA ESQUERDA
            // ------------------------------------------------

            if(
                curvaAtualEsquerda
            )
            {
                girarEsquerda();


                if(
                    encontrouNovaLinhaEsquerda(
                        dadosLinha
                    )
                )
                {
                    faseCurva =
                        CURVA_FASE_ALINHANDO;


                    inicioAlinhamentoCurva =
                        millis();


                    return;
                }


                return;
            }


            // ------------------------------------------------
            // CURVA DIREITA
            // ------------------------------------------------

            girarDireita();


            if(
                encontrouNovaLinhaDireita(
                    dadosLinha
                )
            )
            {
                faseCurva =
                    CURVA_FASE_ALINHANDO;


                inicioAlinhamentoCurva =
                    millis();


                return;
            }


            return;
        }


        // ====================================================
        // FASE 4
        //
        // PEQUENO AVANÇO APÓS ENCONTRAR NOVA LINHA
        // ====================================================

        if(
            faseCurva ==
            CURVA_FASE_ALINHANDO
        )
        {
            avancarReto();


            if(
                millis() -
                inicioAlinhamentoCurva >=
                TEMPO_ALINHAMENTO_CURVA
            )
            {
                estado.definirEstado(
                    ESTADO_SEGUINDO_LINHA
                );


                resetControleCurva();


                pid.reset();


                tempoAnteriorPID =
                    micros();
            }


            return;
        }
    }


    // ========================================================
    // ATUALIZA ESTADO NOVAMENTE
    // ========================================================

    estadoAtual =
        estado.getEstado();


    // ========================================================
    // SEGUINDO LINHA COM PID
    // ========================================================

    if(
        estadoAtual ==
        ESTADO_SEGUINDO_LINHA
    )
    {
        unsigned long agora =
            micros();


        float deltaTime =
            (
                agora -
                tempoAnteriorPID
            )
            / 1000000.0f;


        tempoAnteriorPID =
            agora;


        // ----------------------------------------------------
        // LINHA DETECTADA
        // ----------------------------------------------------

        if(
            dadosLinha.linhaDetectada
        )
        {
            pid.update(
                dadosLinha.erro,
                deltaTime
            );


            PIDData dadosPID =
                pid.getData();


            direcao.update(
                dadosPID.correcao
            );
        }


        // ----------------------------------------------------
        // GAP
        // ----------------------------------------------------

        else
        {
            direcao.update(
                0.0f
            );
        }


        // ----------------------------------------------------
        // MOTORES
        // ----------------------------------------------------

        MotoresData dadosMotores =
            direcao.getData();


        motores.setSpeed(
            dadosMotores.m1,
            dadosMotores.m2,
            dadosMotores.m3,
            dadosMotores.m4
        );
    }


    // ========================================================
    // PEQUENO INTERVALO
    // ========================================================

    delay(5);
}