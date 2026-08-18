#include <Arduino.h>

#include "hardware/TCA.h"
#include "hardware/AS7341.h"
#include "hardware/ArrayLinha.h"
#include "hardware/TOF200F.h"
#include "hardware/MotorControlador.h"

#include "sensores/AS7341Analise.h"
#include "sensores/LinhaAnalise.h"
#include "sensores/TOFAnalise.h"

#include "controle/ControleSegueLinha.h"

#include "decisao/DecisaoRobo.h"
#include "estados/EstadoRobo.h"


// ============================================================
// PINOS
// ============================================================

const int PINO_BOTAO = 32;


// ============================================================
// VELOCIDADES
// ============================================================

const int VELOCIDADE_RETA = 600;
const int VELOCIDADE_CURVA = 520;
const int VELOCIDADE_GIRO = 550;
const int VELOCIDADE_ALINHAMENTO = 350;


// ============================================================
// TEMPOS DAS MANOBRAS
// ============================================================
//
// São parâmetros centralizados para facilitar a calibração
// posteriormente, sem alterar a lógica.
//

const unsigned long TEMPO_PASSAGEM_RETA = 300;

const unsigned long TEMPO_CORRECAO_CURVA = 100;

const unsigned long TEMPO_MAX_CURVA = 1400;

const unsigned long TEMPO_MEIA_VOLTA = 950;

const unsigned long TEMPO_MAX_ALINHAMENTO = 700;

const unsigned long TEMPO_ALINHAMENTO_FINAL = 100;

const unsigned long TEMPO_INTERSECCAO = 350;


// ============================================================
// OBSTÁCULO
// ============================================================

const unsigned long TEMPO_GIRO_90 = 550;

const unsigned long TEMPO_AVANCO_OBSTACULO = 650;

const unsigned long TEMPO_AVANCO_RETORNO = 700;

const unsigned long TEMPO_BUSCA_LINHA = 1000;


// ============================================================
// OBJETOS
// ============================================================

TCA tca;

AS7341Sensores sensoresAS7341(tca);

TOF200F tof200f(tca);

TOFAnalise tofAnalise(tof200f);


// ============================================================
// ARRAY
// RX = 5
// TX = 18
// ============================================================

ArrayLinha arrayLinha(
    Serial2,
    5,
    18,
    115200
);


// ============================================================
// MOTORES
// RX = 16
// TX = 17
// ============================================================

MotorControlador motorControlador(
    Serial1,
    16,
    17,
    115200
);


// ============================================================
// ANÁLISES
// ============================================================

AS7341Analise as7341Analise;

LinhaAnalise linhaAnalise;


// ============================================================
// CONTROLE
// ============================================================

ControleSegueLinha controleLinha;


// ============================================================
// DECISÃO
// ============================================================

DecisaoRobo decisaoRobo;


// ============================================================
// ESTADOS
// ============================================================

EstadoRoboControl estadoRobo;


// ============================================================
// TIPOS DE MANOBRA
// ============================================================

enum TipoManobra
{
    MANOBRA_NENHUMA,

    MANOBRA_CURVA_ESQUERDA,

    MANOBRA_CURVA_DIREITA,

    MANOBRA_INTERSECCAO,

    MANOBRA_PASSAGEM_RETA,

    MANOBRA_MEIA_VOLTA,

    MANOBRA_OBSTACULO
};


// ============================================================
// FASE DO OBSTÁCULO
// ============================================================

enum FaseObstaculo
{
    OBSTACULO_GIRO_DIREITA,

    OBSTACULO_AVANCA_1,

    OBSTACULO_GIRO_ESQUERDA_1,

    OBSTACULO_AVANCA_2,

    OBSTACULO_GIRO_ESQUERDA_2,

    OBSTACULO_AVANCA_3,

    OBSTACULO_GIRO_DIREITA_2,

    OBSTACULO_BUSCA_LINHA,

    OBSTACULO_ALINHAMENTO
};


// ============================================================
// CONTROLE DA MANOBRA
// ============================================================

TipoManobra manobraAtual =
    MANOBRA_NENHUMA;

FaseObstaculo faseObstaculo =
    OBSTACULO_GIRO_DIREITA;

unsigned long inicioManobra = 0;

unsigned long inicioFase = 0;

bool alinhamentoFinal = false;


// ============================================================
// NOME DA AÇÃO VERDE
// ============================================================

const char* nomeAcaoVerde(
    TipoAcaoVerde acao
)
{
    switch(acao)
    {
        case VERDE_SEM_EVENTO:
            return "SEM EVENTO";

        case VERDE_PASSAGEM_RETA:
            return "PASSAGEM RETA";

        case VERDE_CURVA_ESQUERDA:
            return "CURVA ESQUERDA";

        case VERDE_CURVA_DIREITA:
            return "CURVA DIREITA";

        case VERDE_MEIA_VOLTA:
            return "MEIA VOLTA";

        default:
            return "DESCONHECIDA";
    }
}


// ============================================================
// NOME EVENTO ARRAY
// ============================================================

const char* nomeEventoArray(
    TipoEventoArray evento
)
{
    switch(evento)
    {
        case ARRAY_NORMAL:
            return "NORMAL";

        case ARRAY_CURVA_ESQUERDA:
            return "CURVA ESQUERDA";

        case ARRAY_CURVA_DIREITA:
            return "CURVA DIREITA";

        case ARRAY_INTERSECCAO:
            return "INTERSECCAO";

        default:
            return "DESCONHECIDO";
    }
}


// ============================================================
// NOME EVENTO TOF
// ============================================================

const char* nomeEventoToF(
    TipoEventoToF evento
)
{
    switch(evento)
    {
        case TOF_NORMAL:
            return "NORMAL";

        case TOF_OBSTACULO:
            return "OBSTACULO";

        default:
            return "DESCONHECIDO";
    }
}


// ============================================================
// INICIA MANOBRA
// ============================================================

void iniciarManobra(
    TipoManobra novaManobra
)
{
    manobraAtual =
        novaManobra;

    inicioManobra =
        millis();

    inicioFase =
        millis();

    alinhamentoFinal =
        false;

    controleLinha.reset();
}


// ============================================================
// FINALIZA MANOBRA
// ============================================================

void finalizarManobra()
{
    motorControlador.stop();

    controleLinha.reset();

    manobraAtual =
        MANOBRA_NENHUMA;

    alinhamentoFinal =
        false;
}


// ============================================================
// LINHA CENTRALIZADA
// ============================================================

bool linhaCentralizada(
    const LinhaData& linha
)
{
    if(!linha.linhaDetectada)
    {
        return false;
    }

    return fabsf(linha.posicao) <= 1.5f;
}


// ============================================================
// ALINHAMENTO
// ============================================================

bool executarAlinhamento(
    const LinhaData& linha
)
{
    // --------------------------------------------------------
    // Se já está centralizado, faz pequena correção reta
    // antes de devolver ao PID.
    // --------------------------------------------------------

    if(linhaCentralizada(linha))
    {
        if(!alinhamentoFinal)
        {
            alinhamentoFinal =
                true;

            inicioFase =
                millis();
        }

        motorControlador.setSpeed(
            VELOCIDADE_ALINHAMENTO,
            VELOCIDADE_ALINHAMENTO,
            VELOCIDADE_ALINHAMENTO,
            VELOCIDADE_ALINHAMENTO
        );

        if(
            millis() - inicioFase >=
            TEMPO_ALINHAMENTO_FINAL
        )
        {
            finalizarManobra();

            return true;
        }

        return false;
    }

    alinhamentoFinal =
        false;

    inicioFase =
        millis();

    // --------------------------------------------------------
    // Linha à esquerda
    // --------------------------------------------------------

    if(linha.posicao < 0.0f)
    {
        motorControlador.setSpeed(
            -VELOCIDADE_ALINHAMENTO,
            -VELOCIDADE_ALINHAMENTO,
            VELOCIDADE_ALINHAMENTO,
            VELOCIDADE_ALINHAMENTO
        );
    }

    // --------------------------------------------------------
    // Linha à direita
    // --------------------------------------------------------

    else
    {
        motorControlador.setSpeed(
            VELOCIDADE_ALINHAMENTO,
            VELOCIDADE_ALINHAMENTO,
            -VELOCIDADE_ALINHAMENTO,
            -VELOCIDADE_ALINHAMENTO
        );
    }

    // --------------------------------------------------------
    // Segurança contra ficar preso
    // --------------------------------------------------------

    if(
        millis() - inicioManobra >=
        TEMPO_MAX_ALINHAMENTO
    )
    {
        finalizarManobra();

        return true;
    }

    return false;
}


// ============================================================
// CURVA
// ============================================================

void executarCurva(
    const LinhaData& linha,
    bool esquerda
)
{
    unsigned long tempo =
        millis() - inicioManobra;


    // --------------------------------------------------------
    // Pequena correção inicial
    // --------------------------------------------------------

    if(
        tempo <
        TEMPO_CORRECAO_CURVA
    )
    {
        if(esquerda)
        {
            motorControlador.setSpeed(
                250,
                250,
                500,
                500
            );
        }
        else
        {
            motorControlador.setSpeed(
                500,
                500,
                250,
                250
            );
        }

        return;
    }


    // --------------------------------------------------------
    // Giro
    // --------------------------------------------------------

    if(esquerda)
    {
        motorControlador.setSpeed(
            -VELOCIDADE_CURVA,
            -VELOCIDADE_CURVA,
            VELOCIDADE_CURVA,
            VELOCIDADE_CURVA
        );
    }
    else
    {
        motorControlador.setSpeed(
            VELOCIDADE_CURVA,
            VELOCIDADE_CURVA,
            -VELOCIDADE_CURVA,
            -VELOCIDADE_CURVA
        );
    }


    // --------------------------------------------------------
    // Quando encontra novamente a linha, alinha
    // --------------------------------------------------------

    if(
        tempo >
        TEMPO_CORRECAO_CURVA
    )
    {
        if(linhaCentralizada(linha))
        {
            executarAlinhamento(linha);

            return;
        }
    }


    // --------------------------------------------------------
    // Timeout
    // --------------------------------------------------------

    if(
        tempo >=
        TEMPO_MAX_CURVA
    )
    {
        executarAlinhamento(linha);
    }
}


// ============================================================
// INTERSECÇÃO
// ============================================================

void executarInterseccao()
{
    unsigned long tempo =
        millis() - inicioManobra;


    motorControlador.setSpeed(
        VELOCIDADE_RETA,
        VELOCIDADE_RETA,
        VELOCIDADE_RETA,
        VELOCIDADE_RETA
    );


    if(
        tempo >=
        TEMPO_INTERSECCAO
    )
    {
        finalizarManobra();
    }
}


// ============================================================
// PASSAGEM RETA
// ============================================================

void executarPassagemReta()
{
    unsigned long tempo =
        millis() - inicioManobra;


    motorControlador.setSpeed(
        VELOCIDADE_RETA,
        VELOCIDADE_RETA,
        VELOCIDADE_RETA,
        VELOCIDADE_RETA
    );


    if(
        tempo >=
        TEMPO_PASSAGEM_RETA
    )
    {
        finalizarManobra();
    }
}


// ============================================================
// MEIA VOLTA
// ============================================================

void executarMeiaVolta(
    const LinhaData& linha
)
{
    unsigned long tempo =
        millis() - inicioManobra;


    // --------------------------------------------------------
    // Giro
    // --------------------------------------------------------

    motorControlador.setSpeed(
        VELOCIDADE_GIRO,
        VELOCIDADE_GIRO,
        -VELOCIDADE_GIRO,
        -VELOCIDADE_GIRO
    );


    // --------------------------------------------------------
    // Depois do giro, procura a linha
    // --------------------------------------------------------

    if(
        tempo >=
        TEMPO_MEIA_VOLTA
    )
    {
        executarAlinhamento(linha);
    }
}


// ============================================================
// OBSTÁCULO
// ============================================================

void executarObstaculo(
    const LinhaData& linha
)
{
    unsigned long tempo =
        millis() - inicioFase;


    switch(faseObstaculo)
    {

        // ====================================================
        // 1. GIRA 90° DIREITA
        // ====================================================

        case OBSTACULO_GIRO_DIREITA:

            motorControlador.setSpeed(
                VELOCIDADE_GIRO,
                VELOCIDADE_GIRO,
                -VELOCIDADE_GIRO,
                -VELOCIDADE_GIRO
            );

            if(
                tempo >=
                TEMPO_GIRO_90
            )
            {
                faseObstaculo =
                    OBSTACULO_AVANCA_1;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 2. AVANÇA
        // ====================================================

        case OBSTACULO_AVANCA_1:

            motorControlador.setSpeed(
                VELOCIDADE_RETA,
                VELOCIDADE_RETA,
                VELOCIDADE_RETA,
                VELOCIDADE_RETA
            );

            if(
                tempo >=
                TEMPO_AVANCO_OBSTACULO
            )
            {
                faseObstaculo =
                    OBSTACULO_GIRO_ESQUERDA_1;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 3. GIRA ESQUERDA
        // ====================================================

        case OBSTACULO_GIRO_ESQUERDA_1:

            motorControlador.setSpeed(
                -VELOCIDADE_GIRO,
                -VELOCIDADE_GIRO,
                VELOCIDADE_GIRO,
                VELOCIDADE_GIRO
            );

            if(
                tempo >=
                TEMPO_GIRO_90
            )
            {
                faseObstaculo =
                    OBSTACULO_AVANCA_2;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 4. PASSA AO LADO DO OBSTÁCULO
        // ====================================================

        case OBSTACULO_AVANCA_2:

            motorControlador.setSpeed(
                VELOCIDADE_RETA,
                VELOCIDADE_RETA,
                VELOCIDADE_RETA,
                VELOCIDADE_RETA
            );

            if(
                tempo >=
                TEMPO_AVANCO_RETORNO
            )
            {
                faseObstaculo =
                    OBSTACULO_GIRO_ESQUERDA_2;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 5. GIRA NOVAMENTE ESQUERDA
        // ====================================================

        case OBSTACULO_GIRO_ESQUERDA_2:

            motorControlador.setSpeed(
                -VELOCIDADE_GIRO,
                -VELOCIDADE_GIRO,
                VELOCIDADE_GIRO,
                VELOCIDADE_GIRO
            );

            if(
                tempo >=
                TEMPO_GIRO_90
            )
            {
                faseObstaculo =
                    OBSTACULO_AVANCA_3;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 6. AVANÇA PARA LINHA
        // ====================================================

        case OBSTACULO_AVANCA_3:

            motorControlador.setSpeed(
                VELOCIDADE_RETA,
                VELOCIDADE_RETA,
                VELOCIDADE_RETA,
                VELOCIDADE_RETA
            );

            if(
                linha.linhaDetectada &&
                linhaCentralizada(linha)
            )
            {
                faseObstaculo =
                    OBSTACULO_GIRO_DIREITA_2;

                inicioFase =
                    millis();

                break;
            }

            if(
                tempo >=
                TEMPO_AVANCO_RETORNO
            )
            {
                faseObstaculo =
                    OBSTACULO_GIRO_DIREITA_2;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 7. VOLTA PARA ORIENTAÇÃO ORIGINAL
        // ====================================================

        case OBSTACULO_GIRO_DIREITA_2:

            motorControlador.setSpeed(
                VELOCIDADE_GIRO,
                VELOCIDADE_GIRO,
                -VELOCIDADE_GIRO,
                -VELOCIDADE_GIRO
            );

            if(
                tempo >=
                TEMPO_GIRO_90
            )
            {
                faseObstaculo =
                    OBSTACULO_BUSCA_LINHA;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 8. BUSCA LINHA
        // ====================================================

        case OBSTACULO_BUSCA_LINHA:

            if(linhaCentralizada(linha))
            {
                faseObstaculo =
                    OBSTACULO_ALINHAMENTO;

                inicioFase =
                    millis();

                break;
            }

            motorControlador.setSpeed(
                VELOCIDADE_ALINHAMENTO,
                VELOCIDADE_ALINHAMENTO,
                VELOCIDADE_ALINHAMENTO,
                VELOCIDADE_ALINHAMENTO
            );

            if(
                tempo >=
                TEMPO_BUSCA_LINHA
            )
            {
                faseObstaculo =
                    OBSTACULO_ALINHAMENTO;

                inicioFase =
                    millis();
            }

            break;


        // ====================================================
        // 9. ALINHAMENTO FINAL
        // ====================================================

        case OBSTACULO_ALINHAMENTO:

            executarAlinhamento(linha);

            break;
    }
}


// ============================================================
// EXECUTA MANOBRA ATUAL
// ============================================================

void executarManobra(
    const LinhaData& linha
)
{
    switch(manobraAtual)
    {
        case MANOBRA_CURVA_ESQUERDA:

            executarCurva(
                linha,
                true
            );

            break;


        case MANOBRA_CURVA_DIREITA:

            executarCurva(
                linha,
                false
            );

            break;


        case MANOBRA_INTERSECCAO:

            executarInterseccao();

            break;


        case MANOBRA_PASSAGEM_RETA:

            executarPassagemReta();

            break;


        case MANOBRA_MEIA_VOLTA:

            executarMeiaVolta(
                linha
            );

            break;


        case MANOBRA_OBSTACULO:

            executarObstaculo(
                linha
            );

            break;


        case MANOBRA_NENHUMA:

        default:

            break;
    }
}


// ============================================================
// ESCOLHE MANOBRA DA DECISÃO
// ============================================================

void iniciarManobraDaDecisao(
    const DecisaoData& decisao
)
{
    // --------------------------------------------------------
    // OBSTÁCULO — prioridade absoluta
    // --------------------------------------------------------

    if(decisao.obstaculoDetectado)
    {
        faseObstaculo =
            OBSTACULO_GIRO_DIREITA;

        iniciarManobra(
            MANOBRA_OBSTACULO
        );

        return;
    }


    // --------------------------------------------------------
    // VERDE
    // --------------------------------------------------------

    switch(decisao.acaoVerde)
    {
        case VERDE_CURVA_ESQUERDA:

            iniciarManobra(
                MANOBRA_CURVA_ESQUERDA
            );

            return;


        case VERDE_CURVA_DIREITA:

            iniciarManobra(
                MANOBRA_CURVA_DIREITA
            );

            return;


        case VERDE_MEIA_VOLTA:

            iniciarManobra(
                MANOBRA_MEIA_VOLTA
            );

            return;


        case VERDE_PASSAGEM_RETA:

            iniciarManobra(
                MANOBRA_PASSAGEM_RETA
            );

            return;


        case VERDE_SEM_EVENTO:

        default:

            break;
    }


    // --------------------------------------------------------
    // ARRAY
    // --------------------------------------------------------

    switch(decisao.eventoArray)
    {
        case ARRAY_CURVA_ESQUERDA:

            iniciarManobra(
                MANOBRA_CURVA_ESQUERDA
            );

            return;


        case ARRAY_CURVA_DIREITA:

            iniciarManobra(
                MANOBRA_CURVA_DIREITA
            );

            return;


        case ARRAY_INTERSECCAO:

            iniciarManobra(
                MANOBRA_INTERSECCAO
            );

            return;


        case ARRAY_NORMAL:

        default:

            break;
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    // ========================================================
    // BOTÃO
    // ========================================================

    pinMode(
        PINO_BOTAO,
        INPUT_PULLUP
    );


    // ========================================================
    // TCA
    // ========================================================

    tca.begin();


    // ========================================================
    // AS7341
    // ========================================================

    if(!sensoresAS7341.begin())
    {
        while(true)
        {
            delay(1000);
        }
    }


    // ========================================================
    // TOF
    // ========================================================

    if(!tof200f.begin(0))
    {
        while(true)
        {
            delay(1000);
        }
    }


    // ========================================================
    // ARRAY
    // ========================================================

    arrayLinha.begin();


    // ========================================================
    // MOTOR
    // ========================================================

    motorControlador.begin();


    // ========================================================
    // PID
    // ========================================================

    controleLinha.configurarPID(
        20.0f,
        0.0f,
        0.5f
    );

    controleLinha.configurarVelocidade(
        VELOCIDADE_RETA,
        1000
    );

    controleLinha.configurarVelocidadeAdaptativa(
        1.0f,
        6.0f,
        500
    );

    controleLinha.reset();


    // ========================================================
    // SEGURANÇA
    // ========================================================

    motorControlador.stop();


    Serial.println();
    Serial.println("========================================");
    Serial.println(" ROBO COMPLETO");
    Serial.println("========================================");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // BOTÃO — PRIORIDADE ABSOLUTA
    // ========================================================

    if(
        digitalRead(PINO_BOTAO) ==
        LOW
    )
    {
        motorControlador.stop();

        controleLinha.reset();

        manobraAtual =
            MANOBRA_NENHUMA;

        return;
    }


    // ========================================================
    // TOF
    // ========================================================

    tof200f.update();

    tofAnalise.update();


    // ========================================================
    // ARRAY
    // ========================================================

    arrayLinha.update();


    // ========================================================
    // AS7341
    // ========================================================

    sensoresAS7341.update();


    AS7341Data dadosEsquerda =
        sensoresAS7341.getEsquerda();

    AS7341Data dadosDireita =
        sensoresAS7341.getDireita();


    // ========================================================
    // ARRAY DATA
    // ========================================================

    ArrayData dadosLinha =
        arrayLinha.getData();


    // ========================================================
    // ANÁLISE AS7341
    // ========================================================

    AS7341Resultado esquerda =
        as7341Analise.analisar(
            dadosEsquerda
        );

    AS7341Resultado direita =
        as7341Analise.analisar(
            dadosDireita
        );


    // ========================================================
    // ANÁLISE LINHA
    // ========================================================

    linhaAnalise.update(
        dadosLinha
    );

    LinhaData linha =
        linhaAnalise.getData();


    // ========================================================
    // DECISÃO
    // ========================================================

    decisaoRobo.update(
        esquerda,
        direita,
        linha,
        tofAnalise
    );

    const DecisaoData& decisao =
        decisaoRobo.getData();


    // ========================================================
    // ESTADO
    // ========================================================

    estadoRobo.update(
        decisao
    );


    // ========================================================
    // TOF SEMPRE TEM PRIORIDADE
    // ========================================================

    if(decisao.obstaculoDetectado)
    {
        estadoRobo.definirEstado(
            ESTADO_OBSTACULO
        );
    }


    // ========================================================
    // SE NÃO EXISTE MANOBRA:
    // ESCOLHE UMA
    // ========================================================

    if(
        manobraAtual ==
        MANOBRA_NENHUMA
    )
    {
        iniciarManobraDaDecisao(
            decisao
        );
    }


    // ========================================================
    // EXECUTA MANOBRA
    // ========================================================

    if(
        manobraAtual !=
        MANOBRA_NENHUMA
    )
    {
        executarManobra(
            linha
        );
    }


    // ========================================================
    // PID NORMAL
    // ========================================================

    if(
        manobraAtual ==
        MANOBRA_NENHUMA
    )
    {
        controleLinha.update(
            linha
        );

        ControleData controle =
            controleLinha.getData();


        motorControlador.setSpeed(
            controle.velocidadeEsquerda,
            controle.velocidadeEsquerda,
            controle.velocidadeDireita,
            controle.velocidadeDireita
        );
    }


    // ========================================================
    // SERIAL
    // ========================================================

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        " ROBO"
    );

    Serial.println(
        "========================================"
    );


    Serial.print(
        "ToF: "
    );

    Serial.print(
        decisao.distanciaToF
    );

    Serial.println(
        " mm"
    );


    Serial.print(
        "Obstaculo: "
    );

    Serial.println(
        decisao.obstaculoDetectado
        ? "SIM"
        : "NAO"
    );


    Serial.print(
        "Verde E: "
    );

    Serial.println(
        decisao.verdeEsquerda
        ? "SIM"
        : "NAO"
    );


    Serial.print(
        "Verde D: "
    );

    Serial.println(
        decisao.verdeDireita
        ? "SIM"
        : "NAO"
    );


    Serial.print(
        "Pretos: "
    );

    Serial.println(
        decisao.sensoresPretos
    );


    Serial.print(
        "Posicao: "
    );

    Serial.println(
        decisao.posicaoDigital
    );


    Serial.print(
        "Evento array: "
    );

    Serial.println(
        nomeEventoArray(
            decisao.eventoArray
        )
    );


    Serial.print(
        "Acao verde: "
    );

    Serial.println(
        nomeAcaoVerde(
            decisao.acaoVerde
        )
    );


    Serial.print(
        "Estado: "
    );

    Serial.println(
        estadoRobo.getNomeEstado()
    );


    Serial.print(
        "Manobra: "
    );

    Serial.println(
        (int)manobraAtual
    );


    Serial.print(
        "Motor E: "
    );

    Serial.println(
        motorControlador.getM1()
    );


    Serial.print(
        "Motor D: "
    );

    Serial.println(
        motorControlador.getM3()
    );


    Serial.println(
        "========================================"
    );


    delay(10);
}