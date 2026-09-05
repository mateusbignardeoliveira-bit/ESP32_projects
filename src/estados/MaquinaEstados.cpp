#include "MaquinaEstados.h"


// ============================================================
// CONFIGURAÇÃO DA RÉ ANTES DE CURVAS NORMAIS
// ============================================================

static constexpr int VELOCIDADE_RE_CURVA = -100;

static constexpr int RE_CURVA_MS = 125;

static constexpr int PAUSA_APOS_RE_MS = 30;


// ============================================================
// CONFIGURAÇÃO DA CURVA VERDE
// ============================================================
//
// Sequência:
//
// 1. Giro controlado até 45°
// 2. Giro contínuo na mesma direção
// 3. Espera sensor físico 3, 4, 5 ou 6 detectar preto
// 4. Se sensor 4 ou 5 já estiver preto:
//        termina
// 5. Caso contrário:
//        faz pequena correção
// 6. Termina quando sensor 4 ou 5 estiver preto
//
// ============================================================

static constexpr int VELOCIDADE_GIRO_VERDE_CONTINUO = 100;

static constexpr int VELOCIDADE_CORRECAO_CURVA_VERDE = 100;

static constexpr unsigned long TIMEOUT_CURVA_VERDE_MS = 3000;

static constexpr unsigned long TIMEOUT_CORRECAO_CURVA_VERDE_MS = 800;


// ============================================================
// CONFIGURAÇÃO DA CURVA VERDE 180°
// ============================================================
//
// Sequência:
//
// 1. Dois verdes detectados
// 2. Giro controlado pela IMU até 120°
// 3. Abandona o alvo angular
// 4. Continua girando na mesma direção
// 5. Espera sensor físico 3, 4, 5 ou 6 detectar preto
// 6. Se sensor 4 ou 5 estiver preto:
//        termina
// 7. Caso contrário:
//        faz pequena correção
// 8. Termina quando sensor 4 ou 5 estiver preto
//
// ============================================================

static constexpr float ANGULO_INICIAL_CURVA_180 = 120.0f;


// ============================================================
// CONFIGURAÇÃO DA MANOBRA DE OBSTÁCULO
// ============================================================

static constexpr int VELOCIDADE_RE_OBSTACULO = -180;

static constexpr int RE_OBSTACULO_MS = 300;

static constexpr int PAUSA_APOS_RE_OBSTACULO_MS = 30;

static constexpr unsigned long OBSTACULO_RETO_INICIAL_MS = 1300;

static constexpr unsigned long OBSTACULO_RETO_ORBITA_MS = 2500;


// ------------------------------------------------------------
// Confirmação de linha durante obstáculo
// ------------------------------------------------------------

static constexpr int LEITURAS_PRETAS_CONFIRMAR_OBSTACULO = 3;

static constexpr float LIMIAR_PRETO_OBSTACULO = 0.35f;


// ============================================================
// CONFIGURAÇÃO DA CURVA PRETA
// ============================================================

static constexpr float LIMIAR_PRETO_CURVA = 0.35f;

static constexpr int VELOCIDADE_CURVA_PRETA = 100;


// ------------------------------------------------------------
// Avanço antes da curva preta
// ------------------------------------------------------------
//
// Aumente TEMPO_AVANCO_CURVA_PRETA_MS se quiser que o robô
// avance mais antes de começar a girar.
//
// ============================================================

static constexpr int VELOCIDADE_AVANCO_CURVA_PRETA = 100;

static constexpr unsigned long TEMPO_AVANCO_CURVA_PRETA_MS = 50;


// ============================================================
// CONSTRUTOR
// ============================================================

MaquinaEstados::MaquinaEstados(
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
)
:
    analiseLinha(linha),
    sensorCorEsquerda(corEsq),
    sensorCorDireita(corDir),
    analiseVerde(verde),
    analiseTOF(tof),
    sensorIMU(imu),
    controleLinha(controleLinhaRef),
    controleGiro(controleGiroRef),
    controleObstaculo(controleObstaculoRef),
    motores(motoresRef)
{
    estado = INICIO;

    acaoGiro = NENHUM_GIRO;

    tendenciaAntes = 0.0f;


    inicioAvaliacao = 0;

    tempoMinimoAvaliacao = 400;

    tempoMaximoAvaliacao = 1000;


    inicioBusca = 0;

    tempoMaximoBusca = 2000;


    direcaoCurvaVerde = NENHUM_GIRO;

    inicioCurvaVerde = 0;

    inicioCorrecaoCurvaVerde = 0;


    direcaoCurvaPreta = NENHUM_GIRO;

    inicioAvancoCurvaPreta = 0;


    inicioTrechoObstaculo = 0;

    tempoRetoObstaculoInicial =
        OBSTACULO_RETO_INICIAL_MS;

    tempoRetoObstaculoOrbita =
        OBSTACULO_RETO_ORBITA_MS;


    leiturasPretasObstaculo = 0;
}


// ============================================================
// RÉ ANTES DE CURVA NORMAL
// ============================================================

void MaquinaEstados::reAntesDaCurva()
{
    motores.setSpeed(
        VELOCIDADE_RE_CURVA,
        VELOCIDADE_RE_CURVA,
        VELOCIDADE_RE_CURVA,
        VELOCIDADE_RE_CURVA
    );

    delay(RE_CURVA_MS);

    motores.stop();

    delay(PAUSA_APOS_RE_MS);
}


// ============================================================
// BEGIN
// ============================================================

void MaquinaEstados::begin()
{
    estado = INICIO;

    acaoGiro = NENHUM_GIRO;

    tendenciaAntes = 0.0f;


    inicioAvaliacao = 0;

    inicioBusca = 0;


    direcaoCurvaVerde = NENHUM_GIRO;

    inicioCurvaVerde = 0;

    inicioCorrecaoCurvaVerde = 0;


    direcaoCurvaPreta = NENHUM_GIRO;

    inicioAvancoCurvaPreta = 0;


    inicioTrechoObstaculo = 0;

    leiturasPretasObstaculo = 0;


    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.cancelar();

    analiseVerde.reset();
}


// ============================================================
// RESET EXECUÇÃO
// ============================================================

void MaquinaEstados::resetExecucao()
{
    estado = INICIO;

    acaoGiro = NENHUM_GIRO;

    tendenciaAntes = 0.0f;


    inicioAvaliacao = 0;

    inicioBusca = 0;


    direcaoCurvaVerde = NENHUM_GIRO;

    inicioCurvaVerde = 0;

    inicioCorrecaoCurvaVerde = 0;


    direcaoCurvaPreta = NENHUM_GIRO;

    inicioAvancoCurvaPreta = 0;


    inicioTrechoObstaculo = 0;

    leiturasPretasObstaculo = 0;


    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.cancelar();

    analiseVerde.reset();


    motores.stop();

    delay(100);

    motores.release();
}


// ============================================================
// UPDATE
// ============================================================

void MaquinaEstados::update(
    const LinhaData& linha,
    const AS7341Data& dadosCorEsquerda,
    const AS7341Data& dadosCorDireita
)
{
    switch(estado)
    {

        // ====================================================
        // INÍCIO
        // ====================================================

        case INICIO:

            controleLinha.start();

            entrarEstado(
                SEGUINDO_LINHA
            );

            break;


        // ====================================================
        // SEGUINDO LINHA
        // ====================================================

        case SEGUINDO_LINHA:

            processarSeguindoLinha(linha);

            break;


        // ====================================================
        // AVALIANDO MARCA
        // ====================================================

        case AVALIANDO_MARCA:

            processarAvaliacao(
                linha,
                dadosCorEsquerda,
                dadosCorDireita
            );

            break;


        // ====================================================
        // GIRO NORMAL
        // ====================================================

        case EXECUTANDO_GIRO:

            processarGiro(linha);

            break;


        // ====================================================
        // GIRO VERDE DE 45°
        // ====================================================

        case GIRO_VERDE_45:

            processarGiroVerde45(linha);

            break;


        // ====================================================
        // CURVA VERDE CONTÍNUA
        // ====================================================

        case CURVA_VERDE_CONTINUA:

            processarCurvaVerdeContinua(linha);

            break;


        // ====================================================
        // CORREÇÃO FINAL DA CURVA VERDE
        // ====================================================

        case CORRECAO_CURVA_VERDE:

            processarCorrecaoCurvaVerde(linha);

            break;


        // ====================================================
        // AVANÇO ANTES DA CURVA PRETA
        // ====================================================

        case AVANCO_CURVA_PRETA:

            processarAvancoCurvaPreta();

            break;


        // ====================================================
        // CURVA PRETA
        // ====================================================

        case CURVA_PRETA:

            processarCurvaPreta(linha);

            break;


        // ====================================================
        // BUSCANDO LINHA
        // ====================================================

        case BUSCANDO_LINHA:

            if(
                linhaEncontrada(linha)
            )
            {
                controleLinha.start();

                entrarEstado(
                    SEGUINDO_LINHA
                );
            }

            break;


        // ====================================================
        // MANOBRA DO OBSTÁCULO
        // ====================================================

        case OBSTACULO_GIRO_1:

        case OBSTACULO_RETO_INICIAL:

        case OBSTACULO_GIRO_2:

        case OBSTACULO_ORBITA_RETO:

        case OBSTACULO_ORBITA_GIRO:

        case OBSTACULO_GIRO_FINAL:

            processarObstaculo(linha);

            break;


        // ====================================================
        // STOP VERMELHO
        // ====================================================

        case STOP_VERMELHO:

            motores.stop();

            break;
    }
}


// ============================================================
// SEGUINDO LINHA
// ============================================================

void MaquinaEstados::processarSeguindoLinha(
    const LinhaData& linha
)
{
    // --------------------------------------------------------
    // Obstáculo tem prioridade.
    // --------------------------------------------------------

    if(
        analiseTOF.temObstaculo()
    )
    {
        iniciarObstaculo();

        return;
    }


    // --------------------------------------------------------
    // Região especial.
    // --------------------------------------------------------

    if(
        contarSensoresPretos(linha) >= 4
    )
    {
        iniciarAvaliacao(linha);

        return;
    }


    controleLinha.update(linha);
}


// ============================================================
// INICIAR AVALIAÇÃO
// ============================================================

void MaquinaEstados::iniciarAvaliacao(
    const LinhaData& linha
)
{
    controleLinha.stop();

    motores.stop();


    tendenciaAntes =
        linha.posicao;


    int quantidadePretos =
        contarSensoresPretos(linha);


    analiseVerde.iniciar(
        quantidadePretos
    );


    inicioAvaliacao =
        millis();


    entrarEstado(
        AVALIANDO_MARCA
    );
}


// ============================================================
// PROCESSAR AVALIAÇÃO
// ============================================================

void MaquinaEstados::processarAvaliacao(
    const LinhaData& linha,
    const AS7341Data& dadosCorEsquerda,
    const AS7341Data& dadosCorDireita
)
{
    motores.setSpeed(
        100,
        100,
        100,
        100
    );


    int quantidadePretos =
        contarSensoresPretos(linha);


    analiseVerde.update(
        dadosCorEsquerda,
        dadosCorDireita,
        quantidadePretos
    );


    unsigned long tempo =
        millis() -
        inicioAvaliacao;


    if(
        tempo >= tempoMinimoAvaliacao &&
        quantidadePretos < 4
    )
    {
        finalizarAvaliacao(linha);

        return;
    }


    if(
        tempo >= tempoMaximoAvaliacao
    )
    {
        finalizarAvaliacao(linha);
    }
}


// ============================================================
// FINALIZAR AVALIAÇÃO
// ============================================================

void MaquinaEstados::finalizarAvaliacao(
    const LinhaData& linha
)
{
    motores.stop();

    analiseVerde.finalizar();


    // ========================================================
    // VERMELHO
    // ========================================================

    if(
        analiseVerde.detectouVermelho()
    )
    {
        pararPorVermelho();

        return;
    }


    // ========================================================
    // VERDE
    // ========================================================

    bool esquerda =
        analiseVerde.detectouVerdeEsquerda();

    bool direita =
        analiseVerde.detectouVerdeDireita();


    // --------------------------------------------------------
    // Verde dos dois lados = 180°
    //
    // Nova estratégia:
    //
    // 1. Giro controlado pela IMU até 120°
    // 2. Giro contínuo na mesma direção
    // 3. Array encontra a linha
    //
    // --------------------------------------------------------

    if(
        esquerda &&
        direita
    )
    {
        iniciarGiro(
            GIRO_180
        );

        return;
    }


    // --------------------------------------------------------
    // Verde esquerda
    //
    // Nova estratégia:
    // 45° controlados + giro contínuo.
    // --------------------------------------------------------

    if(esquerda)
    {
        reAntesDaCurva();

        iniciarGiroVerde(
            GIRO_ESQUERDA
        );

        return;
    }


    // --------------------------------------------------------
    // Verde direita
    //
    // Nova estratégia:
    // 45° controlados + giro contínuo.
    // --------------------------------------------------------

    if(direita)
    {
        reAntesDaCurva();

        iniciarGiroVerde(
            GIRO_DIREITA
        );

        return;
    }


    // ========================================================
    // CURVA PRETA
    // ========================================================

    if(
        todosBrancos(linha)
    )
    {
        iniciarCurvaPreta();

        return;
    }


    // ========================================================
    // FALSA CURVA
    // ========================================================

    controleLinha.start();

    entrarEstado(
        SEGUINDO_LINHA
    );
}


// ============================================================
// INICIAR GIRO NORMAL
// ============================================================

void MaquinaEstados::iniciarGiro(
    AcaoGiro acao
)
{
    controleLinha.stop();

    motores.stop();


    acaoGiro = acao;


    switch(acao)
    {

        case GIRO_ESQUERDA:

            controleGiro.curva90Esquerda();

            break;


        case GIRO_DIREITA:

            controleGiro.curva90Direita();

            break;


        case GIRO_180:

            // ------------------------------------------------
            // Para 180°:
            //
            // Não vamos mais controlar os 180° completos
            // pelo giroscópio.
            //
            // Primeiro fazemos apenas 120°.
            // Depois processarGiro() passa para o giro
            // contínuo na mesma direção.
            // ------------------------------------------------

            controleGiro.girar(
                ANGULO_INICIAL_CURVA_180
            );

            break;


        default:

            controleLinha.start();

            entrarEstado(
                SEGUINDO_LINHA
            );

            return;
    }


    entrarEstado(
        EXECUTANDO_GIRO
    );
}


// ============================================================
// PROCESSAR GIRO NORMAL
// ============================================================

void MaquinaEstados::processarGiro(
    const LinhaData& linha
)
{
    (void)linha;

    controleGiro.update();


    if(
        !controleGiro.terminou()
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Caso especial:
    //
    // GIRO_180 não termina realmente nos 120°.
    //
    // Os 120° são somente a primeira etapa.
    // Depois continuamos girando até o array encontrar
    // a linha.
    // --------------------------------------------------------

    if(
        acaoGiro ==
        GIRO_180
    )
    {
        Serial.println(
            "CURVA 180 | 120 CONCLUIDOS"
        );

        Serial.println(
            "CURVA 180 | GIRO CONTINUO"
        );


        // ----------------------------------------------------
        // O giro180 atual usa heading positivo.
        // Portanto mantemos a mesma direção física:
        // DIREITA.
        // ----------------------------------------------------

        direcaoCurvaVerde =
            GIRO_DIREITA;


        controleGiro.giroContinuoDireita(
            VELOCIDADE_GIRO_VERDE_CONTINUO
        );


        inicioCurvaVerde =
            millis();


        acaoGiro =
            NENHUM_GIRO;


        entrarEstado(
            CURVA_VERDE_CONTINUA
        );

        return;
    }


    // --------------------------------------------------------
    // Giro normal de 90° ou outro giro convencional.
    // --------------------------------------------------------

    acaoGiro =
        NENHUM_GIRO;


    controleLinha.start();

    entrarEstado(
        SEGUINDO_LINHA
    );
}


// ============================================================
// INICIAR GIRO VERDE
// ============================================================
//
// Primeiro estágio:
// giro preciso de 45° usando IMU.
//
// Depois o estado GIRO_VERDE_45 detecta o término e passa
// automaticamente para o giro contínuo.
// ============================================================

void MaquinaEstados::iniciarGiroVerde(
    AcaoGiro acao
)
{
    controleLinha.stop();

    controleGiro.cancelar();

    motores.stop();


    direcaoCurvaVerde =
        acao;


    inicioCurvaVerde =
        millis();


    switch(acao)
    {

        case GIRO_ESQUERDA:

            Serial.println(
                "CURVA VERDE ESQUERDA | INICIO 45"
            );

            controleGiro.curva45Esquerda();

            break;


        case GIRO_DIREITA:

            Serial.println(
                "CURVA VERDE DIREITA | INICIO 45"
            );

            controleGiro.curva45Direita();

            break;


        default:

            controleLinha.start();

            entrarEstado(
                SEGUINDO_LINHA
            );

            return;
    }


    entrarEstado(
        GIRO_VERDE_45
    );
}


// ============================================================
// PROCESSAR GIRO VERDE 45°
// ============================================================

void MaquinaEstados::processarGiroVerde45(
    const LinhaData& linha
)
{
    (void)linha;

    controleGiro.update();


    // --------------------------------------------------------
    // Ainda não chegou nos 45°.
    // --------------------------------------------------------

    if(
        !controleGiro.terminou()
    )
    {
        return;
    }


    // --------------------------------------------------------
    // 45° concluídos.
    //
    // Agora abandonamos completamente o alvo angular.
    // --------------------------------------------------------

    Serial.println(
        "CURVA VERDE | 45 CONCLUIDOS"
    );

    Serial.println(
        "CURVA VERDE | GIRO CONTINUO"
    );


    if(
        direcaoCurvaVerde ==
        GIRO_ESQUERDA
    )
    {
        controleGiro.giroContinuoEsquerda(
            VELOCIDADE_GIRO_VERDE_CONTINUO
        );
    }
    else
    {
        controleGiro.giroContinuoDireita(
            VELOCIDADE_GIRO_VERDE_CONTINUO
        );
    }


    inicioCurvaVerde =
        millis();


    entrarEstado(
        CURVA_VERDE_CONTINUA
    );
}


// ============================================================
// PROCESSAR CURVA VERDE CONTÍNUA
// ============================================================
//
// Durante essa fase:
//
// NÃO usamos alvo angular.
//
// NÃO usamos o heading para determinar quando parar.
//
// O robô simplesmente continua girando no mesmo sentido.
//
// A parada acontece quando qualquer um dos sensores físicos
// 3, 4, 5 ou 6 detectar preto.
// ============================================================

void MaquinaEstados::processarCurvaVerdeContinua(
    const LinhaData& linha
)
{
    // --------------------------------------------------------
    // Segurança contra giro infinito.
    // --------------------------------------------------------

    if(
        millis() - inicioCurvaVerde >=
        TIMEOUT_CURVA_VERDE_MS
    )
    {
        Serial.println(
            "CURVA VERDE | TIMEOUT"
        );

        controleGiro.pararGiroContinuo();

        controleLinha.start();

        entrarEstado(
            SEGUINDO_LINHA
        );

        return;
    }


    // --------------------------------------------------------
    // Nenhum dos sensores centrais encontrou preto ainda.
    // --------------------------------------------------------

    if(
        !sensorCentralEncontrouPreto(linha)
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Encontramos a linha.
    // --------------------------------------------------------

    Serial.println(
        "CURVA VERDE | PRETO ENCONTRADO"
    );


    // --------------------------------------------------------
    // Se sensor físico 4 ou 5 já está na linha,
    // não precisamos corrigir.
    // --------------------------------------------------------

    if(
        sensorCentroNaLinha(linha)
    )
    {
        Serial.println(
            "CURVA VERDE | SENSOR 4/5 CENTRALIZADO"
        );

        controleGiro.pararGiroContinuo();

        controleLinha.start();

        entrarEstado(
            SEGUINDO_LINHA
        );

        return;
    }


    // --------------------------------------------------------
    // Preto apareceu no sensor 3 ou 6.
    //
    // Fazemos a pequena correção final.
    // --------------------------------------------------------

    iniciarCorrecaoCurvaVerde(linha);
}


// ============================================================
// INICIAR CORREÇÃO DA CURVA VERDE
// ============================================================

void MaquinaEstados::iniciarCorrecaoCurvaVerde(
    const LinhaData& linha
)
{
    AcaoGiro correcao =
        determinarDirecaoCorrecao(linha);


    if(
        correcao ==
        NENHUM_GIRO
    )
    {
        // Situação ambígua.
        // Por segurança, retoma o PID.

        controleGiro.pararGiroContinuo();

        controleLinha.start();

        entrarEstado(
            SEGUINDO_LINHA
        );

        return;
    }


    // --------------------------------------------------------
    // Primeiro para o giro contínuo.
    // --------------------------------------------------------

    controleGiro.pararGiroContinuo();


    direcaoCurvaVerde =
        correcao;


    inicioCorrecaoCurvaVerde =
        millis();


    // --------------------------------------------------------
    // Começa correção lenta.
    // --------------------------------------------------------

    if(
        correcao ==
        GIRO_ESQUERDA
    )
    {
        Serial.println(
            "CURVA VERDE | CORRECAO ESQUERDA"
        );

        controleGiro.giroContinuoEsquerda(
            VELOCIDADE_CORRECAO_CURVA_VERDE
        );
    }
    else
    {
        Serial.println(
            "CURVA VERDE | CORRECAO DIREITA"
        );

        controleGiro.giroContinuoDireita(
            VELOCIDADE_CORRECAO_CURVA_VERDE
        );
    }


    entrarEstado(
        CORRECAO_CURVA_VERDE
    );
}


// ============================================================
// PROCESSAR CORREÇÃO DA CURVA VERDE
// ============================================================

void MaquinaEstados::processarCorrecaoCurvaVerde(
    const LinhaData& linha
)
{
    // --------------------------------------------------------
    // Objetivo da correção:
    //
    // sensor físico 4 OU sensor físico 5 preto.
    // --------------------------------------------------------

    if(
        sensorCentroNaLinha(linha)
    )
    {
        Serial.println(
            "CURVA VERDE | CORRECAO CONCLUIDA"
        );


        controleGiro.pararGiroContinuo();


        controleLinha.start();

        entrarEstado(
            SEGUINDO_LINHA
        );

        return;
    }


    // --------------------------------------------------------
    // Segurança.
    // --------------------------------------------------------

    if(
        millis() - inicioCorrecaoCurvaVerde >=
        TIMEOUT_CORRECAO_CURVA_VERDE_MS
    )
    {
        Serial.println(
            "CURVA VERDE | TIMEOUT CORRECAO"
        );


        controleGiro.pararGiroContinuo();


        controleLinha.start();

        entrarEstado(
            SEGUINDO_LINHA
        );

        return;
    }
}


// ============================================================
// SENSOR CENTRAL ENCONTROU PRETO
// ============================================================
//
// Sensores físicos:
//
// 3 -> sensores[2]
// 4 -> sensores[3]
// 5 -> sensores[4]
// 6 -> sensores[5]
// ============================================================

bool MaquinaEstados::sensorCentralEncontrouPreto(
    const LinhaData& linha
)
{
    return
        linha.sensores[2] >= LIMIAR_PRETO_CURVA ||
        linha.sensores[3] >= LIMIAR_PRETO_CURVA ||
        linha.sensores[4] >= LIMIAR_PRETO_CURVA ||
        linha.sensores[5] >= LIMIAR_PRETO_CURVA;
}


// ============================================================
// SENSOR 4 OU 5 NA LINHA
// ============================================================

bool MaquinaEstados::sensorCentroNaLinha(
    const LinhaData& linha
)
{
    return
        linha.sensores[3] >= LIMIAR_PRETO_CURVA ||
        linha.sensores[4] >= LIMIAR_PRETO_CURVA;
}


// ============================================================
// DETERMINAR DIREÇÃO DA CORREÇÃO
// ============================================================
//
// Se o sensor físico 3 detectou preto:
//
//     linha está para a esquerda
//     -> pequena correção para esquerda
//
// Se o sensor físico 6 detectou preto:
//
//     linha está para a direita
//     -> pequena correção para direita
//
// Se ambos estiverem ativos, usamos a posição calculada
// pela LinhaAnalise como desempate.
// ============================================================

MaquinaEstados::AcaoGiro
MaquinaEstados::determinarDirecaoCorrecao(
    const LinhaData& linha
)
{
    bool sensor3 =
        linha.sensores[2] >=
        LIMIAR_PRETO_CURVA;

    bool sensor6 =
        linha.sensores[5] >=
        LIMIAR_PRETO_CURVA;


    if(
        sensor3 &&
        !sensor6
    )
    {
        return GIRO_ESQUERDA;
    }


    if(
        sensor6 &&
        !sensor3
    )
    {
        return GIRO_DIREITA;
    }


    // --------------------------------------------------------
    // Caso ambíguo:
    // usa posição da linha.
    //
    // posição negativa = esquerda
    // posição positiva = direita
    // --------------------------------------------------------

    if(linha.posicao < -0.5f)
    {
        return GIRO_ESQUERDA;
    }


    if(linha.posicao > 0.5f)
    {
        return GIRO_DIREITA;
    }


    return NENHUM_GIRO;
}


// ============================================================
// INICIAR CURVA PRETA
// ============================================================
//
// Agora há um avanço antes da rotação.
//
// A direção é determinada pela tendência registrada antes
// da avaliação.
// ============================================================

void MaquinaEstados::iniciarCurvaPreta()
{
    controleLinha.stop();

    controleGiro.cancelar();

    motores.stop();


    // --------------------------------------------------------
    // Tendência negativa = curva para direita.
    // --------------------------------------------------------

    if(
        tendenciaAntes < 0.0f
    )
    {
        direcaoCurvaPreta =
            GIRO_DIREITA;
    }


    // --------------------------------------------------------
    // Tendência positiva = curva para esquerda.
    // --------------------------------------------------------

    else if(
        tendenciaAntes > 0.0f
    )
    {
        direcaoCurvaPreta =
            GIRO_ESQUERDA;
    }


    // --------------------------------------------------------
    // Sem tendência confiável.
    // --------------------------------------------------------

    else
    {
        controleLinha.start();

        entrarEstado(
            SEGUINDO_LINHA
        );

        return;
    }


    // --------------------------------------------------------
    // Primeiro avança para frente.
    // --------------------------------------------------------

    Serial.println(
        "CURVA PRETA | AVANCO"
    );


    motores.setSpeed(
        VELOCIDADE_AVANCO_CURVA_PRETA,
        VELOCIDADE_AVANCO_CURVA_PRETA,
        VELOCIDADE_AVANCO_CURVA_PRETA,
        VELOCIDADE_AVANCO_CURVA_PRETA
    );


    inicioAvancoCurvaPreta =
        millis();


    entrarEstado(
        AVANCO_CURVA_PRETA
    );
}


// ============================================================
// PROCESSAR AVANÇO DA CURVA PRETA
// ============================================================

void MaquinaEstados::processarAvancoCurvaPreta()
{
    if(
        millis() -
        inicioAvancoCurvaPreta <
        TEMPO_AVANCO_CURVA_PRETA_MS
    )
    {
        return;
    }


    motores.stop();


    delay(30);


    iniciarRotacaoCurvaPreta();
}


// ============================================================
// INICIAR ROTAÇÃO DA CURVA PRETA
// ============================================================

void MaquinaEstados::iniciarRotacaoCurvaPreta()
{
    // --------------------------------------------------------
    // Direita
    // --------------------------------------------------------

    if(
        direcaoCurvaPreta ==
        GIRO_DIREITA
    )
    {
        motores.setSpeed(
            -VELOCIDADE_CURVA_PRETA,
            -VELOCIDADE_CURVA_PRETA,
            VELOCIDADE_CURVA_PRETA,
            VELOCIDADE_CURVA_PRETA
        );
    }


    // --------------------------------------------------------
    // Esquerda
    // --------------------------------------------------------

    else if(
        direcaoCurvaPreta ==
        GIRO_ESQUERDA
    )
    {
        motores.setSpeed(
            VELOCIDADE_CURVA_PRETA,
            VELOCIDADE_CURVA_PRETA,
            -VELOCIDADE_CURVA_PRETA,
            -VELOCIDADE_CURVA_PRETA
        );
    }


    else
    {
        controleLinha.start();

        entrarEstado(
            SEGUINDO_LINHA
        );

        return;
    }


    entrarEstado(
        CURVA_PRETA
    );
}


// ============================================================
// PROCESSAR CURVA PRETA
// ============================================================
//
// Durante a curva preta NÃO usamos o giroscópio.
//
// O robô continua girando até que:
// sensor físico 4 OU sensor físico 5
// detecte preto.
// ============================================================

void MaquinaEstados::processarCurvaPreta(
    const LinhaData& linha
)
{
    bool sensor4Preto =
        linha.sensores[3] >=
        LIMIAR_PRETO_CURVA;


    bool sensor5Preto =
        linha.sensores[4] >=
        LIMIAR_PRETO_CURVA;


    if(
        sensor4Preto ||
        sensor5Preto
    )
    {
        Serial.println(
            "CURVA PRETA | LINHA ENCONTRADA"
        );


        motores.stop();


        controleLinha.start();


        entrarEstado(
            SEGUINDO_LINHA
        );


        return;
    }
}


// ============================================================
// INICIAR OBSTÁCULO
// ============================================================

void MaquinaEstados::iniciarObstaculo()
{
    controleLinha.stop();

    controleObstaculo.cancelar();

    controleGiro.cancelar();

    motores.stop();


    // --------------------------------------------------------
    // Primeiro: ré.
    // --------------------------------------------------------

    motores.setSpeed(
        VELOCIDADE_RE_OBSTACULO,
        VELOCIDADE_RE_OBSTACULO,
        VELOCIDADE_RE_OBSTACULO,
        VELOCIDADE_RE_OBSTACULO
    );

    delay(RE_OBSTACULO_MS);


    motores.stop();

    delay(PAUSA_APOS_RE_OBSTACULO_MS);


    // --------------------------------------------------------
    // Primeiro giro: direita.
    // --------------------------------------------------------

    controleGiro.curva90Direita();


    entrarEstado(
        OBSTACULO_GIRO_1
    );
}


// ============================================================
// INICIAR TRECHO RETO DO OBSTÁCULO
// ============================================================

void MaquinaEstados::iniciarRetoObstaculo(
    unsigned long duracao
)
{
    controleGiro.cancelar();

    controleObstaculo.parar();


    leiturasPretasObstaculo = 0;


    inicioTrechoObstaculo =
        millis();


    if(
        duracao ==
        tempoRetoObstaculoInicial
    )
    {
        entrarEstado(
            OBSTACULO_RETO_INICIAL
        );
    }
    else
    {
        entrarEstado(
            OBSTACULO_ORBITA_RETO
        );
    }


    controleObstaculo.iniciarReto();
}


// ============================================================
// PROCESSAR OBSTÁCULO
// ============================================================

void MaquinaEstados::processarObstaculo(
    const LinhaData& linha
)
{
    switch(estado)
    {

        // ====================================================
        // PRIMEIRO GIRO
        // ====================================================

        case OBSTACULO_GIRO_1:

            controleGiro.update();


            if(
                controleGiro.terminou()
            )
            {
                iniciarRetoObstaculo(
                    tempoRetoObstaculoInicial
                );
            }

            break;


        // ====================================================
        // PRIMEIRO RETO
        // ====================================================

        case OBSTACULO_RETO_INICIAL:

            controleObstaculo.update();


            if(
                linhaPretaConfirmada(linha)
            )
            {
                controleObstaculo.parar();

                motores.stop();

                controleGiro.curva90Direita();

                entrarEstado(
                    OBSTACULO_GIRO_FINAL
                );

                break;
            }


            if(
                millis() -
                inicioTrechoObstaculo
                >=
                tempoRetoObstaculoInicial
            )
            {
                controleObstaculo.parar();

                controleGiro.curva90Esquerda();

                entrarEstado(
                    OBSTACULO_GIRO_2
                );
            }

            break;


        // ====================================================
        // SEGUNDO GIRO
        // ====================================================

        case OBSTACULO_GIRO_2:

            controleGiro.update();


            if(
                controleGiro.terminou()
            )
            {
                iniciarRetoObstaculo(
                    tempoRetoObstaculoOrbita
                );
            }

            break;


        // ====================================================
        // RETA DA ÓRBITA
        // ====================================================

        case OBSTACULO_ORBITA_RETO:

            controleObstaculo.update();


            if(
                linhaPretaConfirmada(linha)
            )
            {
                controleObstaculo.parar();

                motores.stop();

                controleGiro.curva90Direita();

                entrarEstado(
                    OBSTACULO_GIRO_FINAL
                );

                break;
            }


            if(
                millis() -
                inicioTrechoObstaculo
                >=
                tempoRetoObstaculoOrbita
            )
            {
                controleObstaculo.parar();

                controleGiro.curva90Esquerda();

                entrarEstado(
                    OBSTACULO_ORBITA_GIRO
                );
            }

            break;


        // ====================================================
        // GIRO DA ÓRBITA
        // ====================================================

        case OBSTACULO_ORBITA_GIRO:

            controleGiro.update();


            if(
                controleGiro.terminou()
            )
            {
                iniciarRetoObstaculo(
                    tempoRetoObstaculoOrbita
                );
            }

            break;


        // ====================================================
        // GIRO FINAL PARA A LINHA
        // ====================================================

        case OBSTACULO_GIRO_FINAL:

            controleGiro.update();


            if(
                controleGiro.terminou()
            )
            {
                controleLinha.start();

                entrarEstado(
                    SEGUINDO_LINHA
                );
            }

            break;


        default:

            break;
    }
}


// ============================================================
// CONFIRMAR PRETO DURANTE OBSTÁCULO
// ============================================================

bool MaquinaEstados::linhaPretaConfirmada(
    const LinhaData& linha
)
{
    bool encontrouPreto = false;


    for(
        int i = 0;
        i < 8;
        i++
    )
    {
        if(
            linha.sensores[i] >=
            LIMIAR_PRETO_OBSTACULO
        )
        {
            encontrouPreto = true;

            break;
        }
    }


    if(
        encontrouPreto
    )
    {
        leiturasPretasObstaculo++;


        if(
            leiturasPretasObstaculo >=
            LEITURAS_PRETAS_CONFIRMAR_OBSTACULO
        )
        {
            return true;
        }
    }
    else
    {
        leiturasPretasObstaculo = 0;
    }


    return false;
}


// ============================================================
// INICIAR BUSCA NORMAL
// ============================================================

void MaquinaEstados::iniciarBuscaLinha()
{
    inicioBusca =
        millis();

    entrarEstado(
        BUSCANDO_LINHA
    );
}


// ============================================================
// LINHA ENCONTRADA
// ============================================================

bool MaquinaEstados::linhaEncontrada(
    const LinhaData& linha
)
{
    if(
        !linha.linhaDetectada
    )
    {
        return false;
    }


    if(
        linha.intensidade <
        0.15f
    )
    {
        return false;
    }


    return true;
}


// ============================================================
// TODOS BRANCOS
// ============================================================

bool MaquinaEstados::todosBrancos(
    const LinhaData& linha
)
{
    for(
        int i = 0;
        i < 8;
        i++
    )
    {
        if(
            linha.sensores[i] >
            0.08f
        )
        {
            return false;
        }
    }


    return true;
}


// ============================================================
// CONTAR SENSORES PRETOS
// ============================================================

int MaquinaEstados::contarSensoresPretos(
    const LinhaData& linha
)
{
    int quantidade = 0;


    for(
        int i = 0;
        i < 8;
        i++
    )
    {
        if(
            linha.sensores[i] >=
            0.35f
        )
        {
            quantidade++;
        }
    }


    return quantidade;
}


// ============================================================
// ENTRAR ESTADO
// ============================================================

void MaquinaEstados::entrarEstado(
    Estado novoEstado
)
{
    estado =
        novoEstado;
}


// ============================================================
// PARAR POR VERMELHO
// ============================================================

void MaquinaEstados::pararPorVermelho()
{
    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.parar();

    motores.stop();

    delay(100);

    motores.release();


    entrarEstado(
        STOP_VERMELHO
    );
}


// ============================================================
// GET ESTADO
// ============================================================

MaquinaEstados::Estado
MaquinaEstados::getEstado() const
{
    return estado;
}


// ============================================================
// PARADO
// ============================================================

bool MaquinaEstados::parado() const
{
    return (
        estado ==
        STOP_VERMELHO
    );
}


// ============================================================
// PARADO POR VERMELHO
// ============================================================

bool MaquinaEstados::paradoPorVermelho() const
{
    return (
        estado ==
        STOP_VERMELHO
    );
}