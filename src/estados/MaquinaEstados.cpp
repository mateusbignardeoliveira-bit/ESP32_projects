 #include "MaquinaEstados.h"


// ============================================================
// CONFIGURAÇÃO DA RÉ ANTES DE CURVAS NORMAIS
// ============================================================

static constexpr int VELOCIDADE_RE_CURVA = -100;

static constexpr int RE_CURVA_MS = 125;

static constexpr int PAUSA_APOS_RE_MS = 30;


// ============================================================
// CONFIGURAÇÃO DA MANOBRA DE OBSTÁCULO
// ============================================================
//
// Entrada:
//
// 1. Ré
// 2. 90° direita
// 3. Reto 500 ms
// 4. 90° esquerda
//
// Depois:
//
// Reto 1500 ms
// 90° esquerda
// Reto 1500 ms
// 90° esquerda
// ...
//
// Durante os RETOS o array procura preto.
// Durante os GIROS o array não é considerado.
//
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
//
// A curva preta NÃO usa o giroscópio.
//
// O robô gira de acordo com a tendência anterior:
//
// tendência < 0 -> direita
// tendência > 0 -> esquerda
//
// O giro termina quando o sensor físico 4 ou 5
// volta a detectar preto.
//
// LinhaData usa índice começando em 0:
//
// sensor físico 4 -> sensores[3]
// sensor físico 5 -> sensores[4]
//
// ============================================================

static constexpr float LIMIAR_PRETO_CURVA = 0.35f;

static constexpr int VELOCIDADE_CURVA_PRETA = 100;


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

    tempoMinimoAvaliacao = 150;

    tempoMaximoAvaliacao = 1000;


    inicioBusca = 0;

    tempoMaximoBusca = 2000;


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
        // CURVA PRETA
        // ====================================================

        case CURVA_PRETA:

            processarCurvaPreta(linha);

            break;


        // ====================================================
        // BUSCANDO LINHA NORMAL
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
        // STOP CINZA
        // ====================================================

        case STOP_CINZA:

            motores.stop();

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
    // CINZA
    // ========================================================

    if(
        analiseVerde.detectouCinza()
    )
    {
        pararPorCinza();

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
    // Verde esquerda = 90°
    // --------------------------------------------------------

    if(esquerda)
    {
        reAntesDaCurva();

        iniciarGiro(
            GIRO_ESQUERDA
        );

        return;
    }


    // --------------------------------------------------------
    // Verde direita = 90°
    // --------------------------------------------------------

    if(direita)
    {
        reAntesDaCurva();

        iniciarGiro(
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

            controleGiro.curva180();

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
    controleGiro.update();


    if(
        !controleGiro.terminou()
    )
    {
        return;
    }


    acaoGiro =
        NENHUM_GIRO;


    controleLinha.start();

    entrarEstado(
        SEGUINDO_LINHA
    );
}


// ============================================================
// INICIAR CURVA PRETA
// ============================================================

void MaquinaEstados::iniciarCurvaPreta()
{
    controleLinha.stop();

    controleGiro.cancelar();

    motores.stop();


    // --------------------------------------------------------
    // Tendência negativa = curva para a direita.
    // --------------------------------------------------------

    if(
        tendenciaAntes < 0.0f
    )
    {
        motores.setSpeed(
            VELOCIDADE_CURVA_PRETA,
            VELOCIDADE_CURVA_PRETA,
            -VELOCIDADE_CURVA_PRETA,
            -VELOCIDADE_CURVA_PRETA
        );

        entrarEstado(
            CURVA_PRETA
        );

        return;
    }


    // --------------------------------------------------------
    // Tendência positiva = curva para a esquerda.
    // --------------------------------------------------------

    if(
        tendenciaAntes > 0.0f
    )
    {
        motores.setSpeed(
            -VELOCIDADE_CURVA_PRETA,
            -VELOCIDADE_CURVA_PRETA,
            VELOCIDADE_CURVA_PRETA,
            VELOCIDADE_CURVA_PRETA
        );

        entrarEstado(
            CURVA_PRETA
        );

        return;
    }


    // --------------------------------------------------------
    // Sem tendência confiável.
    // --------------------------------------------------------

    controleLinha.start();

    entrarEstado(
        SEGUINDO_LINHA
    );
}


// ============================================================
// PROCESSAR CURVA PRETA
// ============================================================
//
// Durante a curva preta NÃO usamos o giroscópio.
//
// O robô continua girando até que:
//
// sensor físico 4 OU sensor físico 5
//
// detecte preto.
//
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


    // --------------------------------------------------------
    // Encontrou a linha.
    // --------------------------------------------------------

    if(
        sensor4Preto ||
        sensor5Preto
    )
    {
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
    // Primeiro: ré para afastar do obstáculo.
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
    // Primeiro giro: SEMPRE para a direita.
    // --------------------------------------------------------

    controleGiro.curva90Direita();


    // --------------------------------------------------------
    // Durante esse giro não procuramos linha.
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // Nova reta = começa uma nova confirmação.
    // --------------------------------------------------------

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

            // ------------------------------------------------
            // Não procuramos preto durante o giro.
            // ------------------------------------------------

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
// LINHA ENCONTRADA - BUSCA NORMAL
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
// PARAR POR CINZA
// ============================================================

void MaquinaEstados::pararPorCinza()
{
    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.parar();

    motores.stop();


    entrarEstado(
        STOP_CINZA
    );
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
        estado == STOP_CINZA ||
        estado == STOP_VERMELHO
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


// ============================================================
// PARADO POR CINZA
// ============================================================

bool MaquinaEstados::paradoPorCinza() const
{
    return (
        estado ==
        STOP_CINZA
    );
}