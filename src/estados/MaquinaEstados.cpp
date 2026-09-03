#include "MaquinaEstados.h"


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


    // --------------------------------------------------------
    // Lado do obstáculo
    //
    // Direita como configuração inicial.
    // Pode ser alterado pelo setLadoObstaculo().
    // --------------------------------------------------------

    ladoObstaculo = 1;

    ladoObstaculoDefinido = true;


    // --------------------------------------------------------
    // Tempos iniciais do contorno
    // --------------------------------------------------------

    tempoRetoObstaculo1 = 450;

    tempoRetoObstaculo2 = 650;

    tempoMaximoRetoObstaculo3 = 2000;

    inicioTrechoObstaculo = 0;
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

        case INICIO:

            controleLinha.start();

            entrarEstado(SEGUINDO_LINHA);

            break;


        case SEGUINDO_LINHA:

            processarSeguindoLinha(linha);

            break;


        case AVALIANDO_MARCA:

            processarAvaliacao(
                linha,
                dadosCorEsquerda,
                dadosCorDireita
            );

            break;


        case EXECUTANDO_GIRO:

            processarGiro(linha);

            break;


        case BUSCANDO_LINHA:

            if(linhaEncontrada(linha))
            {
                controleLinha.start();

                entrarEstado(SEGUINDO_LINHA);
            }

            break;


        case OBSTACULO_GIRO_1:
        case OBSTACULO_RETO_1:
        case OBSTACULO_GIRO_2:
        case OBSTACULO_RETO_2:
        case OBSTACULO_GIRO_3:
        case OBSTACULO_RETO_3:
        case OBSTACULO_GIRO_4:
        case OBSTACULO_BUSCANDO_LINHA:

            processarObstaculo(linha);

            break;


        case STOP_CINZA:

            motores.stop();

            break;


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
    // Obstáculo possui prioridade.
    // --------------------------------------------------------

    if(analiseTOF.temObstaculo())
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


    // --------------------------------------------------------
    // Guarda tendência antes da região especial.
    // --------------------------------------------------------

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
    // --------------------------------------------------------
    // Avanço lento durante avaliação.
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // Saiu da região larga.
    // --------------------------------------------------------

    if(
        tempo >= tempoMinimoAvaliacao &&
        quantidadePretos < 4
    )
    {
        finalizarAvaliacao(linha);

        return;
    }


    // --------------------------------------------------------
    // Proteção.
    // --------------------------------------------------------

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


    if(esquerda)
    {
        iniciarGiro(
            GIRO_ESQUERDA
        );

        return;
    }


    if(direita)
    {
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
        // Mantido conforme a versão que você confirmou
        // funcionando no robô.

        if(tendenciaAntes < 0.0f)
        {
            iniciarGiro(
                GIRO_ESQUERDA
            );
        }
        else if(tendenciaAntes > 0.0f)
        {
            iniciarGiro(
                GIRO_DIREITA
            );
        }
        else
        {
            controleLinha.start();

            entrarEstado(
                SEGUINDO_LINHA
            );
        }

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
// INICIAR GIRO
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
// PROCESSAR GIRO
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
// INICIAR OBSTÁCULO
// ============================================================

void MaquinaEstados::iniciarObstaculo()
{
    controleLinha.stop();

    motores.stop();


    if(!ladoObstaculoDefinido)
    {
        return;
    }


    // --------------------------------------------------------
    // Primeira lateralização
    // --------------------------------------------------------

    if(
        ladoObstaculo < 0
    )
    {
        controleGiro.curva90Esquerda();
    }
    else
    {
        controleGiro.curva90Direita();
    }


    entrarEstado(
        OBSTACULO_GIRO_1
    );
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
        // GIRO 1
        // ====================================================

        case OBSTACULO_GIRO_1:

            controleGiro.update();

            if(
                controleGiro.terminou()
            )
            {
                controleObstaculo.iniciarReto();

                inicioTrechoObstaculo =
                    millis();

                entrarEstado(
                    OBSTACULO_RETO_1
                );
            }

            break;


        // ====================================================
        // RETO 1
        //
        // Afasta o robô lateralmente do obstáculo.
        // ====================================================

        case OBSTACULO_RETO_1:

            controleObstaculo.update();

            if(
                millis() -
                inicioTrechoObstaculo
                >=
                tempoRetoObstaculo1
            )
            {
                controleObstaculo.parar();


                // Lado oposto ao primeiro giro.

                if(
                    ladoObstaculo < 0
                )
                {
                    controleGiro.curva90Direita();
                }
                else
                {
                    controleGiro.curva90Esquerda();
                }


                entrarEstado(
                    OBSTACULO_GIRO_2
                );
            }

            break;


        // ====================================================
        // GIRO 2
        // ====================================================

        case OBSTACULO_GIRO_2:

            controleGiro.update();

            if(
                controleGiro.terminou()
            )
            {
                controleObstaculo.iniciarReto();

                inicioTrechoObstaculo =
                    millis();

                entrarEstado(
                    OBSTACULO_RETO_2
                );
            }

            break;


        // ====================================================
        // RETO 2
        //
        // Passa além do comprimento do obstáculo.
        // ====================================================

        case OBSTACULO_RETO_2:

            controleObstaculo.update();

            if(
                millis() -
                inicioTrechoObstaculo
                >=
                tempoRetoObstaculo2
            )
            {
                controleObstaculo.parar();


                // Continua contornando para o mesmo lado
                // da segunda mudança.

                if(
                    ladoObstaculo < 0
                )
                {
                    controleGiro.curva90Direita();
                }
                else
                {
                    controleGiro.curva90Esquerda();
                }


                entrarEstado(
                    OBSTACULO_GIRO_3
                );
            }

            break;


        // ====================================================
        // GIRO 3
        // ====================================================

        case OBSTACULO_GIRO_3:

            controleGiro.update();

            if(
                controleGiro.terminou()
            )
            {
                controleObstaculo.iniciarReto();

                inicioTrechoObstaculo =
                    millis();

                entrarEstado(
                    OBSTACULO_RETO_3
                );
            }

            break;


        // ====================================================
        // RETO 3
        //
        // Volta lateralmente até reencontrar a linha.
        // ====================================================

        case OBSTACULO_RETO_3:

            controleObstaculo.update();


            if(
                linhaEncontrada(linha)
            )
            {
                controleObstaculo.parar();


                // Último giro para alinhar com a linha.

                if(
                    ladoObstaculo < 0
                )
                {
                    controleGiro.curva90Esquerda();
                }
                else
                {
                    controleGiro.curva90Direita();
                }


                entrarEstado(
                    OBSTACULO_GIRO_4
                );

                break;
            }


            // ------------------------------------------------
            // Segurança.
            // ------------------------------------------------

            if(
                millis() -
                inicioTrechoObstaculo
                >=
                tempoMaximoRetoObstaculo3
            )
            {
                controleObstaculo.parar();


                if(
                    ladoObstaculo < 0
                )
                {
                    controleGiro.curva90Esquerda();
                }
                else
                {
                    controleGiro.curva90Direita();
                }


                entrarEstado(
                    OBSTACULO_GIRO_4
                );
            }

            break;


        // ====================================================
        // GIRO 4
        // ====================================================

        case OBSTACULO_GIRO_4:

            controleGiro.update();

            if(
                controleGiro.terminou()
            )
            {
                controleObstaculo.iniciarReto();

                entrarEstado(
                    OBSTACULO_BUSCANDO_LINHA
                );
            }

            break;


        // ====================================================
        // BUSCAR LINHA
        // ====================================================

        case OBSTACULO_BUSCANDO_LINHA:

            controleObstaculo.update();


            if(
                linhaEncontrada(linha)
            )
            {
                controleObstaculo.parar();

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
// INICIAR BUSCA
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
    if(!linha.linhaDetectada)
        return false;

    if(
        linha.intensidade < 0.15f
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
    for(int i = 0; i < 8; i++)
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
// CONTAR PRETOS
// ============================================================

int MaquinaEstados::contarSensoresPretos(
    const LinhaData& linha
)
{
    int quantidade = 0;

    for(int i = 0; i < 8; i++)
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
// CONFIGURAR LADO DO OBSTÁCULO
// ============================================================

void MaquinaEstados::setLadoObstaculo(
    int lado
)
{
    if(lado == 0)
    {
        ladoObstaculoDefinido = false;

        return;
    }


    if(lado < 0)
        ladoObstaculo = -1;
    else
        ladoObstaculo = 1;


    ladoObstaculoDefinido = true;
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
    return estado ==
           STOP_VERMELHO;
}


// ============================================================
// PARADO POR CINZA
// ============================================================

bool MaquinaEstados::paradoPorCinza() const
{
    return estado ==
           STOP_CINZA;
}