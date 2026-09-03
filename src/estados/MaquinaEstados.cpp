#include "MaquinaEstados.h"


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

    ladoObstaculo = 0;

    ladoObstaculoDefinido = false;
}


void MaquinaEstados::begin()
{
    estado = INICIO;

    acaoGiro = NENHUM_GIRO;

    tendenciaAntes = 0.0f;

    inicioAvaliacao = 0;

    inicioBusca = 0;

    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.cancelar();

    analiseVerde.reset();
}


void MaquinaEstados::resetExecucao()
{
    // --------------------------------------------------------
    // IMPORTANTE:
    //
    // Este método NÃO mexe na IMU.
    //
    // Não calibra.
    // Não zera heading.
    //
    // Apenas limpa tudo que pertence à execução do robô.
    // --------------------------------------------------------

    estado = INICIO;

    acaoGiro = NENHUM_GIRO;

    tendenciaAntes = 0.0f;

    inicioAvaliacao = 0;

    inicioBusca = 0;


    // --------------------------------------------------------
    // Limpa PID / controle de linha
    // --------------------------------------------------------

    controleLinha.stop();


    // --------------------------------------------------------
    // Cancela qualquer giro em andamento
    // --------------------------------------------------------

    controleGiro.cancelar();


    // --------------------------------------------------------
    // Cancela qualquer movimento de obstáculo
    // --------------------------------------------------------

    controleObstaculo.cancelar();


    // --------------------------------------------------------
    // Limpa toda a memória da avaliação de cores
    // --------------------------------------------------------

    analiseVerde.reset();


    // --------------------------------------------------------
    // Garante motores parados.
    // --------------------------------------------------------

    motores.stop();

    delay(100);

    motores.release();
}


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
        case OBSTACULO_BUSCANDO_LINHA:

            processarObstaculo(linha);

            break;


        case STOP_PERMANENTE:

            motores.stop();

            break;
    }
}


void MaquinaEstados::processarSeguindoLinha(
    const LinhaData& linha
)
{
    // Obstáculo possui prioridade.
    if(analiseTOF.temObstaculo())
    {
        iniciarObstaculo();

        return;
    }


    // Quatro ou mais sensores pretos:
    // região especial / marca.
    if(contarSensoresPretos(linha) >= 4)
    {
        iniciarAvaliacao(linha);

        return;
    }


    controleLinha.update(linha);
}


void MaquinaEstados::iniciarAvaliacao(
    const LinhaData& linha
)
{
    controleLinha.stop();

    motores.stop();


    // --------------------------------------------------------
    // Guarda o lado da linha antes de entrar na região especial.
    //
    // posição negativa = esquerda
    // posição positiva = direita
    // --------------------------------------------------------

    tendenciaAntes = linha.posicao;


    int quantidadePretos =
        contarSensoresPretos(linha);


    analiseVerde.iniciar(
        quantidadePretos
    );


    inicioAvaliacao = millis();


    entrarEstado(AVALIANDO_MARCA);
}


void MaquinaEstados::processarAvaliacao(
    const LinhaData& linha,
    const AS7341Data& dadosCorEsquerda,
    const AS7341Data& dadosCorDireita
)
{
    // Avanço lento durante avaliação.

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
        millis() - inicioAvaliacao;


    // Depois do tempo mínimo, quando deixou a região larga,
    // podemos concluir a avaliação.

    if(
        tempo >= tempoMinimoAvaliacao &&
        quantidadePretos < 4
    )
    {
        finalizarAvaliacao(linha);

        return;
    }


    // Segurança contra ficar preso na avaliação.

    if(tempo >= tempoMaximoAvaliacao)
    {
        finalizarAvaliacao(linha);
    }
}


void MaquinaEstados::finalizarAvaliacao(
    const LinhaData& linha
)
{
    motores.stop();

    analiseVerde.finalizar();


    // --------------------------------------------------------
    // Vermelho ou cinza:
    // parada permanente.
    // --------------------------------------------------------

    if(
        analiseVerde.detectouVermelho() ||
        analiseVerde.detectouCinza()
    )
    {
        pararPermanentemente();

        return;
    }


    // --------------------------------------------------------
    // Verde.
    //
    // Ambos os lados, mesmo que detectados em momentos
    // diferentes, significam 180 graus.
    // --------------------------------------------------------

    bool esquerda =
        analiseVerde.detectouVerdeEsquerda();

    bool direita =
        analiseVerde.detectouVerdeDireita();


    if(
        esquerda &&
        direita
    )
    {
        iniciarGiro(GIRO_180);

        return;
    }


    if(esquerda)
    {
        iniciarGiro(GIRO_ESQUERDA);

        return;
    }


    if(direita)
    {
        iniciarGiro(GIRO_DIREITA);

        return;
    }


    // --------------------------------------------------------
    // Nenhuma cor relevante.
    //
    // Todos brancos:
    // curva para o lado que estava sendo seguido antes.
    // --------------------------------------------------------

    if(todosBrancos(linha))
    {
        if(tendenciaAntes < 0.0f)
        {
            iniciarGiro(GIRO_DIREITA);
        }
        else if(tendenciaAntes > 0.0f)
        {
            iniciarGiro(GIRO_ESQUERDA);
        }
        else
        {
            controleLinha.start();

            entrarEstado(SEGUINDO_LINHA);
        }

        return;
    }


    // --------------------------------------------------------
    // Ainda existe linha:
    // falsa curva -> continua seguindo.
    // --------------------------------------------------------

    controleLinha.start();

    entrarEstado(SEGUINDO_LINHA);
}


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

            entrarEstado(SEGUINDO_LINHA);

            return;
    }


    entrarEstado(EXECUTANDO_GIRO);
}


void MaquinaEstados::processarGiro(
    const LinhaData& linha
)
{
    controleGiro.update();

    if(!controleGiro.terminou())
        return;


    acaoGiro = NENHUM_GIRO;

    controleLinha.start();

    entrarEstado(SEGUINDO_LINHA);
}


void MaquinaEstados::iniciarBuscaLinha()
{
    inicioBusca = millis();

    entrarEstado(BUSCANDO_LINHA);
}


bool MaquinaEstados::linhaEncontrada(
    const LinhaData& linha
)
{
    if(!linha.linhaDetectada)
        return false;

    if(linha.intensidade < 0.15f)
        return false;


    return true;
}


bool MaquinaEstados::todosBrancos(
    const LinhaData& linha
)
{
    for(int i = 0; i < 8; i++)
    {
        if(linha.sensores[i] > 0.08f)
            return false;
    }

    return true;
}


int MaquinaEstados::contarSensoresPretos(
    const LinhaData& linha
)
{
    int quantidade = 0;

    for(int i = 0; i < 8; i++)
    {
        if(linha.sensores[i] >= 0.35f)
        {
            quantidade++;
        }
    }

    return quantidade;
}


void MaquinaEstados::iniciarObstaculo()
{
    controleLinha.stop();

    motores.stop();


    // O lado ainda será definido.

    if(!ladoObstaculoDefinido)
    {
        return;
    }


    if(ladoObstaculo < 0)
    {
        controleGiro.curva90Esquerda();

        entrarEstado(OBSTACULO_GIRO_1);
    }
    else
    {
        controleGiro.curva90Direita();

        entrarEstado(OBSTACULO_GIRO_1);
    }
}


void MaquinaEstados::processarObstaculo(
    const LinhaData& linha
)
{
    switch(estado)
    {

        case OBSTACULO_GIRO_1:

            controleGiro.update();

            if(controleGiro.terminou())
            {
                controleObstaculo.iniciarReto();

                entrarEstado(OBSTACULO_RETO_1);
            }

            break;


        case OBSTACULO_RETO_1:

            controleObstaculo.update();

            if(!analiseTOF.temObstaculo())
            {
                controleGiro.curva90Direita();

                entrarEstado(OBSTACULO_GIRO_2);
            }

            break;


        case OBSTACULO_GIRO_2:

            controleGiro.update();

            if(controleGiro.terminou())
            {
                controleObstaculo.iniciarReto();

                entrarEstado(OBSTACULO_RETO_2);
            }

            break;


        case OBSTACULO_RETO_2:

            controleObstaculo.update();

            if(linhaEncontrada(linha))
            {
                controleObstaculo.parar();

                controleGiro.curva90Esquerda();

                entrarEstado(OBSTACULO_GIRO_3);
            }

            break;


        case OBSTACULO_GIRO_3:

            controleGiro.update();

            if(controleGiro.terminou())
            {
                entrarEstado(OBSTACULO_BUSCANDO_LINHA);
            }

            break;


        case OBSTACULO_BUSCANDO_LINHA:

            if(linhaEncontrada(linha))
            {
                controleLinha.start();

                entrarEstado(SEGUINDO_LINHA);
            }

            break;


        default:

            break;
    }
}


void MaquinaEstados::entrarEstado(
    Estado novoEstado
)
{
    estado = novoEstado;
}


void MaquinaEstados::pararPermanentemente()
{
    controleLinha.stop();

    controleGiro.cancelar();

    controleObstaculo.parar();

    motores.stop();

    entrarEstado(STOP_PERMANENTE);
}


MaquinaEstados::Estado
MaquinaEstados::getEstado() const
{
    return estado;
}


bool MaquinaEstados::parado() const
{
    return estado == STOP_PERMANENTE;
}