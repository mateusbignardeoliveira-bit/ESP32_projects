#include "Verde.h"


// ============================================================
// CONFIGURAÇÕES
// ============================================================

namespace
{
    // --------------------------------------------------------
    // VELOCIDADE NORMAL
    // --------------------------------------------------------

    constexpr int VELOCIDADE_ROBO =
        400;


    // --------------------------------------------------------
    // VELOCIDADE DURANTE A BUSCA DO VERDE
    //
    // O robô anda devagar enquanto continua fazendo
    // medições dos sensores de cor e do array.
    // --------------------------------------------------------

    constexpr int VELOCIDADE_VERDE =
        60;


    // --------------------------------------------------------
    // VELOCIDADE DO AVANÇO APÓS DETECTAR LINHA
    //
    // Se durante a busca do verde forem encontrados
    // 4 ou mais pretos, o robô vem para cá.
    // --------------------------------------------------------

    constexpr int VELOCIDADE_AVANCO_LINHA =
        400;


    // --------------------------------------------------------
    // TEMPO DO AVANÇO APÓS DETECTAR LINHA
    //
    // Este é o "pequeno avanço" que você queria.
    //
    // 300 ms
    // --------------------------------------------------------

    constexpr unsigned long TEMPO_AVANCO_LINHA_MS =
        300;


    // --------------------------------------------------------
    // VELOCIDADE DA CURVA
    // --------------------------------------------------------

    constexpr int VELOCIDADE_CURVA =
        400;


    // --------------------------------------------------------
    // FREIO
    // --------------------------------------------------------

    constexpr int VELOCIDADE_FREIO =
        -1000;


    constexpr unsigned long TEMPO_FREIO_MS =
        20;


    // --------------------------------------------------------
    // TEMPO DA CURVA DE 90°
    // --------------------------------------------------------

    constexpr unsigned long TEMPO_CURVA_90_MS =
        700;


    // --------------------------------------------------------
    // TEMPO DA MEIA VOLTA
    // --------------------------------------------------------

    constexpr unsigned long TEMPO_MEIA_VOLTA_MS =
        1400;


    // --------------------------------------------------------
    // QUANTIDADE DE LEITURAS SEM VERDE
    //
    // O verde precisa desaparecer por 35 leituras consecutivas
    // para considerarmos que o robô saiu da região verde.
    // --------------------------------------------------------

    constexpr int LEITURAS_SEM_VERDE_NECESSARIAS =
        40;


    // --------------------------------------------------------
    // LIMITE PARA CONSIDERAR QUE EXISTE LINHA
    //
    // 4 ou mais pretos:
    //
    //      NÃO faz curva verde.
    //
    // 3 ou menos:
    //
    //      pode executar a manobra correspondente ao verde.
    // --------------------------------------------------------

    constexpr int MINIMO_PRETOS_LINHA =
        6;
}


// ============================================================
// CONSTRUTOR
// ============================================================

Verde::Verde(
    MotorControlador& motores,
    AS7341Analise& analise
)
    :
    motores(motores),
    analise(analise),

    estado(NORMAL),

    verdeEsquerda(false),
    verdeDireita(false),

    maiorQuantidadePretos(0),

    tempoInicio(0),

    leiturasSemVerde(0)
{
}


// ============================================================
// PARAR
// ============================================================

void Verde::parar()
{
    motores.stop();
}


// ============================================================
// ANDAR NORMAL
// ============================================================

void Verde::andarNormal()
{
    motores.setSpeed(
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO,
        VELOCIDADE_ROBO
    );
}


// ============================================================
// ANDAR DEVAGAR
// ============================================================

void Verde::andarDevagar()
{
    motores.setSpeed(
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE,
        VELOCIDADE_VERDE
    );
}


// ============================================================
// ANDAR RETO
// ============================================================

void Verde::andarReto()
{
    motores.setSpeed(
        VELOCIDADE_AVANCO_LINHA,
        VELOCIDADE_AVANCO_LINHA,
        VELOCIDADE_AVANCO_LINHA,
        VELOCIDADE_AVANCO_LINHA
    );
}


// ============================================================
// FREAR
// ============================================================

void Verde::frear()
{
    motores.setSpeed(
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO,
        VELOCIDADE_FREIO
    );
}


// ============================================================
// CURVA ESQUERDA
// ============================================================

void Verde::curvaEsquerda()
{
    motores.setSpeed(
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA
    );
}


// ============================================================
// CURVA DIREITA
// ============================================================

void Verde::curvaDireita()
{
    motores.setSpeed(
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// MEIA VOLTA
// ============================================================

void Verde::meiaVolta()
{
    motores.setSpeed(
         VELOCIDADE_CURVA,
         VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA,
        -VELOCIDADE_CURVA
    );
}


// ============================================================
// INICIA AVANÇO APÓS LINHA
// ============================================================
//
// Aqui chegamos quando:
//
// maiorQuantidadePretos >= 4
//
// O robô simplesmente avança reto por 300 ms.
//
// Depois disso o estado vira FINALIZADO.
//
// O Main então devolve o controle para o PID.
// ============================================================

void Verde::iniciarAvancoLinha()
{
    Serial.println(
        "LINHA DETECTADA DURANTE VERDE"
    );


    Serial.print(
        "MAIOR QUANTIDADE DE PRETOS: "
    );


    Serial.println(
        maiorQuantidadePretos
    );


    Serial.println(
        "AVANCANDO 300 MS ANTES DO PID"
    );


    tempoInicio =
        millis();


    estado =
        AVANCO_LINHA;
}


// ============================================================
// INICIA CURVA ESQUERDA
// ============================================================

void Verde::iniciarCurvaEsquerda()
{
    Serial.println(
        "CURVA VERDE ESQUERDA"
    );


    tempoInicio =
        millis();


    estado =
        CURVA_ESQUERDA;
}


// ============================================================
// INICIA CURVA DIREITA
// ============================================================

void Verde::iniciarCurvaDireita()
{
    Serial.println(
        "CURVA VERDE DIREITA"
    );


    tempoInicio =
        millis();


    estado =
        CURVA_DIREITA;
}


// ============================================================
// INICIA MEIA VOLTA
// ============================================================

void Verde::iniciarMeiaVolta()
{
    Serial.println(
        "MEIA VOLTA VERDE"
    );


    tempoInicio =
        millis();


    estado =
        MEIA_VOLTA;
}


// ============================================================
// FINALIZA
// ============================================================

void Verde::finalizar()
{
    parar();


    estado =
        FINALIZADO;
}


// ============================================================
// UPDATE
// ============================================================

void Verde::update(
    const AS7341Data& dadosEsquerda,
    const AS7341Data& dadosDireita,
    int quantidadePretos
)
{
    // ========================================================
    // ANALISA OS DOIS SENSORES DE COR
    // ========================================================

    AS7341Resultado esquerda =
        analise.analisar(
            dadosEsquerda
        );


    AS7341Resultado direita =
        analise.analisar(
            dadosDireita
        );


    // ========================================================
    // NORMAL
    // ========================================================

    if(
        estado ==
        NORMAL
    )
    {
        // ----------------------------------------------------
        // DETECTOU VERDE
        // ----------------------------------------------------

        if(
            esquerda.verdeDetectado ||
            direita.verdeDetectado
        )
        {
            Serial.println(
                "VERDE DETECTADO"
            );


            // ------------------------------------------------
            // GUARDA O LADO DO VERDE
            // ------------------------------------------------

            if(
                esquerda.verdeDetectado
            )
            {
                verdeEsquerda =
                    true;
            }


            if(
                direita.verdeDetectado
            )
            {
                verdeDireita =
                    true;
            }


            // ------------------------------------------------
            // NOVA DETECÇÃO
            //
            // Começamos uma nova análise.
            // Portanto o maior número de pretos precisa
            // começar novamente em zero.
            // ------------------------------------------------

            maiorQuantidadePretos =
                quantidadePretos;


            leiturasSemVerde =
                0;


            // ------------------------------------------------
            // FREIA
            // ------------------------------------------------

            frear();


            tempoInicio =
                millis();


            estado =
                FREANDO;


            return;
        }


        // ----------------------------------------------------
        // NENHUM VERDE
        // ----------------------------------------------------

        andarNormal();


        return;
    }


    // ========================================================
    // FREANDO
    // ========================================================

    if(
        estado ==
        FREANDO
    )
    {
        frear();


        if(
            millis() -
            tempoInicio >=
            TEMPO_FREIO_MS
        )
        {
            Serial.println(
                "FREIO VERDE TERMINADO"
            );


            parar();


            leiturasSemVerde =
                0;


            // ------------------------------------------------
            // IMPORTANTE:
            //
            // O valor inicial continua sendo a leitura que
            // ocorreu no momento da detecção.
            // ------------------------------------------------

            tempoInicio =
                millis();


            estado =
                AVANCANDO;
        }


        return;
    }


    // ========================================================
    // AVANÇANDO PELO VERDE
    // ========================================================
    //
    // AQUI ESTÁ A PRINCIPAL MUDANÇA.
    //
    // Enquanto o robô anda devagar:
    //
    // 1. continua procurando verde;
    // 2. guarda esquerda/direita;
    // 3. olha o array;
    // 4. guarda o MAIOR número de pretos encontrado.
    //
    // Não importa se em uma leitura apareceram 2 pretos,
    // depois 3, depois 1 e depois 5.
    //
    // O resultado será:
    //
    // maiorQuantidadePretos = 5
    //
    // ========================================================

    if(
        estado ==
        AVANCANDO
    )
    {
        // ----------------------------------------------------
        // ANDA DEVAGAR
        // ----------------------------------------------------

        andarDevagar();


        // ----------------------------------------------------
        // GUARDA VERDE ESQUERDO
        // ----------------------------------------------------

        if(
            esquerda.verdeDetectado
        )
        {
            verdeEsquerda =
                true;
        }


        // ----------------------------------------------------
        // GUARDA VERDE DIREITO
        // ----------------------------------------------------

        if(
            direita.verdeDetectado
        )
        {
            verdeDireita =
                true;
        }


        // ====================================================
        // GUARDA O MAIOR NÚMERO DE PRETOS
        // ====================================================

        if(
            quantidadePretos >
            maiorQuantidadePretos
        )
        {
            maiorQuantidadePretos =
                quantidadePretos;


            Serial.print(
                "NOVO MAIOR NUMERO DE PRETOS: "
            );


            Serial.println(
                maiorQuantidadePretos
            );
        }


        // ====================================================
        // AINDA ESTÁ VENDO VERDE
        // ====================================================

        if(
            esquerda.verdeDetectado ||
            direita.verdeDetectado
        )
        {
            leiturasSemVerde =
                0;


            return;
        }


        // ====================================================
        // NÃO ESTÁ VENDO VERDE
        // ====================================================

        leiturasSemVerde++;


        // ----------------------------------------------------
        // Ainda não confirmou saída do verde.
        // ----------------------------------------------------

        if(
            leiturasSemVerde <
            LEITURAS_SEM_VERDE_NECESSARIAS
        )
        {
            return;
        }


        // ====================================================
        // SAIU DO VERDE
        // ====================================================

        Serial.println(
            "SAIU DO VERDE"
        );


        Serial.print(
            "MAIOR NUMERO DE PRETOS VISTO: "
        );


        Serial.println(
            maiorQuantidadePretos
        );


        Serial.print(
            "VERDE ESQUERDA: "
        );


        Serial.println(
            verdeEsquerda
        );


        Serial.print(
            "VERDE DIREITA: "
        );


        Serial.println(
            verdeDireita
        );


        // ====================================================
        // DECISÃO FINAL
        // ====================================================
        //
        // IMPORTANTE:
        //
        // NÃO usamos mais a quantidadePretos atual.
        //
        // Usamos:
        //
        // maiorQuantidadePretos
        //
        // que representa o maior número observado durante
        // todo o avanço procurando pelo verde.
        // ====================================================

        if(
            maiorQuantidadePretos >=
            MINIMO_PRETOS_LINHA
        )
        {
            // ------------------------------------------------
            // 4 OU MAIS PRETOS
            //
            // NÃO É CURVA VERDE.
            //
            // Avança 300 ms e depois termina.
            // ------------------------------------------------

            iniciarAvancoLinha();


            return;
        }


        // ====================================================
        // MENOS DE 4 PRETOS
        //
        // Agora sim usamos os verdes encontrados.
        // ====================================================

        // ----------------------------------------------------
        // VERDE SOMENTE À ESQUERDA
        // ----------------------------------------------------

        if(
            verdeEsquerda &&
            !verdeDireita
        )
        {
            iniciarCurvaEsquerda();


            return;
        }


        // ----------------------------------------------------
        // VERDE SOMENTE À DIREITA
        // ----------------------------------------------------

        if(
            verdeDireita &&
            !verdeEsquerda
        )
        {
            iniciarCurvaDireita();


            return;
        }


        // ----------------------------------------------------
        // VERDE DOS DOIS LADOS
        //
        // MEIA VOLTA
        // ----------------------------------------------------

        if(
            verdeEsquerda &&
            verdeDireita
        )
        {
            iniciarMeiaVolta();


            return;
        }


        // ----------------------------------------------------
        // ERRO
        // ----------------------------------------------------

        Serial.println(
            "NENHUM VERDE REGISTRADO - ERRO"
        );


        finalizar();


        return;
    }


    // ========================================================
    // AVANÇO APÓS DETECTAR LINHA
    // ========================================================
    //
    // Este estado só acontece quando:
    //
    // maiorQuantidadePretos >= 4
    //
    // Aqui o robô não usa PID.
    //
    // Ele simplesmente vai reto durante 300 ms.
    //
    // Depois:
    //
    //     FINALIZADO
    //
    // O Main então devolve o controle ao PID.
    // ========================================================

    if(
        estado ==
        AVANCO_LINHA
    )
    {
        andarReto();


        if(
            millis() -
            tempoInicio >=
            TEMPO_AVANCO_LINHA_MS
        )
        {
            parar();


            Serial.println(
                "AVANCO DE LINHA TERMINADO"
            );


            Serial.println(
                "DEVOLVENDO CONTROLE AO PID"
            );


            estado =
                FINALIZADO;
        }


        return;
    }


    // ========================================================
    // CURVA ESQUERDA
    // ========================================================

    if(
        estado ==
        CURVA_ESQUERDA
    )
    {
        curvaEsquerda();


        if(
            millis() -
            tempoInicio >=
            TEMPO_CURVA_90_MS
        )
        {
            parar();


            Serial.println(
                "CURVA VERDE ESQUERDA TERMINADA"
            );


            estado =
                FINALIZADO;
        }


        return;
    }


    // ========================================================
    // CURVA DIREITA
    // ========================================================

    if(
        estado ==
        CURVA_DIREITA
    )
    {
        curvaDireita();


        if(
            millis() -
            tempoInicio >=
            TEMPO_CURVA_90_MS
        )
        {
            parar();


            Serial.println(
                "CURVA VERDE DIREITA TERMINADA"
            );


            estado =
                FINALIZADO;
        }


        return;
    }


    // ========================================================
    // MEIA VOLTA
    // ========================================================

    if(
        estado ==
        MEIA_VOLTA
    )
    {
        meiaVolta();


        if(
            millis() -
            tempoInicio >=
            TEMPO_MEIA_VOLTA_MS
        )
        {
            parar();


            Serial.println(
                "MEIA VOLTA VERDE TERMINADA"
            );


            estado =
                FINALIZADO;
        }


        return;
    }


    // ========================================================
    // FINALIZADO
    // ========================================================

    if(
        estado ==
        FINALIZADO
    )
    {
        parar();


        return;
    }
}


// ============================================================
// RESET
// ============================================================

void Verde::reset()
{
    parar();


    estado =
        NORMAL;


    verdeEsquerda =
        false;


    verdeDireita =
        false;


    maiorQuantidadePretos =
        0;


    tempoInicio =
        0;


    leiturasSemVerde =
        0;
}


// ============================================================
// ESTÁ EXECUTANDO?
// ============================================================

bool Verde::estaExecutando() const
{
    return (
        estado != NORMAL &&
        estado != FINALIZADO
    );
}


// ============================================================
// FINALIZADO?
// ============================================================

bool Verde::finalizado() const
{
    return (
        estado ==
        FINALIZADO
    );
}


// ============================================================
// GET ESTADO
// ============================================================

Verde::Estado Verde::getEstado() const
{
    return estado;
}


// ============================================================
// VERDE ESQUERDA
// ============================================================

bool Verde::detectouVerdeEsquerda() const
{
    return verdeEsquerda;
}


// ============================================================
// VERDE DIREITA
// ============================================================

bool Verde::detectouVerdeDireita() const
{
    return verdeDireita;
}