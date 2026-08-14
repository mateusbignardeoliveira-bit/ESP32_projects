#include "ArrayLinha.h"


// ============================================================
// CONSTRUTOR
// ============================================================

ArrayLinha::ArrayLinha(
    HardwareSerial& portaSerial,
    int RX,
    int TX,
    int velocidade
)
:
serial(portaSerial)
{

    rxPin = RX;

    txPin = TX;

    baud = velocidade;


    indice = 0;


    // ========================================================
    // Inicializa dados
    // ========================================================

    dados =
    {
        // Analógico
        0, 0, 0, 0,
        0, 0, 0, 0,

        // Digital
        0, 0, 0, 0,
        0, 0, 0, 0
    };

}



// ============================================================
// BEGIN
// ============================================================

void ArrayLinha::begin()
{

    serial.begin(
        baud,
        SERIAL_8N1,
        rxPin,
        txPin
    );


    delay(300);


    // ========================================================
    // Solicita:
    //
    // $0,1,1#
    //
    // Analógico + Digital
    // ========================================================

    serial.print("$0,1,1#");

}



// ============================================================
// UPDATE
// ============================================================

void ArrayLinha::update()
{

    while(serial.available())
    {

        char c =
            (char)serial.read();


        // ----------------------------------------------------
        // Ignora quebra de linha
        // ----------------------------------------------------

        if(c == '\r' || c == '\n')
        {
            continue;
        }


        // ----------------------------------------------------
        // Fim do pacote
        // ----------------------------------------------------

        if(c == '#')
        {

            buffer[indice] =
                '\0';


            if(indice > 0)
            {
                processarPacote();
            }


            indice = 0;

        }


        // ----------------------------------------------------
        // Continua recebendo pacote
        // ----------------------------------------------------

        else
        {

            if(indice < sizeof(buffer) - 1)
            {

                buffer[indice++] =
                    c;

            }

            else
            {

                // Estouro do buffer
                // descarta pacote atual

                indice = 0;

            }

        }

    }

}



// ============================================================
// PROCESSA PACOTE
// ============================================================

void ArrayLinha::processarPacote()
{

    // ========================================================
    // Verifica se é pacote ANALÓGICO
    //
    // $A,x1:123,x2:456,...#
    // ========================================================

    if(buffer[0] == '$' &&
       buffer[1] == 'A')
    {

        processarAnalogico();

        return;

    }


    // ========================================================
    // Verifica se é pacote DIGITAL
    //
    // $D,x1:0,x2:1,...#
    // ========================================================

    if(buffer[0] == '$' &&
       buffer[1] == 'D')
    {

        processarDigital();

        return;

    }

}



// ============================================================
// PROCESSA ANALÓGICO
// ============================================================

void ArrayLinha::processarAnalogico()
{

    int lidos =
        sscanf(

            buffer,

            "$A,x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d",

            &dados.s1,
            &dados.s2,
            &dados.s3,
            &dados.s4,

            &dados.s5,
            &dados.s6,
            &dados.s7,
            &dados.s8

        );


    // ========================================================
    // Se pacote estiver errado,
    // mantém último valor válido.
    // ========================================================

    if(lidos != 8)
    {
        return;
    }

}



// ============================================================
// PROCESSA DIGITAL
// ============================================================

void ArrayLinha::processarDigital()
{

    int d[8];


    int lidos =
        sscanf(

            buffer,

            "$D,x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d",

            &d[0],
            &d[1],
            &d[2],
            &d[3],

            &d[4],
            &d[5],
            &d[6],
            &d[7]

        );


    // ========================================================
    // Pacote inválido
    // ========================================================

    if(lidos != 8)
    {
        return;
    }


    // ========================================================
    // Salva valores digitais
    // ========================================================

    dados.d1 = d[0];
    dados.d2 = d[1];
    dados.d3 = d[2];
    dados.d4 = d[3];

    dados.d5 = d[4];
    dados.d6 = d[5];
    dados.d7 = d[6];
    dados.d8 = d[7];

}



// ============================================================
// GET DATA
// ============================================================

ArrayData ArrayLinha::getData()
{

    return dados;

}