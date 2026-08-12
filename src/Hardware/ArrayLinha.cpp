#include "ArrayLinha.h"




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



    dados =
    {
        0,
        0,
        0,
        0,

        0,
        0,
        0,
        0
    };

}




void ArrayLinha::begin()
{

    serial.begin(
        baud,
        SERIAL_8N1,
        rxPin,
        txPin
    );


    delay(300);



    // Solicita ao módulo Yahboom
    // enviar somente valores analógicos

    serial.print("$0,1,0#");


}





void ArrayLinha::update()
{


    while(serial.available())
    {


        char c = (char)serial.read();



        // Ignora quebra de linha

        if(c == '\r' || c == '\n')
        {
            continue;
        }




        // Final do pacote

        if(c == '#')
        {

            buffer[indice] = '\0';



            if(indice > 0)
            {
                processarPacote();
            }



            indice = 0;

        }




        else
        {

            // Proteção contra estouro

            if(indice < sizeof(buffer)-1)
            {

                buffer[indice++] = c;

            }

            else
            {

                indice = 0;

            }

        }


    }


}





void ArrayLinha::processarPacote()
{


    int lidos = sscanf(

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



    // Se o pacote vier errado,
    // mantém o último valor válido

    if(lidos != 8)
    {
        return;
    }


}





ArrayData ArrayLinha::getData()
{

    return dados;

}