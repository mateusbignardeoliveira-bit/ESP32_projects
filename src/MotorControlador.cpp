#include "MotorControlador.h"



MotorControlador::MotorControlador(
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

}




void MotorControlador::begin()
{


    serial.begin(
        baud,
        SERIAL_8N1,
        rxPin,
        txPin
    );



    delay(1000);



    // Configuração do driver


    enviarComando("$mtype:2#");


    delay(300);


    enviarComando("$mline:13#");


    delay(300);


    enviarComando("$mphase:20#");


    delay(300);


    enviarComando("$wdiameter:48#");


    delay(300);


    enviarComando("$deadzone:1600#");


    delay(300);


    enviarComando("$MPID:0.8,0.06,0.5#");


    delay(300);



    // Para os motores ao iniciar

    setSpeed(0,0,0,0);

}





void MotorControlador::enviarComando(String comando)
{

    serial.print(comando);

}





void MotorControlador::setSpeed(
    int m1,
    int m2,
    int m3,
    int m4
)
{

    String comando =
        "$spd:" +
        String(m1) + "," +
        String(m2) + "," +
        String(m3) + "," +
        String(m4) +
        "#";



    enviarComando(comando);

}





bool MotorControlador::available()
{

    return serial.available();

}





String MotorControlador::readData()
{

    String resposta = "";


    while(serial.available())
    {

        resposta += (char)serial.read();

    }


    return resposta;

}