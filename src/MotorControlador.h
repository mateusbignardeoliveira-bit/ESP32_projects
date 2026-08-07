#ifndef MOTORCONTROLADOR_H
#define MOTORCONTROLADOR_H


#include <Arduino.h>



class MotorControlador
{


private:


    HardwareSerial& serial;


    int rxPin;

    int txPin;

    int baud;



    void enviarComando(String comando);



public:


    MotorControlador(
        HardwareSerial& portaSerial,
        int RX,
        int TX,
        int velocidade = 115200
    );



    void begin();



    void setSpeed(
        int m1,
        int m2,
        int m3,
        int m4
    );



    bool available();



    String readData();



};



#endif