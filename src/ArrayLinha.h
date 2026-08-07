#ifndef ARRAYLINHA_H
#define ARRAYLINHA_H


#include <Arduino.h>



struct ArrayData
{

    int s1;
    int s2;
    int s3;
    int s4;

    int s5;
    int s6;
    int s7;
    int s8;

};



class ArrayLinha
{

private:


    HardwareSerial& serial;


    int rxPin;

    int txPin;

    int baud;



    char buffer[120];

    int indice;



    ArrayData dados;



    void processarPacote();



public:


    ArrayLinha(
        HardwareSerial& portaSerial,
        int RX,
        int TX,
        int velocidade = 115200
    );



    void begin();



    void update();



    ArrayData getData();



};



#endif