#ifndef MOTORCONTROLADOR_H
#define MOTORCONTROLADOR_H

#include <Arduino.h>


class MotorControlador
{

private:

    // ========================================================
    // Comunicação UART
    // ========================================================

    HardwareSerial& serial;

    int rxPin;
    int txPin;
    int baud;


    // ========================================================
    // Configuração dos motores
    // ========================================================

    int velocidadeMaxima;


    // ========================================================
    // Última velocidade enviada
    // ========================================================

    int ultimoM1;
    int ultimoM2;
    int ultimoM3;
    int ultimoM4;


    // ========================================================
    // Envia comando bruto para o driver
    // ========================================================

    void enviarComando(const char* comando);


    // ========================================================
    // Limita uma velocidade ao intervalo permitido
    // ========================================================

    int limitarVelocidade(int velocidade);


public:

    // ========================================================
    // Construtor
    // ========================================================

    MotorControlador(
        HardwareSerial& portaSerial,
        int RX,
        int TX,
        int velocidade = 115200
    );


    // ========================================================
    // Inicializa UART e configura o driver
    // ========================================================

    void begin();


    // ========================================================
    // Define velocidade individual dos quatro motores
    //
    // M1/M2 = esquerda
    // M3/M4 = direita
    //
    // positivo = frente
    // negativo = ré
    // ========================================================

    void setSpeed(
        int m1,
        int m2,
        int m3,
        int m4
    );


    // ========================================================
    // Para os motores usando controle de velocidade (PID)
    //
    // IMPORTANTE:
    // O PID interno do Yahboom continua ativo.
    // ========================================================

    void stop();


    // ========================================================
    // Libera completamente o controle PID dos motores
    //
    // Envia:
    // $pwm:0,0,0,0#
    //
    // Usado quando queremos que os motores realmente fiquem
    // sem comando de torque pelo controlador de velocidade.
    // ========================================================

    void release();


    // ========================================================
    // Retorna as últimas velocidades enviadas
    // ========================================================

    int getM1();
    int getM2();
    int getM3();
    int getM4();


    // ========================================================
    // Verifica se existe resposta aguardando na UART
    // ========================================================

    bool available();


    // ========================================================
    // Lê respostas do driver
    // ========================================================

    String readData();

};

#endif