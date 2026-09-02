#ifndef CONTROLE_OBSTACULO_H
#define CONTROLE_OBSTACULO_H

#include <Arduino.h>

#include "../Hardware/MotorControlador.h"
#include "../sensores/IMU.h"


class ControleObstaculo
{
public:

    enum Estado
    {
        PARADO,
        RETO,
        FINALIZADO
    };


    ControleObstaculo(
        IMU& imu,
        MotorControlador& motores
    );


    void begin();

    void iniciarReto();

    void update();

    void parar();

    bool executando() const;

    bool terminou() const;

    void cancelar();


    // --------------------------------------------------------
    // Configurações
    // --------------------------------------------------------

    void setVelocidade(int velocidade);

    void setKp(float valor);

    void setTolerancia(float valor);


private:

    IMU& sensorIMU;

    MotorControlador& motores;


    Estado estado;


    // --------------------------------------------------------
    // Controle de orientação
    // --------------------------------------------------------

    float headingAlvo;

    float kp;

    float tolerancia;


    int velocidade;


    // --------------------------------------------------------
    // Limites
    // --------------------------------------------------------

    int correcaoMaxima;


    float limitarAngulo(
        float valor
    );


    int limitarVelocidade(
        int valor
    );
};

#endif