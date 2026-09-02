#include "ControleObstaculo.h"


ControleObstaculo::ControleObstaculo(
    IMU& imu,
    MotorControlador& motorControlador
)
:
sensorIMU(imu),
motores(motorControlador)
{
    estado = PARADO;

    headingAlvo = 0.0f;

    kp = 3.0f;

    tolerancia = 2.0f;

    velocidade = 180;

    correcaoMaxima = 100;
}


void ControleObstaculo::begin()
{
    estado = PARADO;

    motores.stop();
}


void ControleObstaculo::iniciarReto()
{
    headingAlvo = sensorIMU.getHeading();

    estado = RETO;
}


void ControleObstaculo::update()
{
    if(estado != RETO)
        return;


    sensorIMU.update();


    float erro =
        IMU::diferencaAngular(
            sensorIMU.getHeading(),
            headingAlvo
        );


    if(fabs(erro) <= tolerancia)
    {
        motores.setSpeed(
            velocidade,
            velocidade,
            velocidade,
            velocidade
        );

        return;
    }


    float correcao = kp * erro;


    if(correcao > correcaoMaxima)
        correcao = correcaoMaxima;


    if(correcao < -correcaoMaxima)
        correcao = -correcaoMaxima;


    int esquerda =
        limitarVelocidade(
            velocidade + (int)correcao
        );


    int direita =
        limitarVelocidade(
            velocidade - (int)correcao
        );


    motores.setSpeed(
        esquerda,
        esquerda,
        direita,
        direita
    );
}


void ControleObstaculo::parar()
{
    motores.stop();

    estado = FINALIZADO;
}


bool ControleObstaculo::executando() const
{
    return estado == RETO;
}


bool ControleObstaculo::terminou() const
{
    return estado == FINALIZADO;
}


void ControleObstaculo::cancelar()
{
    motores.stop();

    estado = PARADO;
}


void ControleObstaculo::setVelocidade(
    int valor
)
{
    if(valor < 0)
        valor = -valor;

    velocidade = limitarVelocidade(valor);
}


void ControleObstaculo::setKp(
    float valor
)
{
    kp = valor;
}


void ControleObstaculo::setTolerancia(
    float valor
)
{
    if(valor < 0)
        valor = -valor;

    tolerancia = valor;
}


float ControleObstaculo::limitarAngulo(
    float valor
)
{
    while(valor > 180.0f)
        valor -= 360.0f;

    while(valor < -180.0f)
        valor += 360.0f;

    return valor;
}


int ControleObstaculo::limitarVelocidade(
    int valor
)
{
    if(valor > 1000)
        valor = 1000;

    if(valor < -1000)
        valor = -1000;

    return valor;
}