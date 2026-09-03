#include "MotorControlador.h"


// ============================================================
// CONSTRUTOR
// ============================================================

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


    // --------------------------------------------------------
    // Limite utilizado pelo robô
    // --------------------------------------------------------

    velocidadeMaxima = 1000;


    // --------------------------------------------------------
    // Últimos comandos
    // --------------------------------------------------------

    ultimoM1 = 0;
    ultimoM2 = 0;
    ultimoM3 = 0;
    ultimoM4 = 0;

}



// ============================================================
// BEGIN
// ============================================================

void MotorControlador::begin()
{

    // --------------------------------------------------------
    // Inicializa UART
    // --------------------------------------------------------

    serial.begin(
        baud,
        SERIAL_8N1,
        rxPin,
        txPin
    );


    delay(1000);


    // ========================================================
    // CONFIGURAÇÃO DO DRIVER YAHBOOM
    // ========================================================


    // --------------------------------------------------------
    // Tipo do motor
    // Motor 310
    // --------------------------------------------------------

    enviarComando("$mtype:2#");

    delay(300);


    // --------------------------------------------------------
    // Encoder
    // 13 linhas
    // --------------------------------------------------------

    enviarComando("$mline:13#");

    delay(300);


    // --------------------------------------------------------
    // Redução
    // 1:20
    // --------------------------------------------------------

    enviarComando("$mphase:20#");

    delay(300);


    // --------------------------------------------------------
    // Diâmetro da roda
    // 66 mm
    // --------------------------------------------------------

    enviarComando("$wdiameter:66#");

    delay(300);


    // --------------------------------------------------------
    // Deadzone
    // --------------------------------------------------------

    enviarComando("$deadzone:1600#");

    delay(300);


    // --------------------------------------------------------
    // PID interno do driver
    //
    // Mantido exatamente como estava.
    // --------------------------------------------------------

    enviarComando("$MPID:0.8,0.06,0.5#");

    delay(300);


    // --------------------------------------------------------
    // Segurança:
    // primeiro zera a referência de velocidade
    // --------------------------------------------------------

    stop();

    delay(100);


    // --------------------------------------------------------
    // Depois libera completamente o controle PID
    // --------------------------------------------------------

    release();

}



// ============================================================
// ENVIAR COMANDO
// ============================================================

void MotorControlador::enviarComando(const char* comando)
{

    serial.print(comando);

}



// ============================================================
// LIMITAR VELOCIDADE
// ============================================================

int MotorControlador::limitarVelocidade(int velocidade)
{

    if(velocidade > velocidadeMaxima)
    {
        return velocidadeMaxima;
    }


    if(velocidade < -velocidadeMaxima)
    {
        return -velocidadeMaxima;
    }


    return velocidade;

}



// ============================================================
// SET SPEED
// ============================================================

void MotorControlador::setSpeed(
    int m1,
    int m2,
    int m3,
    int m4
)
{

    // --------------------------------------------------------
    // Limita velocidades
    // --------------------------------------------------------

    m1 = limitarVelocidade(m1);
    m2 = limitarVelocidade(m2);
    m3 = limitarVelocidade(m3);
    m4 = limitarVelocidade(m4);

    // INVERTE O SENTIDO FÍSICO DOS MOTORES
    m1 = -m1;
    m2 = -m2;
    m3 = -m3;
    m4 = -m4;

    // --------------------------------------------------------
    // Guarda último comando
    // --------------------------------------------------------

    ultimoM1 = m1;
    ultimoM2 = m2;
    ultimoM3 = m3;
    ultimoM4 = m4;


    // --------------------------------------------------------
    // Monta comando do driver
    //
    // $spd:M1,M2,M3,M4#
    // --------------------------------------------------------

    String comando = "$spd:";

    comando += String(m1);
    comando += ",";
    comando += String(m2);
    comando += ",";
    comando += String(m3);
    comando += ",";
    comando += String(m4);
    comando += "#";


    enviarComando(comando.c_str());

}



// ============================================================
// STOP
// ============================================================

void MotorControlador::stop()
{

    // --------------------------------------------------------
    // Zera a referência de velocidade.
    //
    // ATENÇÃO:
    // Segundo o protocolo Yahboom, o PID continua ativo
    // mesmo com velocidade igual a zero.
    // --------------------------------------------------------

    setSpeed(
        0,
        0,
        0,
        0
    );

}



// ============================================================
// RELEASE
// ============================================================

void MotorControlador::release()
{

    // --------------------------------------------------------
    // Libera completamente o controle dos motores.
    //
    // O comando PWM com zero desativa a atuação do PID
    // sobre os motores.
    //
    // $pwm:0,0,0,0#
    // --------------------------------------------------------

    enviarComando("$pwm:0,0,0,0#");


    // --------------------------------------------------------
    // Como não estamos enviando velocidade aqui, mantemos
    // os últimos valores registrados como zero.
    // --------------------------------------------------------

    ultimoM1 = 0;
    ultimoM2 = 0;
    ultimoM3 = 0;
    ultimoM4 = 0;

}



// ============================================================
// GET M1
// ============================================================

int MotorControlador::getM1()
{

    return ultimoM1;

}



// ============================================================
// GET M2
// ============================================================

int MotorControlador::getM2()
{

    return ultimoM2;

}



// ============================================================
// GET M3
// ============================================================

int MotorControlador::getM3()
{

    return ultimoM3;

}



// ============================================================
// GET M4
// ============================================================

int MotorControlador::getM4()
{

    return ultimoM4;

}



// ============================================================
// AVAILABLE
// ============================================================

bool MotorControlador::available()
{

    return serial.available();

}



// ============================================================
// READ DATA
// ============================================================

String MotorControlador::readData()
{

    String resposta = "";


    while(serial.available())
    {

        resposta += (char)serial.read();

    }


    return resposta;

}