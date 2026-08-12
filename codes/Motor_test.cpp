#include <Arduino.h>

HardwareSerial MotorSerial(2);

// UART2
#define RX_PIN 16
#define TX_PIN 17

void sendCommand(String cmd)
{
  Serial.print("Enviando: ");
  Serial.println(cmd);
  MotorSerial.print(cmd);
  delay(300);
}

void setup()
{
  Serial.begin(115200);

  MotorSerial.begin(
    115200,
    SERIAL_8N1,
    RX_PIN,
    TX_PIN
  );

  delay(1000);

  Serial.println("Configurando driver...");

  // Motor 310
  sendCommand("$mtype:2#");

  // Encoder: 13 linhas
  sendCommand("$mline:13#");

  // Redução 1:20
  sendCommand("$mphase:20#");

  // Roda 48mm
  sendCommand("$wdiameter:48#");

  // Deadzone padrão
  sendCommand("$deadzone:1600#");

  // PID padrão
  sendCommand("$MPID:0.8,0.06,0.5#");


  Serial.println("Configurado!");

  delay(1000);


  // M1 = 500
  // M2 = 0
  // M3 = 0
  // M4 = 0

  sendCommand("$spd:0,0,0,0#");

  sendCommand("$upload:0,0,1#");

}


void loop()
{
  // apenas mostra respostas da placa
  while(MotorSerial.available())
  {
    Serial.write(MotorSerial.read());
  }
}