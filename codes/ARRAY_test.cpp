#include <Arduino.h>

// Código para ESP32 (RoboCore Vespa)
// Lê pacotes do sensor no formato:
// $A,x1:4096,x2:4096,x3:4096,x4:4096,x5:4096,x6:4096,x7:4096,x8:4096#
// Extrai os números e guarda em s1..s8

#define RX2_PIN 5     
#define TX2_PIN 18     
#define BAUD 115200    

void processarPacote();

char buffer[120];     // buffer para montar a mensagem recebida
int indice = 0;       // índice atual no buffer

// variáveis individuais dos sensores (use s1..s8 diretamente no seu código)
int s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0, s7 = 0, s8 = 0;

void setup() {
  Serial.begin(115200); // debug no PC
  // inicializa UART2 (Serial2) com pinos RX2_PIN/TX2_PIN
  Serial2.begin(BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(300);

  // pede ao módulo para enviar somente valores analógicos (opcional,
  // dependendo do seu módulo isso ativa a transmissão)
  Serial2.print("$0,1,0#");

  Serial.println("Iniciado. Aguardando pacotes do sensor...");
}

void loop() {
  // Leitura contínua da Serial2
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    // Ignora CR/LF que podem vir no final das linhas
    if (c == '\r' || c == '\n') continue;

    // Se chegar '#', pacote finalizado -> processa buffer
    if (c == '#') {
      buffer[indice] = '\0';        // finaliza string C
      if (indice > 0) {
        processarPacote();         // extrai valores e popula s1..s8
      }
      indice = 0;                  // reinicia buffer para próxima mensagem
    } else {
      // Guarda caracteres no buffer (com proteção contra overflow)
      if (indice < (int)sizeof(buffer) - 1) {
        buffer[indice++] = c;
      } else {
        // overflow do buffer: descarta dados (pode ajustar estratégia)
        indice = 0;
      }
    }
  }
}

// Função que usa sscanf para extrair os 8 valores do buffer
void processarPacote() {
  // Exemplo esperado no buffer:
  // $A,x1:4096,x2:1234,x3:4095,x4:1000,x5:0,x6:4096,x7:4000,x8:3900

  // Tenta ler com sscanf; retorna o número de itens lidos
  int lidos = sscanf(buffer,
    "$A,x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d",
    &s1, &s2, &s3, &s4, &s5, &s6, &s7, &s8);

  // Se lidos != 8, pacote pode estar com formato diferente — pode tratar aqui
  if (lidos != 8) {
    // DEBUG: mostrar pacote incompleto/errado (comente em produção)
    Serial.print("Pacote ignorado (formato errado): ");
    Serial.println(buffer);
    return;
  }

  // Sucesso: agora s1..s8 contêm os valores. Mostra no Monitor Serial:
  Serial.print("Sensores: ");
  Serial.print(s1); Serial.print(" ");
  Serial.print(s2); Serial.print(" ");
  Serial.print(s3); Serial.print(" ");
  Serial.print(s4); Serial.print(" ");
  Serial.print(s5); Serial.print(" ");
  Serial.print(s6); Serial.print(" ");
  Serial.print(s7); Serial.print(" ");
  Serial.println(s8);

}
