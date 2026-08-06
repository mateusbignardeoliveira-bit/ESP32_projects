#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ==================================================================
// PARÂMETRO DE CALIBRAÇÃO (OFFSET)
// Se o sensor está medindo 12cm quando deveria medir 10cm, ele está 
// com um erro de +20 milímetros. Ajuste o valor abaixo para corrigir.
// ==================================================================
const int OFFSET_CORRECAO_MM = -20; // Subtrai 20mm (2cm) da leitura bruta

// Configurações do Filtro de Estabilização
const int NUM_LEITURAS = 10; 
int leituras[NUM_LEITURAS];   
int indiceLeitura = 0;       
long totalSoma = 0;          

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Serial.println("\n==========================================");
  Serial.println(" TOF200F I2C - Modo Calibrado de Precisao ");
  Serial.println("==========================================");

  Wire.begin(21, 22); 
  Wire.setClock(100000); 

  if (!lox.begin(0x29)) { 
    Serial.println(F("ERRO: Sensor TOF200F nao encontrado!"));
    while (1); 
  }
  
  // Aumenta o tempo de coleta de luz para máxima precisão interna do chip
  lox.setMeasurementTimingBudgetMicroSeconds(200000); 

  // Inicializa o vetor do filtro
  for (int i = 0; i < NUM_LEITURAS; i++) {
    leituras[i] = 0;
  }

  Serial.println(F("Sensor calibrado e filtrado pronto!"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t medida;
  lox.rangingTest(&medida, false); 

  if (medida.RangeStatus != 4) {  
    // 1. Pega a leitura bruta do chip em milímetros
    int leituraBruta = medida.RangeMilliMeter;

    // 2. Aplica a compensação de calibração matemática
    int leituraCalibrada = leituraBruta + OFFSET_CORRECAO_MM;

    // Impede que a calibração gere números negativos se estiver encostado em algo
    if (leituraCalibrada < 0) {
      leituraCalibrada = 0; 
    }

    // 3. Aplica o Filtro de Média Móvel para eliminar tremulações
    totalSoma = totalSoma - leituras[indiceLeitura];
    leituras[indiceLeitura] = leituraCalibrada;
    totalSoma = totalSoma + leituras[indiceLeitura];
    indiceLeitura = indiceLeitura + 1;

    if (indiceLeitura >= NUM_LEITURAS) {
      indiceLeitura = 0;
    }

    // Resultado final com estabilidade e calibração
    int distanciaFinal_mm = totalSoma / NUM_LEITURAS;
    float distanciaFinal_cm = (float)distanciaFinal_mm / 10.0;

    // Exibe os dados corrigidos na tela
    Serial.print("Distancia Corrigida: ");
    Serial.print(distanciaFinal_mm);
    Serial.print(" mm  |  ");
    Serial.print(distanciaFinal_cm, 1);
    Serial.println(" cm");
  } 
  else {
    Serial.println("Aviso: Objeto fora de alcance ou sinal fraco");
  }
  
  delay(30); 
}
