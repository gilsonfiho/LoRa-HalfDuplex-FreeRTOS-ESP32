#define HELTEC_POWER_BUTTON   // Deve ser definido antes de "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#define SERIAL_TX_PIN 19  // Pino TX para comunicação serial
#define SERIAL_RX_PIN 20  // Pino RX para comunicação serial

HardwareSerial mySerial(1);  // UART1 do ESP32 para comunicação serial

#define FREQUENCY 915.2       // Frequência em MHz (Brasil)
#define BANDWIDTH 250.0       // Largura de banda em kHz
#define SPREADING_FACTOR 7   // Fator de espalhamento
#define TRANSMIT_POWER 12     // Potência de transmissão em dBm

#define MAX_PACKET_SIZE 256

String rxData;
volatile bool rxFlag = false;

uint8_t buffer[MAX_PACKET_SIZE];
int bufferIndex = 0;

String retornoControladora = "";
String pacoteStr = "";

int erroContador = 0;
int pacotesEnviados = 0;
int retornoRecebido = 0;

const uint8_t validBytes[] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x90, 0x91, 0x92, 0x93};  // Vetor de hexadecimais válidos
const int validBytesSize = sizeof(validBytes) / sizeof(validBytes[0]);

bool isValidSecondByte(uint8_t byte) {
    for (int i = 0; i < validBytesSize; i++) {
        if (byte == validBytes[i]) {
            return true;
        }
    }
    return false;
}

void displayUpdate() {
  display.clear();
  display.drawString(0, 0, "Recebendo Pacote:");
  display.drawString(0, 10, pacoteStr);
  display.drawString(0, 20, "Retorno da Controladora:");
  display.drawString(0, 30, retornoControladora);
  display.drawString(0, 40, "QP: " + String(pacotesEnviados));
  display.drawString(70, 40, "T: " + String(millis() / 1000));
  display.drawString(0, 50, "ErrosE1: " + String(erroContador));
  display.drawString(70, 50, "(%): " + String((float)erroContador / pacotesEnviados * 100, 2));
  display.display();
}

// Tarefa para receber dados via LoRa e enviar para a controladora
void TaskLoRaToControladora(void *pvParameters) {
  while (true) {
    if (rxFlag) {
      rxFlag = false;
      pacoteStr = "";
    
      if (rxData.length() > 1 && isValidSecondByte(rxData[1])) {
        // Envia os dados via Serial
         for (int i = 0; i < rxData.length(); i++) {
            char byteToSend = rxData[i]; // Obtém o caractere da String
            mySerial.write(byteToSend);  // Envia byte a byte
            pacoteStr += String(rxData[i], HEX) + " ";
         }
      }
      RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));  
      pacotesEnviados++;
      vTaskDelay(1 / portTICK_PERIOD_MS);  // Delay para simular um tempo de processamento
    }
    else{
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
  }
}

// Tarefa para ler o retorno da controladora e enviar via LoRa
void TaskControladoraToLoRa(void *pvParameters) {
  while (true) {
    if (mySerial.available()) {
      uint8_t byte = mySerial.read();
      
      if (byte == 0xAB) {
          bufferIndex = 0;
          buffer[bufferIndex++] = byte;
      } 
      else if (byte == 0xCD && bufferIndex > 0 && buffer[0] == 0xAB) {
          buffer[bufferIndex++] = byte;
  
          retornoControladora = "";
          for (int i = 0; i < bufferIndex; i++) {
              retornoControladora += String(buffer[i], HEX) + " ";
          }
  
          if (retornoControladora.indexOf("e1") >= 0) {
              erroContador++;
          }

          RADIOLIB(radio.transmit(buffer, bufferIndex));
          vTaskDelay(1 / portTICK_PERIOD_MS);
          RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF)); 
  
          bufferIndex = 0;
      } 
      else if (bufferIndex > 0 && bufferIndex < MAX_PACKET_SIZE) {
          buffer[bufferIndex++] = byte;
      }
    }
    else{
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
  }
}

void rx() {
  rxFlag = true;
  radio.readData(rxData);
}

void setup() {
  heltec_setup();
  mySerial.begin(9600, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
  both.println("LoRa RX init");
  RADIOLIB_OR_HALT(radio.begin());
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

  xTaskCreatePinnedToCore(
    TaskLoRaToControladora,    
    "LoRaToControladora",      
    16384,                     
    NULL,                      
    2,                         
    NULL,                      
    1                         
  );

  xTaskCreatePinnedToCore(
    TaskControladoraToLoRa,    
    "ControladoraToLoRa",      
    16384,                     
    NULL,                      
    2,                         
    NULL,                      
    1                         
  );

  esp_task_wdt_delete(NULL); // Remove qualquer WDT de tarefa 
}

void loop() {
  heltec_loop();
  displayUpdate();
  vTaskDelay(1 / portTICK_PERIOD_MS);
}
