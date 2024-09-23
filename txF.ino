#define HELTEC_POWER_BUTTON   // Deve ser definido antes de "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <HardwareSerial.h>
#include <esp_task_wdt.h>

#define FREQUENCY 915.2       // Frequência em MHz (Brasil)
#define BANDWIDTH 250.0       // Largura de banda em kHz
#define SPREADING_FACTOR 7   // Fator de espalhamento
#define TRANSMIT_POWER 12     // Potência de transmissão em dBm

#define SERIAL_TX_PIN 19
#define SERIAL_RX_PIN 20

HardwareSerial mySerial(1);
#define MAX_PACKET_SIZE 256

uint8_t buffer[MAX_PACKET_SIZE];
int bufferIndex = 0;
int receivedCount = 0;

String dataStr = "";
String retornoStr = "";

String rxData;
volatile bool rxFlag = false;

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
  display.drawString(0, 0, "Data received:");
  display.drawString(0, 10, dataStr);
  display.drawString(0, 20, "Data transmitted!");
  display.drawString(0, 30, "Retorno Recebido:");
  display.drawString(0, 40, retornoStr);
  display.display();
}

void taskSend(void *pvParameters) {
  while (true) {
    if (mySerial.available()) {
      uint8_t byte = mySerial.read();
      if (byte == ) {
        bufferIndex = 0;
        buffer[bufferIndex++] = byte;
      } else if (byte ==  && bufferIndex > 0 && buffer[0] == ) {
        buffer[bufferIndex++] = byte;
        
        // Preparando a string para exibição
        dataStr = "";
        for (int i = 0; i < bufferIndex; i++) {
          dataStr += String(buffer[i], HEX) + " ";
        }
        // Transmitir os dados recebidos via LoRa
        RADIOLIB(radio.transmit(buffer, bufferIndex));
        vTaskDelay(1 / portTICK_PERIOD_MS);
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF)); 
        bufferIndex = 0;
      } else if (bufferIndex > 0 && bufferIndex < MAX_PACKET_SIZE) {
        buffer[bufferIndex++] = byte;
      }
    } 
    else {
      vTaskDelay(2/ portTICK_PERIOD_MS); // Redução do uso da CPU quando não há dados disponíveis
    }
  }
}

void taskReceive(void *pvParameters) {
  while (true) {
    if (rxFlag) {
      rxFlag = false;
      retornoStr = "";  // Reinicializa a string para evitar sobreposição

      if (rxData.length() > 1 && isValidSecondByte(rxData[1])) {
        // Envia os dados via Serial
         for (int i = 0; i < rxData.length(); i++) {
            retornoStr += String(rxData[i], HEX) + " ";
            mySerial.write(rxData[i]);  // Envia byte a byte
         }
      }
      RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));  
      
      vTaskDelay(1 / portTICK_PERIOD_MS); // Redução do uso da CPU quando não há dados disponíveis
    } 
    else {
      vTaskDelay(2 / portTICK_PERIOD_MS); // Redução do uso da CPU quando não há dados disponíveis
    }
  }
}

void rx() {
  rxFlag = true;
  radio.readData(rxData);
}

void setup() {
  esp_task_wdt_delete(NULL);
  heltec_setup();
  mySerial.begin(9600, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
  both.println("LoRa TX init");
  
  RADIOLIB_OR_HALT(radio.begin());
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  
  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

  xTaskCreatePinnedToCore(taskSend, "TaskSend",  16384, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskReceive, "TaskReceive",  16384, NULL,2, NULL, 1);

  esp_task_wdt_delete(NULL); // Remove qualquer WDT de tarefa 
  
}

void loop() {
  heltec_loop();
  displayUpdate();
  vTaskDelay(1 / portTICK_PERIOD_MS);
}
