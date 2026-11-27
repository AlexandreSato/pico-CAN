
#define CAN_2518FD

#include <SPI.h>
#include "mcp2518fd_can.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

 // =====================================================
 // PINAGEM
 // =====================================================

// SPI0 -> CAN0 -> Panda bus 0
#define CAN0_CS    17
#define CAN0_SCK   18
#define CAN0_MISO  16
#define CAN0_MOSI  19

// SPI1 -> CAN2 -> Panda bus 2
#define CAN2_CS    13
#define CAN2_SCK   10
#define CAN2_MISO  12
#define CAN2_MOSI  11

#define CAN_SPEED  CAN_500K_2M
#define CAN_CLOCK  MCP2518FD_20MHz

// =====================================================
// CONTROLADORES
// =====================================================

mcp2518fd CAN0(CAN0_CS);
mcp2518fd CAN2(CAN2_CS);

// =====================================================
// MODOS DE OPERAÇÃO
// =====================================================

enum OperationMode {
  MODE_STOPPED,
  MODE_TEST_SPI0,
  MODE_TEST_SPI1,
  MODE_GATEWAY
};

OperationMode operationMode = MODE_GATEWAY;

// =====================================================
// PROCESSAMENTO 0x147
// =====================================================

// Processamento do torque sempre ativo nos dois sentidos.
// Torque entre -102 e +102 é limitado a -102 ou +102.
// Torque zero resulta em +102.

int16_t spoofTorque = 102;

// =====================================================
// BLOQUEIO DE CAN ID
// =====================================================

// Independente do processamento 0x147.
// Aplicado nos dois sentidos.
// Para configurar diretamente no código, altere blockFilterId.

bool blockFilterEnabled = false;
unsigned long blockFilterId = 0x147;

// =====================================================
// LOG
// =====================================================

bool verbose = false;

// =====================================================
// CONTADORES
// =====================================================

uint32_t count_0_to_2 = 0;
uint32_t count_2_to_0 = 0;

uint32_t errors_0_to_2 = 0;
uint32_t errors_2_to_0 = 0;

uint32_t rx_seen_0 = 0;
uint32_t rx_seen_2 = 0;

uint32_t read_ok_0 = 0;
uint32_t read_ok_2 = 0;

uint32_t read_errors_0 = 0;
uint32_t read_errors_2 = 0;

uint32_t invalid_len_0 = 0;
uint32_t invalid_len_2 = 0;

uint32_t tx_attempts_0 = 0;
uint32_t tx_attempts_2 = 0;

// Contadores específicos do 0x147
uint32_t modified_147 = 0;
uint32_t unprocessed_147 = 0;

// Contador do bloqueio configurável
uint32_t blocked_frames = 0;

// =====================================================
// TEMPORIZAÇÃO
// =====================================================

uint32_t lastTestSend = 0;
uint32_t testSequence = 0;

// =====================================================
// SERIAL NÃO BLOQUEANTE
// =====================================================

char commandBuffer[64];
byte commandLength = 0;
uint32_t lastSerialChar = 0;

// =====================================================
// CRC RX_STEER_RELATED
// =====================================================

uint8_t checksum(const uint8_t *data, uint8_t len, uint8_t xorOutput) {

  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x1D;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc ^ xorOutput;
}

// =====================================================
// PROCESSAMENTO DO PACOTE 0x147
// =====================================================

// Altera somente a cópia que será encaminhada.
// Não altera os outros bytes.
// Retorna true somente quando o torque foi modificado.

bool process0x147(uint8_t *data, uint8_t len) {

  if (len < 16 || spoofTorque <= 0) {
    return false;
  }

  if (verbose) {
      Serial.print("ANTES:  ");
      for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
      }
      Serial.println();
  }

  uint16_t rawTorque =
    ((uint16_t)(data[9] & 0x7F) << 4) |
    ((data[10] >> 4) & 0x0F);

  int16_t originalTorque =
    (rawTorque & 0x400) ?
    (int16_t)(rawTorque - 0x800) :
    (int16_t)rawTorque;

  int16_t adjustedTorque = originalTorque;

  if (originalTorque >= 0 && originalTorque < spoofTorque) {
    adjustedTorque = spoofTorque;
  } else if (originalTorque < 0 && originalTorque > -spoofTorque) {
    adjustedTorque = -spoofTorque;
  }

  if (adjustedTorque == originalTorque) {
    return false;
  }

  if (verbose) {
    Serial.printf(
      "raw=0x%03X original=%d adjusted=%d spoof=%d\n",
      rawTorque,
      originalTorque,
      adjustedTorque,
      spoofTorque
    );
  }

  // Converte para complemento de dois de 11 bits.
  uint16_t torque = (uint16_t)adjustedTorque & 0x07FF;

  if (verbose) {
    Serial.printf("torque=0x%03X\n", torque);
  }

  // Byte 9:
  // Preserva bit 7.
  // Substitui bits 6:0.
  data[9] = (data[9] & 0x80) |
            ((torque >> 4) & 0x7F);

  // Byte 10:
  // Preserva nibble inferior.
  // Substitui nibble superior.
  data[10] = (data[10] & 0x0F) |
             ((torque & 0x0F) << 4);

  // Byte 8:
  // CRC sobre bytes 9 a 15.
  data[8] = checksum(&data[9], 7, 0x61);
   
  if (verbose) {
      Serial.print("DEPOIS: ");
      for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
      }
      Serial.println();
  }

  modified_147++;
  return true;
}

// =====================================================
// INICIALIZAÇÃO DOS CAN
// =====================================================

bool initCAN() {

  SPI.setSCK(CAN0_SCK);
  SPI.setRX(CAN0_MISO);
  SPI.setTX(CAN0_MOSI);
  SPI.begin();

  SPI1.setSCK(CAN2_SCK);
  SPI1.setRX(CAN2_MISO);
  SPI1.setTX(CAN2_MOSI);
  SPI1.begin();

  CAN0.setSPI(&SPI);
  CAN2.setSPI(&SPI1);

  Serial.println("Inicializando CAN0 / SPI0...");
  CAN0.setMode(CAN_NORMAL_MODE);

  if (CAN0.begin(CAN_SPEED, CAN_CLOCK, false) != CAN_OK) {
    Serial.println("ERRO: CAN0 nao inicializou!");
    return false;
  }
  Serial.println("CAN0 OK");

  Serial.println("Inicializando CAN2 / SPI1...");

  CAN2.setMode(CAN_NORMAL_MODE);
  if (CAN2.begin(CAN_SPEED, CAN_CLOCK, false) != CAN_OK) {
    Serial.println("ERRO: CAN2 nao inicializou!");
    return false;
  }
  Serial.println("CAN2 OK");

  return true;
}

// =====================================================
// TESTE INDIVIDUAL SPI0
// =====================================================

void testSPI0() {

  byte data[8] = {
    0x53, 0x50, 0x49, 0x30,
    (byte)(testSequence >> 24),
    (byte)(testSequence >> 16),
    (byte)(testSequence >> 8),
    (byte)testSequence
  };

  byte result = CAN0.sendMsgBuf(0x456, 0, 0, 8, data);

  Serial.print("[SPI0 TX] ID=0x456 SEQ=");
  Serial.print(testSequence);
  Serial.print(" RESULT=");
  Serial.println(result);

  testSequence++;
}

// =====================================================
// TESTE INDIVIDUAL SPI1
// =====================================================

void testSPI1() {

  byte data[64] = {
    0x53, 0x50, 0x49, 0x31,
    (byte)(testSequence >> 24),
    (byte)(testSequence >> 16),
    (byte)(testSequence >> 8),
    (byte)testSequence
  };

  for (int i = 8; i < 64; i++) {
    // data[i] = (byte)i; // Ou use 0x00 se preferir tudo zerado;
    data[i] = 0xDE; // Ou use 0x00 se preferir tudo zerado
  }  

  byte result = CAN2.sendMsgBuf(0x456, 0, CANFD::len2dlc(64), data);

  Serial.print("[SPI1 TX] ID=0x456 SEQ=");
  Serial.print(testSequence);
  Serial.print(" RESULT=");
  Serial.println(result);

  testSequence++;
}

// =====================================================
// ENCAMINHAMENTO
// =====================================================

void forwardFrame(mcp2518fd &src, mcp2518fd &dst, bool direction_0_to_2) {

  // 1. Verifica se existe quadro na FIFO RX
  byte rxStatus = src.checkReceive();

  if ((rxStatus & 0x01) == 0) {
    return;
  }

  if (direction_0_to_2) {
    rx_seen_0++;
  } else {
    rx_seen_2++;
  }

  // 2. Lê quadro
  byte len = 0;
  byte data[64] = {0};

  byte readResult = src.readMsgBuf(&len, data);

  if (readResult != CAN_OK) {
    if (direction_0_to_2) {
      read_errors_0++;
    } else {
      read_errors_2++;
    }
    return;
  }

  if (direction_0_to_2) {
    read_ok_0++;
  } else {
    read_ok_2++;
  }

  if (len > 64) {
    if (direction_0_to_2) {
      invalid_len_0++;
    } else {
      invalid_len_2++;
    }
    return;
  }

  // 3. Recupera informações originais
  unsigned long id = src.getCanId();
  byte ext = src.isExtendedFrame();
  byte rtr = src.isRemoteRequest();

  // =================================================
  // 4. BLOQUEIO CONFIGURÁVEL DE CAN ID
  // =================================================

  if (blockFilterEnabled && id == blockFilterId) {
    blocked_frames++;

    if (verbose) {
      Serial.print("[BLOCK] ID=0x");
      Serial.println(id, HEX);
    }

    return;
  }

  // =================================================
  // 5. TRATAMENTO ESPECIAL DO 0x147
  // =================================================

  if (id == 0x147) {

    if (rtr || len < 16) {
      unprocessed_147++;

      if (verbose) {
        Serial.print("[0x147 UNCHANGED] LEN=");
        Serial.print(len);
        Serial.print(" RTR=");
        Serial.println(rtr);
      }

    } else if (process0x147(data, len)) {

      if (verbose) {
        Serial.print("[MODIFIED 0x147] Limite torque=");
        Serial.print(spoofTorque);

        Serial.print(" CRC=0x");
        Serial.println(data[8], HEX);
      }
    }
  }

  // =================================================
  // 6. TRANSMISSÃO NORMAL
  // =================================================

  if (direction_0_to_2) {
    tx_attempts_0++;
  } else {
    tx_attempts_2++;
  }

  byte result = dst.trySendMsgBuf(id, ext, rtr, CANFD::len2dlc(len), data);

  if (result == CAN_OK) {

    if (direction_0_to_2) {
      count_0_to_2++;
    } else {
      count_2_to_0++;
    }

    if (verbose) {

      Serial.print(direction_0_to_2 ?
                   "[0->2] " : "[2->0] ");

      Serial.print("ID=0x");
      Serial.print(id, HEX);
      Serial.print(" LEN=");
      Serial.println(len);
    }

  } else {

    if (direction_0_to_2) {
      errors_0_to_2++;
    } else {
      errors_2_to_0++;
    }

    Serial.print("[TX ERROR] ");

    Serial.print(direction_0_to_2 ?
                 "0->2" : "2->0");

    Serial.print(" RESULT=");
    Serial.println(result);
  }
}

// =====================================================
// STATUS
// =====================================================

void printStatus() {

  Serial.println();
  Serial.println("========== STATUS ==========");

  Serial.print("Modo: ");

  switch (operationMode) {
    case MODE_STOPPED:
      Serial.println("PARADO");
      break;

    case MODE_TEST_SPI0:
      Serial.println("TESTE SPI0");
      break;

    case MODE_TEST_SPI1:
      Serial.println("TESTE SPI1");
      break;

    case MODE_GATEWAY:
      Serial.println("GATEWAY");
      break;
  }

  Serial.println("Processamento 0x147: ON");

  Serial.print("Limite minimo torque: ");
  Serial.println(spoofTorque);

  Serial.print("Bloqueio CAN ID: ");
  Serial.println(blockFilterEnabled ? "ON" : "OFF");

  Serial.print("ID bloqueado: 0x");
  Serial.println(blockFilterId, HEX);

  Serial.print("Verbose: ");
  Serial.println(verbose ? "ON" : "OFF");

  Serial.println();

  Serial.print("SPI0 -> SPI1: ");
  Serial.println(count_0_to_2);

  Serial.print("SPI1 -> SPI0: ");
  Serial.println(count_2_to_0);

  Serial.print("Erros TX 0->2: ");
  Serial.println(errors_0_to_2);

  Serial.print("Erros TX 2->0: ");
  Serial.println(errors_2_to_0);

  Serial.println("--- PROCESSAMENTO 0x147 ---");

  Serial.print("Pacotes modificados: ");
  Serial.println(modified_147);

  Serial.print("Pacotes encaminhados sem processamento: ");
  Serial.println(unprocessed_147);

  Serial.println("--- BLOQUEIO CAN ID ---");

  Serial.print("Pacotes bloqueados: ");
  Serial.println(blocked_frames);

  Serial.println("--- DIAGNOSTICO RX/TX ---");

  Serial.print("RX encontrados CAN0: ");
  Serial.println(rx_seen_0);

  Serial.print("RX encontrados CAN2: ");
  Serial.println(rx_seen_2);

  Serial.print("Leituras OK CAN0: ");
  Serial.println(read_ok_0);

  Serial.print("Leituras OK CAN2: ");
  Serial.println(read_ok_2);

  Serial.print("Erros leitura CAN0: ");
  Serial.println(read_errors_0);

  Serial.print("Erros leitura CAN2: ");
  Serial.println(read_errors_2);

  Serial.print("Comprimento invalido CAN0: ");
  Serial.println(invalid_len_0);

  Serial.print("Comprimento invalido CAN2: ");
  Serial.println(invalid_len_2);

  Serial.print("Tentativas TX 0->2: ");
  Serial.println(tx_attempts_0);

  Serial.print("Tentativas TX 2->0: ");
  Serial.println(tx_attempts_2);

  Serial.println("============================");
  Serial.println();
}

// =====================================================
// COMANDOS
// =====================================================

void printHelp() {

  Serial.println();
  Serial.println("========== COMANDOS ==========");
  Serial.println("teste spi0       - Testa transmissao SPI0");
  Serial.println("teste spi1       - Testa transmissao SPI1");
  Serial.println("gateway          - Inicia gateway");
  Serial.println("parar            - Para tudo");
  Serial.println("bloqueio on      - Ativa bloqueio do CAN ID");
  Serial.println("bloqueio off     - Desativa bloqueio do CAN ID");
  Serial.println("bloqueio id 0x147 - Define ID a bloquear (HEX)");
  Serial.println("verbose on       - Ativa logs");
  Serial.println("verbose off      - Desativa logs");
  Serial.println("status           - Mostra status");
  Serial.println("zerar            - Zera contadores");
  Serial.println("help             - Ajuda");
  Serial.println("==============================");
  Serial.println();
}

void processCommand(char *cmd) {

  // Converter para minúsculas
  for (byte i = 0; cmd[i]; i++) {
    if (cmd[i] >= 'A' && cmd[i] <= 'Z') {
      cmd[i] += 32;
    }
  }

  if (strcmp(cmd, "teste spi0") == 0) {
    operationMode = MODE_TEST_SPI0;
    lastTestSend = 0;
    Serial.println("Modo TESTE SPI0 ativado.");

  } else if (strcmp(cmd, "teste spi1") == 0) {
    operationMode = MODE_TEST_SPI1;
    lastTestSend = 0;
    Serial.println("Modo TESTE SPI1 ativado.");

  } else if (strcmp(cmd, "gateway") == 0) {
    operationMode = MODE_GATEWAY;
    Serial.println("Gateway bidirecional ATIVO.");

  } else if (strcmp(cmd, "parar") == 0) {
    operationMode = MODE_STOPPED;
    Serial.println("Tudo PARADO.");

  } else if (strcmp(cmd, "bloqueio on") == 0) {
    blockFilterEnabled = true;
    Serial.print("Bloqueio ATIVADO para ID 0x");
    Serial.println(blockFilterId, HEX);

  } else if (strcmp(cmd, "bloqueio off") == 0) {
    blockFilterEnabled = false;
    Serial.println("Bloqueio DESATIVADO.");

  } else if (strncmp(cmd, "bloqueio id ", 12) == 0) {

    char *idText = cmd + 12;
    char *endPtr;

    unsigned long requestedId = strtoul(idText, &endPtr, 16);

    if (strncmp(idText, "0x", 2) == 0 &&
        endPtr != idText + 2 &&
        *endPtr == '\0' &&
        requestedId <= 0x1FFFFFFF) {

      blockFilterId = requestedId;

      Serial.print("ID de bloqueio definido: 0x");
      Serial.println(blockFilterId, HEX);

    } else {
      Serial.println("ID invalido. Use hexadecimal: bloqueio id 0x147");
    }

  } else if (strcmp(cmd, "verbose on") == 0) {
    verbose = true;
    Serial.println("Verbose ON.");

  } else if (strcmp(cmd, "verbose off") == 0) {
    verbose = false;
    Serial.println("Verbose OFF.");

  } else if (strcmp(cmd, "status") == 0) {
    printStatus();

  } else if (strcmp(cmd, "zerar") == 0) {

    count_0_to_2 = 0;
    count_2_to_0 = 0;

    errors_0_to_2 = 0;
    errors_2_to_0 = 0;

    rx_seen_0 = 0;
    rx_seen_2 = 0;

    read_ok_0 = 0;
    read_ok_2 = 0;

    read_errors_0 = 0;
    read_errors_2 = 0;

    invalid_len_0 = 0;
    invalid_len_2 = 0;

    tx_attempts_0 = 0;
    tx_attempts_2 = 0;

    modified_147 = 0;
    unprocessed_147 = 0;
    blocked_frames = 0;

    Serial.println("Contadores zerados.");

  } else if (strcmp(cmd, "help") == 0) {
    printHelp();

  } else if (cmd[0] != '\0') {
    Serial.print("Comando desconhecido: ");
    Serial.println(cmd);
  }
}

// =====================================================
// RECEPÇÃO SERIAL NÃO BLOQUEANTE
// =====================================================

void readSerialCommands() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\r' || c == '\n') {

      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }

    } else {

      if (commandLength < sizeof(commandBuffer) - 1) {
        commandBuffer[commandLength++] = c;
        lastSerialChar = millis();
      } else {
        commandLength = 0;
        Serial.println("Comando muito longo.");
      }
    }
  }

  // Aceita comandos sem final de linha

  if (commandLength > 0 &&
      millis() - lastSerialChar >= 100) {

    commandBuffer[commandLength] = '\0';
    processCommand(commandBuffer);
    commandLength = 0;
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("================================");
  Serial.println("     PICO MCP2518FD GATEWAY");
  Serial.println("================================");

  if (!initCAN()) {

    Serial.println("Falha na inicializacao CAN.");

    while (1) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("Inicializacao concluida.");
  Serial.println("Aguardando comando.");

  printHelp();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  readSerialCommands();

  uint32_t now = millis();

  // ---------------------------------------------------
  // TESTE SPI0
  // ---------------------------------------------------

  if (operationMode == MODE_TEST_SPI0) {
    if (now - lastTestSend >= 1000) {
      lastTestSend = now;
      testSPI0();
    }
    return;
  }

  // ---------------------------------------------------
  // TESTE SPI1
  // ---------------------------------------------------

  if (operationMode == MODE_TEST_SPI1) {
    if (now - lastTestSend >= 1000) {
      lastTestSend = now;
      testSPI1();
    }

    return;
  }

  // ---------------------------------------------------
  // GATEWAY
  // ---------------------------------------------------

  if (operationMode == MODE_GATEWAY) {
    forwardFrame(CAN0, CAN2, true);
    forwardFrame(CAN2, CAN0, false);
  }
}
