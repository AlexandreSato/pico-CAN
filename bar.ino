
#define CAN_2518FD

#include <SPI.h>
#include "mcp2518fd_can.h"
#include <stdlib.h>
#include <string.h>

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

OperationMode operationMode = MODE_STOPPED;

// =====================================================
// FILTRO
// =====================================================

bool filterEnabled = true;
unsigned long filterId = 0x123;

// =====================================================
// LOG
// =====================================================

bool verbose = true;

// =====================================================
// CONTADORES
// =====================================================

uint32_t count_0_to_2 = 0;
uint32_t count_2_to_0 = 0;

uint32_t blocked_0_to_2 = 0;
uint32_t blocked_2_to_0 = 0;

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

  if (CAN0.begin(CAN_SPEED, CAN_CLOCK, false) != CAN_OK) {
    Serial.println("ERRO: CAN0 nao inicializou!");
    return false;
  }

  CAN0.setMode(CAN_NORMAL_MODE);
  Serial.println("CAN0 OK");

  Serial.println("Inicializando CAN2 / SPI1...");

  if (CAN2.begin(CAN_SPEED, CAN_CLOCK, false) != CAN_OK) {
    Serial.println("ERRO: CAN2 nao inicializou!");
    return false;
  }

  CAN2.setMode(CAN_NORMAL_MODE);
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

  byte result = CAN0.sendMsgBuf(
    0x456, 0, 0, 8, data
  );

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

  byte data[8] = {
    0x53, 0x50, 0x49, 0x31,
    (byte)(testSequence >> 24),
    (byte)(testSequence >> 16),
    (byte)(testSequence >> 8),
    (byte)testSequence
  };

  byte result = CAN2.sendMsgBuf(
    0x456, 0, 0, 8, data
  );

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

  // 2. Lê o quadro
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

  unsigned long id = src.getCanId();
  byte ext = src.isExtendedFrame();
  byte rtr = src.isRemoteRequest();

  // 3. Filtro
  if (filterEnabled && id != filterId) {
    if (direction_0_to_2) {
      blocked_0_to_2++;
    } else {
      blocked_2_to_0++;
    }
    return;
  }

  // 4. Tenta transmitir
  if (direction_0_to_2) {
    tx_attempts_0++;
  } else {
    tx_attempts_2++;
  }

  byte result = dst.trySendMsgBuf(id, ext, rtr,  CANFD::len2dlc(len), data);

  if (result == CAN_OK) {

    if (direction_0_to_2) {
      count_0_to_2++;
    } else {
      count_2_to_0++;
    }

    if (verbose) {
      Serial.print(direction_0_to_2 ? "[0->2] " : "[2->0] ");
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
    Serial.print(direction_0_to_2 ? "0->2" : "2->0");
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

  Serial.print("Filtro: ");
  Serial.println(filterEnabled ? "ON" : "OFF");

  Serial.print("ID permitido: 0x");
  Serial.println(filterId, HEX);

  Serial.print("Verbose: ");
  Serial.println(verbose ? "ON" : "OFF");

  Serial.println();

  Serial.print("SPI0 -> SPI1: ");
  Serial.println(count_0_to_2);

  Serial.print("SPI1 -> SPI0: ");
  Serial.println(count_2_to_0);

  Serial.print("Bloqueados 0->2: ");
  Serial.println(blocked_0_to_2);

  Serial.print("Bloqueados 2->0: ");
  Serial.println(blocked_2_to_0);

  Serial.print("Erros TX 0->2: ");
  Serial.println(errors_0_to_2);

  Serial.print("Erros TX 2->0: ");
  Serial.println(errors_2_to_0);

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
  Serial.println("teste spi0    - Testa transmissao SPI0");
  Serial.println("teste spi1    - Testa transmissao SPI1");
  Serial.println("gateway       - Inicia gateway");
  Serial.println("parar         - Para tudo");
  Serial.println("filtro on     - Ativa filtro");
  Serial.println("filtro off    - Desativa filtro");
  Serial.println("filtro id 123 - Define ID permitido");
  Serial.println("verbose on    - Ativa logs");
  Serial.println("verbose off   - Desativa logs");
  Serial.println("status        - Mostra status");
  Serial.println("zerar         - Zera contadores");
  Serial.println("help          - Ajuda");
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

  } else if (strcmp(cmd, "filtro on") == 0) {

    filterEnabled = true;
    Serial.println("Filtro ATIVADO.");

  } else if (strcmp(cmd, "filtro off") == 0) {

    filterEnabled = false;
    Serial.println("Filtro DESATIVADO.");

  } else if (strncmp(cmd, "filtro id ", 10) == 0) {

    filterId = strtoul(cmd + 10, NULL, 16);

    Serial.print("ID do filtro: 0x");
    Serial.println(filterId, HEX);

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

    blocked_0_to_2 = 0;
    blocked_2_to_0 = 0;

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

  // Também aceita comandos sem final de linha:
  // processa após 100 ms sem receber caracteres.
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
