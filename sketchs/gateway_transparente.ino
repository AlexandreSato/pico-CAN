
#include <SPI.h>
#include "mcp2518fd_can.h"

// ==========================================
// SPI0 - CAN bus0
// ==========================================
#define CAN0_CS  17

// ==========================================
// SPI1 - CAN bus2
// ==========================================
#define CAN2_CS  13

#define CAN_SPEED CAN_500KBPS
#define CAN_CLOCK MCP2518FD_20MHz

#define MAX_DATA_SIZE 8

// Dois controladores independentes
mcp2518fd CAN0(CAN0_CS);
mcp2518fd CAN2(CAN2_CS);

// Estatísticas
uint32_t forwarded_0_to_2 = 0;
uint32_t forwarded_2_to_0 = 0;
uint32_t tx_errors = 0;

// ==========================================
// Encaminha um quadro de SRC para DST
// ==========================================
void forwardFrame(mcp2518fd &src, mcp2518fd &dst,
                  uint32_t &counter) {

    if (src.checkReceive() != CAN_MSGAVAIL) {
        return;
    }

    byte len = 0;
    byte data[MAX_DATA_SIZE] = {};

    // Recebe o quadro
    src.readMsgBuf(&len, data);

    // Recupera os metadados
    unsigned long id = src.getCanId();
    byte ext = src.isExtendedFrame();
    byte rtr = src.isRemoteRequest();

    // Encaminha sem modificar o conteúdo
    byte result = dst.trySendMsgBuf(
        id,
        ext,
        rtr,
        len,
        data
    );

    if (result == CAN_OK) {
        counter++;
    } else {
        tx_errors++;
    }
}

// ==========================================
// Inicialização
// ==========================================
void setup() {

    Serial.begin(115200);

    // SPI0 - bus0
    SPI.setSCK(18);
    SPI.setRX(16);
    SPI.setTX(19);
    SPI.begin();

    // SPI1 - bus2
    SPI1.setSCK(10);
    SPI1.setRX(12);
    SPI1.setTX(11);
    SPI1.begin();

    // Associar cada controlador ao seu SPI
    CAN0.setSPI(&SPI);
    CAN2.setSPI(&SPI1);

    // Inicializar CAN0
    while (CAN_OK != CAN0.begin(
        CAN_SPEED, CAN_CLOCK, false)) {
        Serial.println("CAN0 init fail");
        delay(500);
    }

    // Inicializar CAN2
    while (CAN_OK != CAN2.begin(
        CAN_SPEED, CAN_CLOCK, false)) {
        Serial.println("CAN2 init fail");
        delay(500);
    }

    // Modo normal nos dois barramentos
    CAN0.setMode(CAN_NORMAL_MODE);
    CAN2.setMode(CAN_NORMAL_MODE);

    Serial.println("Gateway iniciado!");
    Serial.println("CAN0 <-> CAN2");
}

// ==========================================
// Loop principal
// ==========================================
void loop() {

    // bus0 -> bus2
    forwardFrame(CAN0, CAN2, forwarded_0_to_2);

    // bus2 -> bus0
    forwardFrame(CAN2, CAN0, forwarded_2_to_0);

}
