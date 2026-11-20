/*
 * demo: CAN-BUS Shield, receive all frames and print all fields id/type/data
 * to receive frame fastly, a poll in loop() is required.
 *
 * Copyright (C) 2020 Seeed Technology Co.,Ltd.
 */
#include <SPI.h>

// #define CAN_2515
#define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
// SPI 0
// const int SPI_CS_PIN = 17;
// const int CAN_INT_PIN = 20;

// SPI1
const int SPI_CS_PIN = 13;
//const int CAN_INT_PIN = 21;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin

// To TEST MCP2518FD CAN2.0 data transfer
#define MAX_DATA_SIZE 8
// To TEST MCP2518FD CANFD data transfer, uncomment below lines
// #undef  MAX_DATA_SIZE
// #define MAX_DATA_SIZE 64

#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8
#endif

// Transmissão periódica
uint32_t txId = 0x123;
byte txData[4] = {0xDE, 0xAD, 0xBE, 0xEF};

unsigned long lastTx = 0;
const unsigned long txInterval = 100; // 100 ms

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while (!SERIAL_PORT_MONITOR) {}

    // #if MAX_DATA_SIZE > 8
    #if true
    /*
     * To compatible with MCP2515 API,
     * default mode is CAN_CLASSIC_MODE
     * Now set to CANFD mode.
     */
     #endif

    //SPI0
    //SPI.setSCK(18); SPI.setRX(16); SPI.setTX(19);  
    //SPI.begin(); 
    
    //SPI1 
    SPI1.setSCK(10); SPI1.setRX(12); SPI1.setTX(11);  
    SPI1.begin();
    CAN.setSPI(&SPI1);
    //  while (CAN_OK != CAN.begin(CAN_500K_1M, MCP2518FD_20MHz)) {             // init can bus : baudrate = 500k
     while (CAN_OK != CAN.begin(CAN_500KBPS, MCP2518FD_20MHz, false)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println(F("CAN init fail, retry..."));
        delay(100);
    }

    // const int modo_desejado = CAN_LISTEN_ONLY_MODE;
    const int modo_desejado = CAN_NORMAL_MODE;
    byte r = CAN.setMode(modo_desejado);
    SERIAL_PORT_MONITOR.print("setMode ret = "); SERIAL_PORT_MONITOR.println((int8_t)r);  
    delay(10);
    SERIAL_PORT_MONITOR.print("Modo solicitado = ");
    SERIAL_PORT_MONITOR.println(modo_desejado);
    SERIAL_PORT_MONITOR.print("Mode = ");  
    SERIAL_PORT_MONITOR.println(CAN.getMode());
    while (CAN.getMode() != modo_desejado) {
        if (Serial.available()) {
            String comando = Serial.readStringUntil('\n');
            comando.trim(); // Remove espaços e quebras de linha

            if (comando == "BOOT") {
                Serial.println("Entrando em modo BOOTSEL...");
                delay(100); // Tempo para enviar a mensagem antes de desligar a USB
                rp2040.rebootToBootloader(); // Reinicia direto no modo BOOTSEL
            } else if (comando == "RESET") {
                Serial.println("Reiniciando normalmente...");
                delay(100);
                rp2040.reboot(); // Faz apenas um reset comum (como o pino RUN)
            } else {
                Serial.println("Comando desconhecido. Use 'BOOT' ou 'RESET'.");
                break; // Sai do loop se o comando for desconhecido
            }
        }
        delay(1);
    }
    if (CAN.getMode() == CAN_LISTEN_ONLY_MODE) {
        SERIAL_PORT_MONITOR.println("Modo CAN LISTEN ONLY ativado!");
    } else if (CAN.getMode() == CAN_NORMAL_MODE) {
        SERIAL_PORT_MONITOR.println("Modo CAN NORMAL ativado!");
    }
    SERIAL_PORT_MONITOR.print("Mode = ");  
    SERIAL_PORT_MONITOR.println(CAN.getMode());
    SERIAL_PORT_MONITOR.println(F("CAN init ok!"));
}

uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;
byte cdata[MAX_DATA_SIZE] = {0};
uint8_t err; 

void loop() {
    if (Serial.available()) {
        String comando = Serial.readStringUntil('\n');
        comando.trim(); // Remove espaços e quebras de linha
    
        if (comando == "BOOT") {
          Serial.println("Entrando em modo BOOTSEL...");
          delay(100); // Tempo para enviar a mensagem antes de desligar a USB
          rp2040.rebootToBootloader(); // Reinicia direto no modo BOOTSEL
        } else if (comando == "RESET") {
          Serial.println("Reiniciando normalmente...");
          delay(100);
          rp2040.reboot(); // Faz apenas um reset comum (como o pino RUN)
        }
        else {
          Serial.println("Comando desconhecido. Use 'BOOT' ou 'RESET'.");
        }
    }

    // Transmite DE AD BE EF a cada 100 ms
    if (millis() - lastTx >= txInterval) {
        lastTx = millis();

        byte result = CAN.sendMsgBuf(txId, 0, 4, txData);

        if (result == CAN_OK) {
            SERIAL_PORT_MONITOR.println("TX: 123#DEADBEEF");
        } else {
            SERIAL_PORT_MONITOR.print("TX ERROR: ");
            SERIAL_PORT_MONITOR.println(result);
        }
    }

    // imprime erro periodicamente, mesmo sem mensagem  
    static unsigned long lastErr = 0;  
    if (millis() - lastErr > 1000) {  
        lastErr = millis();  
        CAN.checkError(&err);  
        SERIAL_PORT_MONITOR.print("ERR=0x"); SERIAL_PORT_MONITOR.println(err, HEX);  
    }  

    // check if data coming
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return;
    }

    char prbuf[32 + MAX_DATA_SIZE * 3];
    int i, n;

    unsigned long t = millis();
    // read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, cdata);

    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) |
           (CAN.isRemoteRequest() << 1);
    /*
     * MCP2515(or this driver) could not handle properly
     * the data carried by remote frame
     */

    n = sprintf(prbuf, "%04lu.%03d ", t / 1000, int(t % 1000));
    /* Displayed type:
     *
     * 0x00: standard data frame
     * 0x02: extended data frame
     * 0x30: standard remote frame
     * 0x32: extended remote frame
     */
    static const byte type2[] = {0x00, 0x02, 0x30, 0x32};
    n += sprintf(prbuf + n, "RX: [%08lX](%02X) ", (unsigned long)id, type2[type]);
    // n += sprintf(prbuf, "RX: [%08lX](%02X) ", id, type);

    for (i = 0; i < len; i++) {
        n += sprintf(prbuf + n, "%02X ", cdata[i]);
    }
    SERIAL_PORT_MONITOR.println(prbuf);
}
// END FILE
