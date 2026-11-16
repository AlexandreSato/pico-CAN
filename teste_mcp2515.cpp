
#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"

// SPI0 - Raspberry Pi Pico
#define PIN_SCK   18
#define PIN_MOSI  19
#define PIN_MISO  16
#define PIN_CS    17

#define CAN_BITRATE CAN_500KBPS
#define CAN_CLOCK   MCP_8MHZ

// SPI a 1 MHz para facilitar o diagnóstico inicial
MCP2515 can0(spi0, PIN_CS, PIN_MOSI, PIN_MISO, PIN_SCK, 1000000);

struct can_frame rx;
struct can_frame tx;
uint64_t tx_frames = 0;  
uint64_t tx_errors = 0;

// Contadores
uint64_t total_frames = 0;
uint64_t standard_frames = 0;
uint64_t extended_frames = 0;
uint64_t remote_frames = 0;
uint64_t error_frames = 0;
uint64_t read_errors = 0;
uint64_t no_message = 0;

// --------------------------------------------------
// Decodificação dos retornos da biblioteca
// --------------------------------------------------

const char* errorName(MCP2515::ERROR error) {
    switch (error) {
        case MCP2515::ERROR_OK:        return "ERROR_OK";
        case MCP2515::ERROR_FAIL:      return "ERROR_FAIL";
        case MCP2515::ERROR_ALLTXBUSY: return "ERROR_ALLTXBUSY";
        case MCP2515::ERROR_FAILINIT:  return "ERROR_FAILINIT";
        case MCP2515::ERROR_FAILTX:    return "ERROR_FAILTX";
        case MCP2515::ERROR_NOMSG:     return "ERROR_NOMSG";
        default:                       return "ERRO_DESCONHECIDO";
    }
}

// --------------------------------------------------
// Decodificação dos flags de erro EFLG
// --------------------------------------------------

void printErrorFlags(uint8_t flags) {
    printf("EFLG: 0x%02X", flags);

    if (flags == 0) {
        printf(" (sem flags de erro)");
    } else {
        if (flags & 0x80) printf(" RX1OVR");
        if (flags & 0x40) printf(" RX0OVR");
        if (flags & 0x20) printf(" TXBO");
        if (flags & 0x10) printf(" TXEP");
        if (flags & 0x08) printf(" RXEP");
        if (flags & 0x04) printf(" TXWAR");
        if (flags & 0x02) printf(" RXWAR");
        if (flags & 0x01) printf(" EWARN");
    }

    printf("\n");
}

// --------------------------------------------------
// Impressão de um frame CAN
// --------------------------------------------------

void printFrame(const struct can_frame* frame) {
    uint32_t id = frame->can_id;

    printf("[FRAME %llu] ", (unsigned long long)total_frames);

    if (id & CAN_ERR_FLAG) {
        printf("FRAME DE ERRO | ID=0x%08lX\n",
               (unsigned long)(id & CAN_ERR_MASK));
        error_frames++;
        return;
    }

    if (id & CAN_EFF_FLAG) {
        printf("EXT | ID=0x%08lX",
               (unsigned long)(id & CAN_EFF_MASK));
        extended_frames++;
    } else {
        printf("STD | ID=0x%03lX",
               (unsigned long)(id & CAN_SFF_MASK));
        standard_frames++;
    }

    printf(" | DLC=%u", frame->can_dlc);

    if (id & CAN_RTR_FLAG) {
        printf(" | RTR");
        remote_frames++;
    } else {
        printf(" | DATA=");

        for (int i = 0; i < frame->can_dlc; i++) {
            printf("%02X ", frame->data[i]);
        }
    }

    printf("\n");
}

// --------------------------------------------------
// Diagnóstico periódico do controlador
// --------------------------------------------------

void printDiagnostics() {
    uint8_t status = can0.getStatus();
    uint8_t interrupts = can0.getInterrupts();
    uint8_t flags = can0.getErrorFlags();

    uint8_t rec = can0.errorCountRX();
    uint8_t tec = can0.errorCountTX();

    printf("\n");
    printf("========== DIAGNOSTICO MCP2515 ==========\n");

    printf("STATUS: 0x%02X", status);
    if (status & 0x01) printf(" RX0IF");
    if (status & 0x02) printf(" RX1IF");
    printf("\n");

    printf("CANINTF: 0x%02X", interrupts);
    if (interrupts & 0x01) printf(" RX0IF");
    if (interrupts & 0x02) printf(" RX1IF");
    if (interrupts & 0x04) printf(" TX0IF");
    if (interrupts & 0x08) printf(" TX1IF");
    if (interrupts & 0x10) printf(" TX2IF");
    if (interrupts & 0x20) printf(" ERRIF");
    if (interrupts & 0x40) printf(" WAKIF");
    if (interrupts & 0x80) printf(" MERRF");
    printf("\n");

    printf("CANCTRL: 0x%02X\n", can0.debugReadCANCTRL());
    printf("CANSTAT: 0x%02X\n", can0.debugReadCANSTAT());

    printErrorFlags(flags);

    printf("REC (erros de recepcao): %u\n", rec);
    printf("TEC (erros de transmissao): %u\n", tec);

    printf("-----------------------------------------\n");
    printf("Frames recebidos:  %llu\n",
           (unsigned long long)total_frames);
    printf("  Standard:        %llu\n",
           (unsigned long long)standard_frames);
    printf("  Extended:        %llu\n",
           (unsigned long long)extended_frames);
    printf("  RTR:             %llu\n",
           (unsigned long long)remote_frames);
    printf("  Error frames:    %llu\n",
           (unsigned long long)error_frames);
    printf("Leituras sem frame: %llu\n",
           (unsigned long long)no_message);
    printf("Erros de leitura:  %llu\n",
           (unsigned long long)read_errors);
    printf("Frames transmitidos: %llu\n",
           (unsigned long long)tx_frames);  
    printf("Erros de TX:         %llu\n",
           (unsigned long long)tx_errors);

    printf("=========================================\n\n");
    fflush(stdout);
}

void sendSpam(uint32_t counter) {  
    tx.can_id  = 0x123;          // ID standard 11 bits  
    tx.can_dlc = 8;  
    tx.data[0] = 0xDE;  
    tx.data[1] = 0xAD;  
    tx.data[2] = 0xBE;  
    tx.data[3] = 0xEF;  
    tx.data[4] = (counter >> 24) & 0xFF;  
    tx.data[5] = (counter >> 16) & 0xFF;  
    tx.data[6] = (counter >> 8)  & 0xFF;  
    tx.data[7] = counter & 0xFF;  
  
    MCP2515::ERROR r = can0.sendMessage(&tx);  
  
    if (r == MCP2515::ERROR_OK) {  
        tx_frames++;  
    } else {  
        tx_errors++;  
        printf("[ERRO TX] %s (%d)\n", errorName(r), r);  
    }  
}

// --------------------------------------------------
// Inicialização
// --------------------------------------------------

int main() {
    stdio_init_all();
    sleep_ms(15000);

    printf("\n");
    printf("=========================================\n");
    printf("       MCP2515 - CAN MONITOR\n");
    printf("=========================================\n");

    printf("[1] Inicializando SPI0...\n");
    printf("    SCK  = GP%d\n", PIN_SCK);
    printf("    MOSI = GP%d\n", PIN_MOSI);
    printf("    MISO = GP%d\n", PIN_MISO);
    printf("    CS   = GP%d\n", PIN_CS);
    printf("    SPI  = 1 MHz\n");

    printf("[2] Enviando RESET ao MCP2515...\n");

    MCP2515::ERROR result = can0.reset();

    printf("    Retorno: %s (%d)\n", errorName(result), result);

    if (result != MCP2515::ERROR_OK) {
        printf("FALHA: MCP2515 nao inicializou.\n");
        while (true) sleep_ms(1000);
    }

    printf("[3] Configurando CAN...\n");
    printf("    Bitrate: 500 kbit/s\n");
    printf("    Cristal: 8 MHz\n");

    result = can0.setBitrate(CAN_BITRATE, CAN_CLOCK);

    printf("    Retorno: %s (%d)\n", errorName(result), result);

    if (result != MCP2515::ERROR_OK) {
        printf("FALHA: nao foi possivel configurar o bitrate.\n");
        while (true) sleep_ms(1000);
    }

    printf("[4] Ativando modo NORMAL...\n");

    result = can0.setNormalMode();

    printf("    Retorno: %s (%d)\n", errorName(result), result);

    if (result != MCP2515::ERROR_OK) {
        printf("FALHA: nao foi possivel ativar o modo NORMAL.\n");
        while (true) sleep_ms(1000);
    }

    sleep_ms(10);
    uint8_t ctrl = can0.debugReadCANCTRL();
    uint8_t stat = can0.debugReadCANSTAT();
    printf("CANCTRL: 0x%02X | REQOP: 0x%02X\n",
        ctrl, ctrl & 0xE0);
    printf("CANSTAT: 0x%02X | OPMOD: 0x%02X\n",
        stat, stat & 0xE0);
    printDiagnostics();

    printf("\n");
    printf("[OK] MCP2515 inicializado.\n");
    printf("[OK] Monitoramento CAN iniciado.\n");
    printf("Aguardando mensagens...\n\n");

    fflush(stdout);

    // --------------------------------------------------
    // Recepção
    // --------------------------------------------------

    uint64_t last_diagnostic = time_us_64();
    uint64_t last_tx = time_us_64();  
    uint32_t tx_counter = 0;  

    while (true) {
        MCP2515::ERROR read_result = can0.readMessage(&rx);

        if (read_result == MCP2515::ERROR_OK) {
            total_frames++;
            printFrame(&rx);
        }
        else if (read_result == MCP2515::ERROR_NOMSG) {
            no_message++;
        }
        else {
            read_errors++;

            printf("[ERRO DE LEITURA] %s (%d)\n",
                   errorName(read_result), read_result);
            fflush(stdout);
        }

        // Spam a cada 100 ms  
        uint64_t now = time_us_64();
        if (now - last_tx >= 100000) {  
            sendSpam(tx_counter++);  
            last_tx = now;  
        }  
        
        // Diagnóstico a cada segundo
        now = time_us_64();  
        if (now - last_diagnostic >= 1000000) {  
            printDiagnostics();  
            last_diagnostic = now;  
        }  

        // if (tx_errors > 50) {           // buffers travados  
        //     can0.setConfigMode();       // limpa TXREQ pendentes  
        //     can0.setNormalMode();  
        //     tx_errors = 0;  
        // }
    }

    return 0;
}
