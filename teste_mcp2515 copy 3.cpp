
#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"

// SPI1 - Raspberry Pi Pico
#define PIN_SCK   10
#define PIN_MOSI  11
#define PIN_MISO  12
#define PIN_CS    13

#define CAN_BITRATE CAN_500KBPS
#define CAN_CLOCK   MCP_8MHZ

MCP2515 can0(spi1, PIN_CS, PIN_MOSI, PIN_MISO, PIN_SCK, 1000000);

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

void printDebug(uint64_t last_time, MCP2515::ERROR reset_result) {
    printf("Último tempo: %llu\n", last_time);
    printf("Resultado do reset: %s\n", errorName(reset_result));
    fflush(stdout);
}

int main() {  
    stdio_init_all();  
    sleep_ms(2000);  
  
    MCP2515::ERROR r_reset   = can0.reset();  
    MCP2515::ERROR r_bitrate = can0.setBitrate(CAN_BITRATE, CAN_CLOCK);  
    MCP2515::ERROR r_mode    = can0.setNormalMode();  
  
    uint64_t last = time_us_64();  
    struct can_frame rx;  
  
    while (true) {  
        if (can0.readMessage(&rx) == MCP2515::ERROR_OK) {  
            printf("ID: 0x%03lX DLC: %d DATA: ",  
                   (unsigned long)(rx.can_id & CAN_SFF_MASK), rx.can_dlc);  
            for (int i = 0; i < rx.can_dlc; i++) printf("%02X ", rx.data[i]);  
            printf("\n");  
        }  
  
        if (time_us_64() - last >= 1000000) {  
            printf("init: reset=%s bitrate=%s mode=%s | status=0x%02X eflg=0x%02X\n",  
                   errorName(r_reset), errorName(r_bitrate), errorName(r_mode),  
                   can0.getStatus(), can0.getErrorFlags());  
            last = time_us_64();  
        }  
        fflush(stdout);  
    }  
}
