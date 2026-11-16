#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define SPI_PORT spi0
// #define PIN_MISO 16
// #define PIN_CS   17
// #define PIN_SCK  18
// #define PIN_MOSI 19


#define SPI_PORT spi1
#define PIN_MISO 12
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

// Comando SPI READ e endereço do CANSTAT
#define MCP2515_READ   0x03
#define MCP2515_CANSTAT 0x0E

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    // Inicializa SPI1 a 1 MHz
    spi_init(SPI_PORT, 1000 * 1000);

    spi_set_format(
        SPI_PORT,
        8,
        SPI_CPOL_0,
        SPI_CPHA_0,
        SPI_MSB_FIRST
    );

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // CS controlado manualmente
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    printf("Teste SPI MCP2515 iniciado\n");

    while (true)
    {
        uint8_t tx[3] = {
            MCP2515_READ,
            MCP2515_CANSTAT,
            0x00
        };

        uint8_t rx[3] = {0};

        // Inicia transação
        gpio_put(PIN_CS, 0);

        spi_write_read_blocking(SPI_PORT, tx, rx, 3);

        // Finaliza transação
        gpio_put(PIN_CS, 1);

        printf("CANSTAT = 0x%02X\n", rx[2]);

        sleep_ms(1000);
    }
}