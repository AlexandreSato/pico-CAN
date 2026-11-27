#include <ACAN2517FD.h>
#include <SPI.h>

// Ajuste os pinos conforme seu hardware (ex: ESP32)
static const byte MCP2518_CS  = 17; // Mesmo pino CS que você usou
static const byte MCP2518_INT = 255; // Use o pino INT se tiver conectado, ou 255 para polling

ACAN2517FD CAN2(MCP2518_CS, SPI, MCP2518_INT);
uint32_t testSequence = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  SPI.setRX(16);  // GP16 (MISO)
  SPI.setCS(17);  // GP17 (CS)
  SPI.setSCK(18); // GP18 (SCK)
  SPI.setTX(19);  // GP19 (MOSI)
  SPI.begin();

  // Configura oscilador de 20MHz (igual ao seu MCP2518FD_20MHz)
  // Arbitration: 500 kbps | Data: 1 Mbps (fator x2)
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_20MHz, 500 * 1000, DataBitRateFactor::x2);
  
  // Garante que a FIFO de transmissão suporte pacotes de até 64 bytes
  settings.mDriverTransmitFIFOSize = 2;
  settings.mDriverReceiveFIFOSize = 2;

  const uint32_t errorCode = CAN2.begin(settings, [] { CAN2.isr(); });
  if (errorCode == 0) {
    Serial.println("CAN-FD 64B init ok!");
  } else {
    Serial.print("Erro na inicializacao: 0x");
    Serial.println(errorCode, HEX);
    while (1);
  }
}

void testSPI1() {
  CANFDMessage frame;
  frame.id = 0x456;
  frame.len = 64; // Define explicitamente 64 bytes
  frame.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH; // Ou CANFD_NO_BIT_RATE_SWITCH se o seu leitor não usar BRS

  // Preenche os 8 primeiros bytes ("SPI1" + sequência)
  frame.data[0] = 0x53; // 'S'
  frame.data[1] = 0x50; // 'P'
  frame.data[2] = 0x49; // 'I'
  frame.data[3] = 0x31; // '1'
  frame.data[4] = (byte)(testSequence >> 24);
  frame.data[5] = (byte)(testSequence >> 16);
  frame.data[6] = (byte)(testSequence >> 8);
  frame.data[7] = (byte)testSequence;

  // Preenche os 56 bytes restantes com 0xDE
  for (int i = 8; i < 64; i++) {
    frame.data[i] = 0xDE;
  }

  const bool ok = CAN2.tryToSend(frame);
  Serial.print("[SPI1 TX 64B] ID=0x456 SEQ=");
  Serial.print(testSequence);
  Serial.print(" OK=");
  Serial.println(ok);

  testSequence++;
}

void loop() {
  // Se estiver usando polling (MCP2518_INT = 255), chame poll() no loop
  CAN2.poll();

  testSPI1();
  delay(20); // ~50 Hz
}
