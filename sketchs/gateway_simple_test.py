
#!/usr/bin/env python3
import time
from collections import Counter

from panda import Panda
from opendbc.car.structs import CarParams

TX_BUS = 2
CAN_ID = 0x123
INTERVAL = 1.0

def describe_bus(bus):
    if 128 <= bus < 192:
        return f"ECHO local do bus {bus - 128}"
    if bus >= 192:
        return f"TX rejeitado / bus codificado {bus}"
    return f"RX fisico no bus {bus}"

def main():
    p = Panda()

    print("Panda:", p.get_serial()[0])
    print("Versao:", p.get_version())

    p.set_can_speed_kbps(0, 500)
    p.set_can_speed_kbps(2, 500)
    p.can_clear(0xFFFF)
    p.set_safety_mode(CarParams.SafetyModel.allOutput)

    sequence = 0
    last_send = 0
    counts = Counter()

    print("\nMonitor CAN + transmissao de teste")
    print(f"TX: bus={TX_BUS}, ID=0x{CAN_ID:X}")
    print("Exibindo todos os quadros recebidos...\n")

    try:
        while True:
            now = time.monotonic()

            if now - last_send >= INTERVAL:
                payload = b"TEST" + sequence.to_bytes(4, "big")

                p.can_send(CAN_ID, payload, TX_BUS)

                print(
                    f"\n[TX] bus={TX_BUS} "
                    f"ID=0x{CAN_ID:03X} "
                    f"SEQ={sequence} "
                    f"DATA={payload.hex(' ')}"
                )

                sequence += 1
                last_send = now

            frames = p.can_recv()

            for addr, data, bus in frames:
                counts[bus] += 1

                print(
                    f"[RX] bus={bus:3} "
                    f"({describe_bus(bus)}) "
                    f"ID=0x{addr:03X} "
                    f"LEN={len(data)} "
                    f"DATA={data.hex(' ')}"
                )

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\nTeste interrompido.")

    finally:
        print("\nResumo de quadros recebidos:")
        for bus, count in sorted(counts.items()):
            print(f"  bus {bus}: {count}")

        p.set_safety_mode(CarParams.SafetyModel.noOutput)
        print("Transmissao desabilitada.")

if __name__ == "__main__":
    main()

