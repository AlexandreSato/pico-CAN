
#!/usr/bin/env python3
import time
from panda import Panda
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.structs import CarParams

DBC_NAME = "gwm_haval_h6_mk3"
BUS_TX = 2
BUS_RX = 0
ADDR = 0x147
LIMIT = 102

# Todos os cenários da tabela-verdade
TEST_CASES = [
    -1024,
    -200,
    -103,
    -102,
    -101,
    -100,
    -50,
    -1,
    0,
    1,
    50,
    100,
    101,
    102,
    103,
    200,
    1023,
]

DELAY = 0.05


def checksum(data, xor_output):
    crc = 0
    poly = 0x1D

    for byte in data:
        crc ^= byte

        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF

    return crc ^ xor_output


def expected_torque(torque):
    if torque >= 0:
        return max(torque, LIMIT)
    else:
        return min(torque, -LIMIT)


def make_rx_steer(packer, torque, counter):
    values = {
        "A_CRC_X61": 0,
        "A_BYPASSME_2": 0,
        "A_RX_STEER_REQUESTED": 1,
        "A_BYPASSME_1": 0,
        "A_COUNTER": counter & 0xF,
        "B_CRC_X61": 0,
        "B_RX_DRIVER_TORQUE": torque,
        "B_BYPASSME_1": 0,
        "B_BYPASSME_2": 0,
        "B_RX_EPS_TORQUE": 0,
        "B_COUNTER": counter & 0xF,
        "B_BYPASSME_4": 0,
        "EPS_FAULT_PERMANENT": 0,
        "B_BYPASSME_3": 0,
    }

    dat = packer.make_can_msg("RX_STEER_RELATED", 0, values)[1]

    values["B_CRC_X61"] = checksum(dat[9:16], 0x61)

    return packer.make_can_msg("RX_STEER_RELATED", BUS_TX, values)


def main():
    p = Panda()
    p.set_safety_mode(CarParams.SafetyModel.allOutput)
    p.set_can_data_speed_kbps(BUS_TX, 2000)

    packer = CANPacker(DBC_NAME)
    parser = CANParser(DBC_NAME, [(ADDR, 0)], BUS_RX)

    ok = 0
    fail = 0
    pending = {}

    print("\n========== TESTE DE LIMITE DE TORQUE ==========")
    print(f"Limite configurado: +/-{LIMIT}")
    print(f"Total de cenarios: {len(TEST_CASES)}\n")

    try:
        for counter, torque in enumerate(TEST_CASES):
            expected = expected_torque(torque)

            addr, dat, bus = make_rx_steer(
                packer, torque, counter
            )

            # Confere o CRC do quadro antes do envio
            tx_crc_ok = (
                dat[8] == checksum(dat[9:16], 0x61)
            )

            if not tx_crc_ok:
                print(f"[TX CRC FAIL] torque={torque}")
                fail += 1
                continue

            pending[counter] = {
                "tx": torque,
                "expected": expected,
                "data": dat,
            }

            print(
                f"\nTX torque={torque:+5d} "
                f"esperado={expected:+5d} "
                f"CRC=0x{dat[8]:02X}"
            )

            p.can_send(addr, dat, bus, fd=True)

            # Aguarda o quadro correspondente
            deadline = time.monotonic() + 0.5
            received = False

            while time.monotonic() < deadline:
                for raddr, rdat, rbus in p.can_recv():

                    if raddr != ADDR or rbus != BUS_RX:
                        continue

                    if len(rdat) < 16:
                        print(
                            f"[FAIL] Comprimento invalido: "
                            f"{len(rdat)} bytes"
                        )
                        fail += 1
                        received = True
                        break

                    parser.update([
                        int(time.monotonic() * 1e9),
                        [(raddr, rdat, rbus)]
                    ])

                    got = parser.vl["RX_STEER_RELATED"]
                    dbc_torque = got["B_RX_DRIVER_TORQUE"]

                    # Extrai independentemente o torque recebido
                    raw = (
                        ((rdat[9] & 0x7F) << 4)
                        | ((rdat[10] >> 4) & 0x0F)
                    )

                    raw_signed = (
                        raw - 0x800 if raw & 0x400 else raw
                    )

                    # Valida CRC recebido
                    crc_calculated = checksum(rdat[9:16], 0x61)
                    crc_ok = rdat[8] == crc_calculated

                    torque_ok = raw_signed == expected
                    dbc_ok = dbc_torque == expected

                    success = crc_ok and torque_ok and dbc_ok

                    if success:
                        ok += 1
                    else:
                        fail += 1

                    status = "OK" if success else "FAIL"

                    print(
                        f"RX torque={raw_signed:+5d} "
                        f"DBC={dbc_torque:+6.1f} "
                        f"CRC_RX=0x{rdat[8]:02X} "
                        f"CRC_CALC=0x{crc_calculated:02X}"
                    )

                    print(
                        f"[{status}] "
                        f"TX={torque:+5d} "
                        f"ESPERADO={expected:+5d} "
                        f"RX={raw_signed:+5d} "
                        f"CRC_OK={crc_ok} "
                        f"DBC_OK={dbc_ok}"
                    )

                    received = True
                    break

                if received:
                    break

                time.sleep(0.005)

            if not received:
                print(
                    f"[TIMEOUT] Nenhuma resposta para "
                    f"torque={torque:+d}"
                )
                fail += 1

            time.sleep(DELAY)

    finally:
        p.set_safety_mode(CarParams.SafetyModel.noOutput)

    print("\n========== RESULTADO FINAL ==========")
    print(f"Total de testes: {len(TEST_CASES)}")
    print(f"OK: {ok}")
    print(f"FAIL: {fail}")

    if fail == 0 and ok == len(TEST_CASES):
        print("\nTODOS OS CENARIOS PASSARAM!")
    else:
        print("\nEXISTEM CENARIOS QUE PRECISAM SER INVESTIGADOS.")


if __name__ == "__main__":
    main()
