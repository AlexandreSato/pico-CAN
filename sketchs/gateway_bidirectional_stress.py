
#!/usr/bin/env python3

import time
from collections import defaultdict

from panda import Panda
from opendbc.car.structs import CarParams


# =========================================================
# CONFIGURACAO
# =========================================================

BUS_A = 0
BUS_B = 2

BITRATE = 500

STAGE_TIME = 5.0
DRAIN_TIME = 1.0

# Taxa POR SENTIDO
RATES = [
    1500,
    1750,
    2000,
    2500,
    3000,
    3500,
    4000,
]

ID_BASE = 0x500
MAGIC = 0xA7


# =========================================================
# ESTATISTICAS
# =========================================================

stats = defaultdict(lambda: {
    "sent": 0,
    "echo": 0,
    "received": 0,
    "rejected": 0,
    "duplicate": 0,
    "corrupt": 0,
    "unknown": 0,
    "sent_seq": set(),
    "received_seq": set(),
})


def make_frame(tx_bus, stage, seq):

    can_id = ID_BASE + (seq % 0x200)

    check = (
        seq
        ^ (stage << 8)
        ^ (tx_bus << 12)
        ^ 0xA55A
    ) & 0xFFFF

    data = (
        bytes([MAGIC, stage])
        + seq.to_bytes(4, "big")
        + check.to_bytes(2, "big")
    )

    return can_id, data


# =========================================================
# RECEPCAO E CLASSIFICACAO
# =========================================================

def process_received(p, stage):

    frames = p.can_recv()

    for addr, data, bus in frames:

        if len(data) != 8 or data[0] != MAGIC:
            continue

        frame_stage = data[1]

        if frame_stage != stage:
            continue

        seq = int.from_bytes(data[2:6], "big")
        check = int.from_bytes(data[6:8], "big")

        # Eco local do Panda
        if bus == BUS_A + 128:
            stats[(BUS_A, stage)]["echo"] += 1
            continue

        if bus == BUS_B + 128:
            stats[(BUS_B, stage)]["echo"] += 1
            continue

        # Transmissao rejeitada pelo Panda
        if bus == BUS_A + 192:
            stats[(BUS_A, stage)]["rejected"] += 1
            continue

        if bus == BUS_B + 192:
            stats[(BUS_B, stage)]["rejected"] += 1
            continue

        # Recepcao fisica no destino
        if bus == BUS_B:
            tx_bus = BUS_A
            rx_bus = BUS_B

        elif bus == BUS_A:
            tx_bus = BUS_B
            rx_bus = BUS_A

        else:
            continue

        s = stats[(tx_bus, stage)]

        expected_check = (
            seq
            ^ (stage << 8)
            ^ (tx_bus << 12)
            ^ 0xA55A
        ) & 0xFFFF

        expected_id = ID_BASE + (seq % 0x200)

        if seq not in s["sent_seq"]:
            s["unknown"] += 1
            continue

        if addr != expected_id or check != expected_check:
            s["corrupt"] += 1
            continue

        if seq in s["received_seq"]:
            s["duplicate"] += 1
            continue

        s["received_seq"].add(seq)
        s["received"] += 1


# =========================================================
# EXECUTA UMA ETAPA BIDIRECIONAL
# =========================================================

def run_stage(p, stage, rate):

    interval = 1.0 / rate

    s_a = stats[(BUS_A, stage)]
    s_b = stats[(BUS_B, stage)]

    print()
    print("=" * 65)
    print(f"ETAPA {stage}: {rate} quadros/s POR SENTIDO")
    print(f"Bus {BUS_A} -> Bus {BUS_B}")
    print(f"Bus {BUS_B} -> Bus {BUS_A}")
    print(f"Total nominal: {rate * 2} quadros/s")
    print("=" * 65)

    start = time.monotonic()
    end = start + STAGE_TIME

    next_send = {
        BUS_A: start,
        BUS_B: start,
    }

    seq = {
        BUS_A: 0,
        BUS_B: 0,
    }

    # -----------------------------------------------------
    # TRANSMISSAO
    # -----------------------------------------------------

    while time.monotonic() < end:

        process_received(p, stage)

        now = time.monotonic()

        # Transmite nos dois barramentos
        for tx_bus in (BUS_A, BUS_B):

            if now >= next_send[tx_bus]:

                can_id, data = make_frame(
                    tx_bus,
                    stage,
                    seq[tx_bus]
                )

                s = stats[(tx_bus, stage)]

                try:
                    p.can_send(can_id, data, tx_bus)

                    s["sent"] += 1
                    s["sent_seq"].add(seq[tx_bus])

                except Exception as e:

                    print(
                        f"ERRO TX bus {tx_bus}: {e}"
                    )

                    return False

                seq[tx_bus] += 1
                next_send[tx_bus] += interval

                # Nao acumular envios atrasados
                if now - next_send[tx_bus] > interval * 5:
                    next_send[tx_bus] = now + interval

        time.sleep(0.0001)

    # -----------------------------------------------------
    # TAXA REAL DO GERADOR
    # -----------------------------------------------------

    tx_elapsed = time.monotonic() - start

    actual_rate_a = s_a["sent"] / tx_elapsed if tx_elapsed else 0
    actual_rate_b = s_b["sent"] / tx_elapsed if tx_elapsed else 0

    print()
    print("--- TAXA REAL DO GERADOR ---")

    print(
        f"Bus {BUS_A}: {s_a['sent']} quadros / "
        f"{tx_elapsed:.3f}s = {actual_rate_a:.1f} quadros/s"
    )

    print(
        f"Bus {BUS_B}: {s_b['sent']} quadros / "
        f"{tx_elapsed:.3f}s = {actual_rate_b:.1f} quadros/s"
    )

    print(
        f"Total real: "
        f"{actual_rate_a + actual_rate_b:.1f} quadros/s"
    )

    # -----------------------------------------------------
    # DRENAGEM DOS QUADROS PENDENTES
    # -----------------------------------------------------

    drain_end = time.monotonic() + DRAIN_TIME

    while time.monotonic() < drain_end:

        process_received(p, stage)

        time.sleep(0.001)

    # -----------------------------------------------------
    # RESULTADO POR SENTIDO
    # -----------------------------------------------------

    success = True

    for tx_bus, rx_bus in (
        (BUS_A, BUS_B),
        (BUS_B, BUS_A),
    ):

        s = stats[(tx_bus, stage)]

        lost = s["sent"] - s["received"]

        loss_percent = (
            lost * 100.0 / s["sent"]
            if s["sent"] else 0
        )

        print()
        print(f"--- BUS {tx_bus} -> BUS {rx_bus} ---")
        print(f"Enviados:       {s['sent']}")
        print(f"Eco local:      {s['echo']}")
        print(f"Recebidos:      {s['received']}")
        print(f"Perdidos:       {lost}")
        print(f"Rejeitados:     {s['rejected']}")
        print(f"Duplicados:     {s['duplicate']}")
        print(f"Corrompidos:    {s['corrupt']}")
        print(f"Desconhecidos:  {s['unknown']}")
        print(f"Perda:          {loss_percent:.3f}%")

        if (
            lost
            or s["rejected"]
            or s["duplicate"]
            or s["corrupt"]
            or s["unknown"]
        ):
            success = False

    elapsed = time.monotonic() - start

    print()
    print(f"Tempo total com drenagem: {elapsed:.2f} s")

    if success:
        print("OK: os dois sentidos passaram nesta etapa.")
    else:
        print("FALHA: houve perda ou anomalia.")

    return success


# =========================================================
# MAIN
# =========================================================

def main():

    p = Panda()

    print("Panda conectado")
    print("Serial:", p.get_serial()[0])
    print("Versao:", p.get_version())

    p.set_can_speed_kbps(BUS_A, BITRATE)
    p.set_can_speed_kbps(BUS_B, BITRATE)

    p.can_clear(0xFFFF)

    p.set_safety_mode(CarParams.SafetyModel.allOutput)

    print()
    print("GATEWAY BIDIRECIONAL - STRESS TEST")
    print(f"Bus A: {BUS_A}")
    print(f"Bus B: {BUS_B}")
    print(f"Bitrate: {BITRATE} kbit/s")
    print("Transmissao simultanea nos dois barramentos.")

    try:

        for stage, rate in enumerate(RATES, start=1):

            success = run_stage(p, stage, rate)

            if not success:
                print("\nAumento de velocidade interrompido.")
                break

        print()
        print("=" * 65)
        print("TESTE FINALIZADO")
        print("=" * 65)

    except KeyboardInterrupt:

        print("\nInterrompido pelo usuario.")

    finally:

        p.set_safety_mode(CarParams.SafetyModel.noOutput)

        print("Transmissao desabilitada.")


if __name__ == "__main__":
    main()
