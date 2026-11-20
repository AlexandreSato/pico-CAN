
#!/usr/bin/env python3

import time
from collections import defaultdict

from panda import Panda
from opendbc.car.structs import CarParams

# =========================================================
# CONFIGURACAO
# =========================================================

TX_BUS = 0
RX_BUS = 2

BITRATE = 500

STAGE_TIME = 5.0
DRAIN_TIME = 1.0

RATES = [
    10, 25, 50, 100, 200, 300,
    400, 500, 750, 1000, 1250,
    1500, 1750, 2000
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


def make_frame(stage, seq):
    can_id = ID_BASE + (seq % 0x200)

    check = (seq ^ (stage << 8) ^ 0xA55A) & 0xFFFF

    data = (
        bytes([MAGIC, stage])
        + seq.to_bytes(4, "big")
        + check.to_bytes(2, "big")
    )

    return can_id, data


# =========================================================
# RECEPCAO E CLASSIFICACAO
# =========================================================

def process_received(p):

    frames = p.can_recv()

    for addr, data, bus in frames:

        if len(data) != 8 or data[0] != MAGIC:
            continue

        stage = data[1]

        if stage not in stats:
            continue

        s = stats[stage]

        seq = int.from_bytes(data[2:6], "big")
        check = int.from_bytes(data[6:8], "big")

        expected_check = (
            seq ^ (stage << 8) ^ 0xA55A
        ) & 0xFFFF

        expected_id = ID_BASE + (seq % 0x200)

        # Eco local do Panda: TX_BUS + 128
        if bus == TX_BUS + 128:
            s["echo"] += 1
            continue

        # Transmissao rejeitada: TX_BUS + 192
        if bus == TX_BUS + 192:
            s["rejected"] += 1
            continue

        # Somente o bus fisico de destino conta como recebimento
        if bus != RX_BUS:
            continue

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
# EXECUTA UMA ETAPA
# =========================================================

def run_stage(p, stage, rate):

    s = stats[stage]

    interval = 1.0 / rate

    print()
    print("=" * 55)
    print(f"ETAPA {stage}: {rate} quadros/s")
    print("=" * 55)

    start = time.monotonic()
    end = start + STAGE_TIME

    next_send = start
    seq = 0

    while time.monotonic() < end:

        process_received(p)

        now = time.monotonic()

        if now >= next_send:

            can_id, data = make_frame(stage, seq)

            s["sent_seq"].add(seq)

            try:
                p.can_send(can_id, data, TX_BUS)
                s["sent"] += 1

            except Exception as e:
                print(f"ERRO DE TRANSMISSAO: {e}")
                return False

            seq += 1
            next_send += interval

            # Nao acumular envios atrasados
            if now - next_send > interval * 5:
                next_send = now + interval

        time.sleep(0.0002)

    # Aguarda os quadros pendentes chegarem
    drain_end = time.monotonic() + DRAIN_TIME

    while time.monotonic() < drain_end:
        process_received(p)
        time.sleep(0.001)

    lost = s["sent"] - s["received"]

    loss_percent = (
        lost * 100.0 / s["sent"]
        if s["sent"] else 0
    )

    elapsed = time.monotonic() - start

    print(f"Enviados pelo Panda:  {s['sent']}")
    print(f"Eco local (bus {TX_BUS + 128}): {s['echo']}")
    print(f"Recebidos no bus {RX_BUS}:       {s['received']}")
    print(f"Perdidos:             {lost}")
    print(f"Rejeitados:           {s['rejected']}")
    print(f"Duplicados:           {s['duplicate']}")
    print(f"Corrompidos:          {s['corrupt']}")
    print(f"Desconhecidos:        {s['unknown']}")
    print(f"Perda:                {loss_percent:.3f}%")
    print(f"Tempo:                {elapsed:.2f} s")

    if s["rejected"]:
        print("\nFALHA: Panda rejeitou transmissao.")
        return False

    if lost or s["duplicate"] or s["corrupt"]:
        print("\nFALHA: encaminhamento incompleto.")
        return False

    print(f"\nOK: todos os quadros chegaram ao bus {RX_BUS}.")
    return True


# =========================================================
# MAIN
# =========================================================

def main():

    p = Panda()

    print("Panda conectado")
    print("Serial:", p.get_serial()[0])
    print("Versao:", p.get_version())

    p.set_can_speed_kbps(TX_BUS, BITRATE)
    p.set_can_speed_kbps(RX_BUS, BITRATE)

    p.can_clear(0xFFFF)

    p.set_safety_mode(CarParams.SafetyModel.allOutput)

    print()
    print("GATEWAY STRESS TEST")
    print(f"TX: bus {TX_BUS}")
    print(f"RX: bus {RX_BUS}")
    print(f"Bitrate: {BITRATE} kbit/s")
    print("O eco local nao sera contado como recebimento.")

    try:

        for stage, rate in enumerate(RATES, start=1):

            success = run_stage(p, stage, rate)

            if not success:
                print("\nAumento de velocidade interrompido.")
                break

        print()
        print("=" * 55)
        print("TESTE FINALIZADO")
        print("=" * 55)

    except KeyboardInterrupt:
        print("\nInterrompido pelo usuario.")

    finally:
        p.set_safety_mode(CarParams.SafetyModel.noOutput)
        print("Transmissao desabilitada.")


if __name__ == "__main__":
    main()
