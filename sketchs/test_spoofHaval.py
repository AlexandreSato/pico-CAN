#!/usr/bin/env python3
import time
from panda import Panda
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.structs import CarParams

DBC_NAME = "gwm_haval_h6_mk3"   # nome gerado em values.py (sem _generated se for o caso)
BUS_TX = 2                      # bus do panda ligado ao seu gateway
BUS_RX = 0
ADDR = 0x147                   # RX_STEER_RELATED
SPOOF_TORQUE = 102             # mesmo valor do seu firmware
N_FRAMES = 200


def checksum(data, xor_output):
  crc = 0
  poly = 0x1D
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ xor_output


def make_rx_steer(packer, torque, counter):
  values = {
    "A_CRC_X61": 0,
    "A_BYPASSME_2": 0,
    "A_RX_STEER_REQUESTED": 1,
    "A_BYPASSME_1": 0,
    "A_COUNTER": counter & 0xF,
    "B_CRC_X61": 0,            # placeholder, recalculado abaixo
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

  packer = CANPacker(DBC_NAME)
  parser = CANParser(DBC_NAME, [(ADDR, 0)], BUS_RX)

  ok = fail = 0
  for i in range(N_FRAMES):
    torque = SPOOF_TORQUE if i % 2 == 0 else -SPOOF_TORQUE
    addr, dat, bus = make_rx_steer(packer, torque, i)

    # validação local do CRC antes de enviar
    assert dat[8] == checksum(dat[9:16], 0x61), "CRC local errado!"

    p.can_send(addr, dat, bus)
    time.sleep(0.01)  # ~100 Hz

    # lê de volta o que o gateway retransmitir)
    for raddr, rdat, rbus in p.can_recv():
      if raddr == ADDR and rbus == BUS_RX:
        parser.update([int(time.monotonic() * 1e9), [(raddr, rdat, rbus)]])
        got = parser.vl["RX_STEER_RELATED"]
        crc_ok = rdat[8] == checksum(rdat[9:16], 0x61)
        torque_ok = got["B_RX_DRIVER_TORQUE"] == torque
        status = "OK" if (crc_ok and torque_ok) else "FAIL"
        if crc_ok and torque_ok:
          ok += 1
        else:
          fail += 1
        print(f"[{status}] bus={rbus} torque={got['B_RX_DRIVER_TORQUE']} "
              f"(esp={torque}) crc=0x{rdat[8]:02X} crc_ok={crc_ok}")

  print(f"\nResultado: {ok} OK / {fail} FAIL")
  p.set_safety_mode(CarParams.SafetyModel.noOutput)


if __name__ == "__main__":
  main()