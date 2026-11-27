#!/usr/bin/env python3
import os
import random

from opendbc.car.structs import CarParams
from panda import Panda

def get_test_string():
  return b"test" + os.urandom(60) # exatamente 64 bytes

if __name__ == "__main__":
  p = Panda()
  p.set_safety_mode(CarParams.SafetyModel.allOutput)

  # CAN FD: bitrate da fase de dados 2Mbps nos 3 barramentos
  for bus in range(3):
    p.set_can_data_speed_kbps(bus, 2000)

  print("Spamming CAN FD messages on all buses...")
  while True:
    at = random.randint(0, 2000)
    st = get_test_string()
    bus = random.randint(0, 2)
    p.can_send(at, st, bus, fd=True)
    print(f"Sent CAN FD message on bus {bus}: len={len(st)} data={st.hex()}")