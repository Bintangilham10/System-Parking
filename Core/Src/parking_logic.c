#include "parking_logic.h"

void ParkingLogic_Evaluate(const ParkingSensorData *sensor,
                           uint8_t gateOpen,
                           ParkingDecision *decision)
{
  if (sensor == 0 || decision == 0)
  {
    return;
  }

  decision->parkingFull = (sensor->slot1Occupied && sensor->slot2Occupied) ? 1 : 0;

  /*
    FULL + ENTRY/EXIT aktif dianggap blocked sesuai kode sebelumnya:
    LCD menampilkan FULL - BLOCKED dan gate tetap close.
  */
  decision->gateBlocked = (decision->parkingFull &&
                          (sensor->entryDetected || sensor->exitDetected)) ? 1 : 0;

  /*
    Gate boleh open jika parkir belum penuh dan ada trigger:
    entry, exit, atau safety.
  */
  if (!decision->parkingFull &&
      (sensor->entryDetected || sensor->exitDetected || sensor->safetyDetected))
  {
    decision->allowGateOpen = 1;
  }
  else
  {
    decision->allowGateOpen = 0;
  }

  /*
    Gate tetap ditahan open jika:
    - parkir belum penuh dan entry/exit masih aktif
    - gate sudah open dan safety masih mendeteksi objek
  */
  if ((!decision->parkingFull &&
      (sensor->entryDetected || sensor->exitDetected)) ||
      (gateOpen && sensor->safetyDetected))
  {
    decision->holdGateOpen = 1;
  }
  else
  {
    decision->holdGateOpen = 0;
  }

  decision->systemSafe = decision->gateBlocked ? 0 : 1;
}
