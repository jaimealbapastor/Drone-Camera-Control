#include <sbus.h>

bfs::SbusTx sbus_tx(&Serial2, 16, 17, true);
bfs::SbusData data;


void setup() {
  sbus_tx.Begin();

  data.lost_frame = false;
  data.failsafe = false;
  data.ch17 = false;
  data.ch18 = false;

  uint16_t ex_data[bfs::SbusData::NUM_CH] = { 1400, 1200, 1600, 1800, 1234, 1456, 1566, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  for (int i = 0; i < bfs::SbusData::NUM_CH; ++i) {
    data.ch[i] = ex_data[i];
  }

  sbus_tx.data(data);
}

void loop() {

  sbus_tx.Write();

  // Wait for a short interval before sending the next frame
  delay(14);  // 14ms for a typical 70Hz frame rate
}
