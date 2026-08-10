#include "zigbee/tuya_secondary_mcu.h"
#include "hal/uart.h"

#include <string.h>

static uint8_t tuya_checksum(const uint8_t *buf, uint16_t len)
{
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    sum += buf[i];
  }
  return (uint8_t)(-sum);
}

int tuya_secondary_mcu_encode_frame(const tuya_secondary_mcu_frame_t *frame,
                                    uint8_t *out, uint16_t out_len,
                                    uint16_t *written)
{
  if (!frame || !out || out_len < 12)
  {
    return -1;
  }

  uint8_t data_len = (uint8_t)(2 + 1 + 1 + 2 + frame->value_len);
  uint8_t frame_buf[64];
  uint8_t idx = 0;

  frame_buf[idx++] = 0x55;
  frame_buf[idx++] = 0xAA;
  frame_buf[idx++] = 0x02;
  frame_buf[idx++] = frame->responder_seq;
  frame_buf[idx++] = 0x00;

  if (out_len < 11 + frame->value_len)
  {
    return -1;
  }

  frame_buf[idx++] = (uint8_t)frame->cmd;
  frame_buf[idx++] = (uint8_t)(data_len & 0xFF);
  frame_buf[idx++] = (uint8_t)(data_len >> 8);
  frame_buf[idx++] = frame->dpid;
  frame_buf[idx++] = frame->dp_type;
  frame_buf[idx++] = (uint8_t)(frame->value_len & 0xFF);
  frame_buf[idx++] = (uint8_t)((frame->value_len >> 8) & 0xFF);

  for (uint16_t i = 0; i < frame->value_len; i++)
  {
    frame_buf[idx++] = frame->value[i];
  }

  frame_buf[idx++] = tuya_checksum(frame_buf, idx);

  if (idx > out_len)
  {
    return -1;
  }

  memcpy(out, frame_buf, idx);
  if (written)
  {
    *written = idx;
  }

  return 0;
}

int tuya_secondary_mcu_decode_frame(const uint8_t *raw, uint16_t raw_len,
                                    tuya_secondary_mcu_frame_t *frame)
{
  if (!raw || !frame || raw_len < 12)
  {
    return -1;
  }

  memset(frame, 0, sizeof(*frame));

  if (raw[0] != 0x55 || raw[1] != 0xAA || raw[2] != 0x02)
  {
    return -1;
  }

  /* The comment field in the logs uses 55 AA 02 01 00 header + cmd etc. */
  frame->responder_seq = raw[3];
  frame->direction = raw[4];
  frame->cmd = raw[5];

  uint16_t dlen = (uint16_t)(raw[6] | (raw[7] << 8));
  uint16_t payload_off = 8;

  if (raw_len < payload_off + dlen + 1)
  {
    return -1;
  }

  frame->dpid = raw[payload_off++];
  frame->dp_type = raw[payload_off++];
  frame->value_len = (uint16_t)(raw[payload_off] | (raw[payload_off + 1] << 8));
  payload_off += 2;

  if (frame->value_len > sizeof(frame->value))
  {
    return -1;
  }

  if (frame->value_len > 0)
  {
    memcpy(frame->value, &raw[payload_off], frame->value_len);
    payload_off += frame->value_len;
  }

  frame->checksum = raw[payload_off];

  (void)dlen;
  return 0;
}

int tuya_secondary_mcu_send_dp(uint8_t dpid, uint8_t dp_type,
                               const void *value, uint16_t value_len,
                               uint8_t *out, uint16_t out_len,
                               uint16_t *written)
{
  tuya_secondary_mcu_frame_t frame;
  memset(&frame, 0, sizeof(frame));

  frame.responder_seq = 0x01;
  frame.direction = 0x00;
  frame.cmd = TUYA_MCU_CMD_WRITE;
  frame.dpid = dpid;
  frame.dp_type = dp_type;
  frame.value_len = value_len;
  memcpy(frame.value, value, value_len);

  return tuya_secondary_mcu_encode_frame(&frame, out, out_len, written);
}
