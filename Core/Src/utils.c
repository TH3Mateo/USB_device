//
// Created by M on 27/07/2023.
//

#include "utils.h"
#include <string.h> // memcpy

uint8_t calculate_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload) {
    uint16_t sum = cmd + len;
    for (uint8_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum % 256);
}

int create_packet(uint8_t *buffer_out, uint8_t cmd, const uint8_t *payload, uint8_t length) {
    if (length > (MAX_PACKET_SIZE - 4)) return -1;

    buffer_out[0] = START_BYTE;
    buffer_out[1] = cmd;
    buffer_out[2] = length;

    if (length > 0 && payload != NULL) {
        memcpy(&buffer_out[3], payload, length);
    }

    buffer_out[3 + length] = calculate_checksum(cmd, length, payload);

    return 4 + length;
}

int parse_packet(const uint8_t *buffer_in, int buffer_len, uint8_t *cmd_out, uint8_t *payload_out, uint8_t *len_out) {
    if (buffer_len < 4) return -1;
    if (buffer_in[0] != START_BYTE) return -2;

    uint8_t cmd = buffer_in[1];
    uint8_t len = buffer_in[2];

    if (buffer_len < (len + 4)) return -3;

    uint8_t checksum = buffer_in[3 + len];
    if (checksum != calculate_checksum(cmd, len, &buffer_in[3])) return -4;

    if (cmd_out) *cmd_out = cmd;
    if (len_out) *len_out = len;
    if (payload_out && len > 0) {
        memcpy(payload_out, &buffer_in[3], len);
    }

    return 0;
}

