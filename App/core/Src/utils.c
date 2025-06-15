/**
 * @file        utils.c
 * @brief       Utility functions for packet parsing, conversion, and checksum calculation.
 *
 * @details     Contains helpers for:
 *              - Byte-to-binary conversion
 *              - Hexadecimal to decimal conversion
 *              - Packet creation and parsing
 *              - Checksum computation
 *
 * @authors     Mateusz Turycz and Aleksander Uliczny
 * @date        2025-05-21
 */

#include "utils.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "configurables.h"

/**
 * @brief Converts a byte into an array of bits (LSB first).
 *
 * @param byte The input byte.
 * @param bits Pointer to an 8-byte array to store the bit values.
 * @return Pointer to the bits array.
 */
const unsigned char* HexToBin(uint8_t byte, unsigned char* bits) {
    for (int i = 0; i < 8; i++) {
        bits[i] = (byte >> i) & 1;
    }
    return bits;
}

/**
 * @brief Converts a hexadecimal array into a decimal float.
 *
 * @param hex Pointer to the hex array.
 * @param byte_count Number of bytes in the array.
 * @return Decimal float in range 0.0 – 1.0 (based on 16-bit resolution).
 */
float HexToDec(uint8_t* hex, uint8_t byte_count) {
    int dec = 0;
    for (int i = 0; i < byte_count; i++) {
        dec += hex[i] << (8 * (byte_count - i - 1));
    }
    return dec / pow(2, 16);
}

/**
 * @brief Calculates checksum for a packet using sum modulo 256.
 *
 * @param cmd Command byte.
 * @param len Length of the payload.
 * @param payload Pointer to payload bytes.
 * @return Checksum byte.
 */
uint8_t calculate_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload) {
    uint16_t sum = cmd + len;
    for (uint8_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum % 256);
}

/**
 * @brief Constructs a packet from command and payload.
 *
 * @param buffer_out Output buffer for the complete packet.
 * @param cmd Command byte.
 * @param payload Pointer to payload data.
 * @param length Length of payload.
 * @return Total length of the packet, or -1 on error.
 */
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

/**
 * @brief Parses a received packet into command, payload, and length.
 *
 * @param buffer_in Input buffer containing the packet.
 * @param buffer_len Length of the input buffer.
 * @param cmd_out Pointer to store extracted command.
 * @param payload_out Pointer to store extracted payload.
 * @param len_out Pointer to store payload length.
 * @return 0 on success, negative value on error.
 */
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
