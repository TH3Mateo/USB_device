//
// Created by M on 27/07/2023.
//

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

#define START_BYTE 0xAA
#define MAX_PACKET_SIZE 64

// Oblicza checksum (modulo 256) z CMD + LEN + PAYLOAD
uint8_t calculate_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload);

// Tworzy pakiet: [START][CMD][LEN][PAYLOAD][CHECKSUM]
int create_packet(uint8_t *buffer_out, uint8_t cmd, const uint8_t *payload, uint8_t length);

// Parsuje pakiet, sprawdza poprawność i wypisuje dane
int parse_packet(const uint8_t *buffer_in, int buffer_len, uint8_t *cmd_out, uint8_t *payload_out, uint8_t *len_out);

#endif // UTILS_H
