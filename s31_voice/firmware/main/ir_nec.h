#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IR_NEC_MAX_REPEATS 60
#define IR_NEC_MAX_PULSES (68 + 4 * IR_NEC_MAX_REPEATS)

typedef struct {
    uint32_t duration_us;
    bool mark;
} ir_pulse_t;

/* Complete NEC frame + repeats, LSB first, 108ms start-to-start cadence.
 * Returns zero on invalid arguments or insufficient capacity; never truncates. */
size_t ir_nec_build(uint16_t address, uint16_t command, unsigned repeats,
                    ir_pulse_t *pulses, size_t capacity);
