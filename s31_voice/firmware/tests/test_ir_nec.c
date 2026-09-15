/* Host-only waveform regression; no GPIO or appliance access.
 * cc -std=c11 -Wall -Wextra -Werror -I main tests/test_ir_nec.c main/ir_nec.c -o /tmp/test_ir_nec
 */
#include <assert.h>
#include <stdio.h>
#include "ir_nec.h"

static void check_waveform(uint16_t command, const uint8_t expected[4], unsigned repeats)
{
    ir_pulse_t pulses[IR_NEC_MAX_PULSES];
    size_t count = ir_nec_build(0x6B86, command, repeats, pulses, IR_NEC_MAX_PULSES);
    assert(count == 68 + repeats * 4);
    assert(pulses[0].mark && pulses[0].duration_us == 9000);
    assert(!pulses[1].mark && pulses[1].duration_us == 4500);
    /* Decode the actual waveform into bytes, including address byte order and
     * command inverse byte, against the existing ESPHome Tivoli configuration. */
    for (unsigned byte = 0; byte < 4; ++byte) {
        uint8_t decoded = 0;
        for (unsigned bit = 0; bit < 8; ++bit) {
            unsigned i = 2 + (byte * 8 + bit) * 2;
            assert(pulses[i].mark && pulses[i].duration_us == 560);
            assert(!pulses[i + 1].mark);
            assert(pulses[i + 1].duration_us == 560 || pulses[i + 1].duration_us == 1690);
            if (pulses[i + 1].duration_us == 1690) decoded |= 1u << bit;
        }
        assert(decoded == expected[byte]);
    }
    assert(pulses[66].mark && pulses[66].duration_us == 560);
    assert(!pulses[67].mark);
    uint32_t elapsed = 0;
    for (size_t i = 0; i < 68; ++i) elapsed += pulses[i].duration_us;
    assert(elapsed == 108000);
    for (unsigned repeat = 0; repeat < repeats; ++repeat) {
        size_t i = 68 + repeat * 4;
        assert(pulses[i].mark && pulses[i].duration_us == 9125);
        assert(!pulses[i + 1].mark && pulses[i + 1].duration_us == 2314);
        assert(pulses[i + 2].mark && pulses[i + 2].duration_us == 526);
        assert(!pulses[i + 3].mark && pulses[i + 3].duration_us == 96035);
        elapsed += pulses[i].duration_us + pulses[i + 1].duration_us
                 + pulses[i + 2].duration_us + pulses[i + 3].duration_us;
    }
    assert(elapsed == 108000 * (1 + repeats));
}

int main(void)
{
    const uint8_t up[] = {0x86, 0x6B, 0x00, 0xFF};
    const uint8_t down[] = {0x86, 0x6B, 0x45, 0xBA};
    unsigned repeat_counts[] = {0, 1, 30, 60};
    for (unsigned i = 0; i < sizeof(repeat_counts) / sizeof(repeat_counts[0]); ++i) {
        check_waveform(0xFF00, up, repeat_counts[i]);
        check_waveform(0xBA45, down, repeat_counts[i]);
    }
    ir_pulse_t sentinel = {123, true};
    assert(ir_nec_build(0, 0, 0, NULL, IR_NEC_MAX_PULSES) == 0);
    assert(ir_nec_build(0, 0, 1, &sentinel, 1) == 0);
    assert(sentinel.duration_us == 123 && sentinel.mark);
    assert(ir_nec_build(0, 0, IR_NEC_MAX_REPEATS + 1, &sentinel, IR_NEC_MAX_PULSES) == 0);
    assert(sentinel.duration_us == 123 && sentinel.mark);
    puts("PASS: Tivoli NEC bytes, repeat cadence and buffer bounds");
    return 0;
}
