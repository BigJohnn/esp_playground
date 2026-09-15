#include "ir_nec.h"

size_t ir_nec_build(uint16_t address, uint16_t command, unsigned repeats,
                    ir_pulse_t *pulses, size_t capacity)
{
    const size_t count = 68 + (size_t)repeats * 4;
    if (!pulses || repeats > IR_NEC_MAX_REPEATS || capacity < count) {
        return 0;
    }
    size_t n = 0;
    uint32_t elapsed = 9000 + 4500;
    pulses[n++] = (ir_pulse_t){9000, true};
    pulses[n++] = (ir_pulse_t){4500, false};
    uint32_t bits = (uint32_t)address | ((uint32_t)command << 16);
    for (unsigned i = 0; i < 32; ++i) {
        uint32_t space = (bits & (1UL << i)) ? 1690 : 560;
        pulses[n++] = (ir_pulse_t){560, true};
        pulses[n++] = (ir_pulse_t){space, false};
        elapsed += 560 + space;
    }
    pulses[n++] = (ir_pulse_t){560, true};
    pulses[n++] = (ir_pulse_t){108000 - elapsed - 560, false};
    for (unsigned i = 0; i < repeats; ++i) {
        /* Measured Tivoli repeat waveform, see esphome/config/tivoli_ir.yaml. */
        pulses[n++] = (ir_pulse_t){9125, true};
        pulses[n++] = (ir_pulse_t){2314, false};
        pulses[n++] = (ir_pulse_t){526, true};
        pulses[n++] = (ir_pulse_t){96035, false};
    }
    return n;
}
