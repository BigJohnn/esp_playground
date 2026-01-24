import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import microphone
from esphome.const import CONF_ID
try:
    from esphome.components import audio as audio_mod
    _HAS_AUDIO_LIMITS = hasattr(audio_mod, "set_stream_limits")
except Exception:
    audio_mod = None
    _HAS_AUDIO_LIMITS = False

AUTO_LOAD = []
DEPENDENCIES = []

ns = cg.esphome_ns.namespace("korvo_tdm_mic")
KorvoTDMMicrophone = ns.class_("KorvoTDMMicrophone", cg.Component, microphone.Microphone)

# Declare fixed audio stream limits so ESPHome's audio final validation
# has concrete min/max values (prevents None-related errors on some builds).
_BASE_SCHEMA = microphone.MICROPHONE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(KorvoTDMMicrophone),
}).extend(cv.COMPONENT_SCHEMA)

if _HAS_AUDIO_LIMITS:
    CONFIG_SCHEMA = cv.All(
        _BASE_SCHEMA,
        audio_mod.set_stream_limits(
            min_bits_per_sample=16,
            max_bits_per_sample=16,
            min_channels=1,
            max_channels=1,
            min_sample_rate=16000,
            max_sample_rate=16000,
        ),
    )
else:
    CONFIG_SCHEMA = _BASE_SCHEMA

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # Pins/port remain fixed to Korvo-1 BSP defaults in the C++ implementation.
