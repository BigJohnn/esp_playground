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

_BASE_SCHEMA = microphone.MICROPHONE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(KorvoTDMMicrophone),
    }
).extend(cv.COMPONENT_SCHEMA)


def _set_stream_limits(config):
    # NOTE: audio.set_stream_limits() returns a validator that mutates the config
    # in place and returns None. Feeding it straight into cv.All() makes the whole
    # validation chain evaluate to None -> "TypeError: 'NoneType' object is not
    # iterable" during `esphome config`. Upstream components wrap it exactly like
    # this (see components/i2s_audio/microphone/__init__.py).
    audio_mod.set_stream_limits(
        min_bits_per_sample=16,
        max_bits_per_sample=16,
        min_channels=1,
        max_channels=1,
        min_sample_rate=16000,
        max_sample_rate=16000,
    )(config)
    return config


if _HAS_AUDIO_LIMITS:
    CONFIG_SCHEMA = cv.All(_BASE_SCHEMA, _set_stream_limits)
else:
    CONFIG_SCHEMA = _BASE_SCHEMA


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # Pins/port remain fixed to Korvo-1 BSP defaults in the C++ implementation.
