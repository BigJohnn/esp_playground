import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import microphone as mic_mod

AUTO_LOAD = []
DEPENDENCIES = ["microphone", "sd_card"]

ns = cg.esphome_ns.namespace("wav_recorder_cc")
WavRecorder = ns.class_("WavRecorder", cg.Component)

CONF_MIC_ID = "microphone_id"
CONF_SD_ID = "sdcard_id"
CONF_BASE_PATH = "base_path"
CONF_MAX_SECS = "max_seconds"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(WavRecorder),
    cv.Required(CONF_MIC_ID): cv.use_id(mic_mod.Microphone),
    cv.Required(CONF_SD_ID): cv.use_id(cg.Component),
    cv.Optional(CONF_BASE_PATH, default="/sdcard/rec"): cv.string,
    cv.Optional(CONF_MAX_SECS, default=60): cv.int_range(min=1, max=600),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    mic = await cg.get_variable(config[CONF_MIC_ID])
    sd = await cg.get_variable(config[CONF_SD_ID])
    if mic is None:
        raise cv.Invalid("wav_recorder: microphone_id not resolved")
    if sd is None:
        raise cv.Invalid("wav_recorder: sdcard_id not resolved")
    cg.add(var.set_mic(mic))
    cg.add(var.set_sd(sd))
    cg.add(var.set_base_path(config[CONF_BASE_PATH]))
    cg.add(var.set_max_seconds(config[CONF_MAX_SECS]))
