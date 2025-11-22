import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = []
AUTO_LOAD = []

korvo_ns = cg.esphome_ns.namespace("korvo_audio")
KorvoAudio = korvo_ns.class_("KorvoAudio", cg.Component)

CONF_SDA = "sda"
CONF_SCL = "scl"
CONF_I2C_CLOCK = "i2c_frequency"
CONF_SPEAKER_RATE = "speaker_sample_rate"
CONF_MIC_RATE = "mic_sample_rate"
CONF_MIC_MCLK_RATIO = "mic_mclk_ratio"
CONF_SPEAKER_VOL = "speaker_volume"
CONF_MIC_VOL = "mic_volume_db"
CONF_MIC_GAIN = "mic_gain"
CONF_MIC_BIAS = "mic_bias"
CONF_MIC_TDM = "mic_tdm"
CONF_ES8311_ADDR = "es8311_address"
CONF_ES7210_ADDR = "es7210_address"
CONF_PA_PIN = "pa_pin"
CONF_PULLUPS = "enable_pullups"

MIC_GAIN_MAP = {
    "0DB": 0,
    "3DB": 1,
    "6DB": 2,
    "9DB": 3,
    "12DB": 4,
    "15DB": 5,
    "18DB": 6,
    "21DB": 7,
    "24DB": 8,
    "27DB": 9,
    "30DB": 10,
    "33DB": 11,
    "34_5DB": 12,
    "36DB": 13,
    "37_5DB": 14,
}

MIC_BIAS_MAP = {
    "2V18": 0x00,
    "2V26": 0x10,
    "2V36": 0x20,
    "2V45": 0x30,
    "2V55": 0x40,
    "2V66": 0x50,
    "2V78": 0x60,
    "2V87": 0x70,
}

def _enum_map(mapping):
    return cv.enum(mapping, upper=True)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(KorvoAudio),
        cv.Optional(CONF_SDA, default=1): cv.int_range(min=0, max=48),
        cv.Optional(CONF_SCL, default=2): cv.int_range(min=0, max=48),
        cv.Optional(CONF_I2C_CLOCK, default=400000): cv.int_range(min=10000, max=1000000),
        cv.Optional(CONF_SPEAKER_RATE, default=22050): cv.int_range(min=8000, max=96000),
        cv.Optional(CONF_MIC_RATE, default=16000): cv.int_range(min=8000, max=96000),
        cv.Optional(CONF_MIC_MCLK_RATIO, default=256): cv.int_range(min=64, max=512),
        cv.Optional(CONF_SPEAKER_VOL, default=85): cv.int_range(min=0, max=100),
        cv.Optional(CONF_MIC_VOL, default=0): cv.int_range(min=-95, max=32),
        cv.Optional(CONF_MIC_GAIN, default="30DB"): _enum_map(MIC_GAIN_MAP),
        cv.Optional(CONF_MIC_BIAS, default="2V87"): _enum_map(MIC_BIAS_MAP),
        cv.Optional(CONF_MIC_TDM, default=True): cv.boolean,
        cv.Optional(CONF_ES8311_ADDR, default=0x18): cv.int_range(min=0x10, max=0x7F),
        cv.Optional(CONF_ES7210_ADDR, default=0x40): cv.int_range(min=0x40, max=0x43),
        cv.Optional(CONF_PA_PIN, default=38): cv.int_range(min=-1, max=48),
        cv.Optional(CONF_PULLUPS, default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sda_pin(config[CONF_SDA]))
    cg.add(var.set_scl_pin(config[CONF_SCL]))
    cg.add(var.set_i2c_clock(config[CONF_I2C_CLOCK]))
    cg.add(var.set_speaker_sample_rate(config[CONF_SPEAKER_RATE]))
    cg.add(var.set_mic_sample_rate(config[CONF_MIC_RATE]))
    cg.add(var.set_mic_mclk_ratio(config[CONF_MIC_MCLK_RATIO]))
    cg.add(var.set_speaker_volume(config[CONF_SPEAKER_VOL]))
    cg.add(var.set_mic_volume_db(config[CONF_MIC_VOL]))
    cg.add(var.set_mic_gain_reg(config[CONF_MIC_GAIN]))
    cg.add(var.set_mic_bias_reg(config[CONF_MIC_BIAS]))
    cg.add(var.set_mic_tdm(config[CONF_MIC_TDM]))
    cg.add(var.set_es8311_address(config[CONF_ES8311_ADDR]))
    cg.add(var.set_es7210_address(config[CONF_ES7210_ADDR]))
    cg.add(var.set_pa_pin(config[CONF_PA_PIN]))
    cg.add(var.set_pullups(config[CONF_PULLUPS]))
