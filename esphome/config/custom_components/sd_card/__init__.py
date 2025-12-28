import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

sd_ns = cg.esphome_ns.namespace("sd_card_cc")
SDCardMount = sd_ns.class_("SDCardMount", cg.Component)

CONF_CLK = "clk_pin"
CONF_CMD = "cmd_pin"
CONF_D0 = "d0_pin"
CONF_WIDTH = "width"
CONF_MOUNT_POINT = "mount_point"
CONF_MAX_FILES = "max_files"
CONF_FORMAT_IF_FAIL = "format_if_mount_failed"
CONF_ALLOC_UNIT = "allocation_unit_kb"
CONF_POWER_PIN = "power_pin"
CONF_POWER_ACTIVE_LOW = "power_active_low"
CONF_POWER_ON_DELAY = "power_on_delay_ms"
CONF_FREQ_KHZ = "frequency_khz"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SDCardMount),
    cv.Required(CONF_CLK): cv.int_range(min=0, max=48),
    cv.Required(CONF_CMD): cv.int_range(min=0, max=48),
    cv.Required(CONF_D0): cv.int_range(min=0, max=48),
    cv.Optional(CONF_WIDTH, default=1): cv.one_of(1, 4, int=True),
    cv.Optional(CONF_MOUNT_POINT, default="/sdcard"): cv.string,
    cv.Optional(CONF_MAX_FILES, default=5): cv.int_range(min=1, max=16),
    cv.Optional(CONF_FORMAT_IF_FAIL, default=False): cv.boolean,
    cv.Optional(CONF_ALLOC_UNIT, default=16): cv.int_range(min=4, max=64),  # KB
    cv.Optional(CONF_POWER_PIN, default=-1): cv.int_range(min=-1, max=48),
    cv.Optional(CONF_POWER_ACTIVE_LOW, default=False): cv.boolean,
    cv.Optional(CONF_POWER_ON_DELAY, default=50): cv.int_range(min=0, max=2000),
    cv.Optional(CONF_FREQ_KHZ, default=26000): cv.int_range(min=400, max=40000),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_pins(config[CONF_CLK], config[CONF_CMD], config[CONF_D0]))
    cg.add(var.set_bus_width(config[CONF_WIDTH]))
    cg.add(var.set_mount_point(config[CONF_MOUNT_POINT]))
    cg.add(var.set_max_files(config[CONF_MAX_FILES]))
    cg.add(var.set_format_if_fail(config[CONF_FORMAT_IF_FAIL]))
    cg.add(var.set_alloc_unit_kb(config[CONF_ALLOC_UNIT]))
    cg.add(var.set_power_pin(config[CONF_POWER_PIN]))
    cg.add(var.set_power_active_low(config[CONF_POWER_ACTIVE_LOW]))
    cg.add(var.set_power_on_delay_ms(config[CONF_POWER_ON_DELAY]))
    cg.add(var.set_frequency_khz(config[CONF_FREQ_KHZ]))
