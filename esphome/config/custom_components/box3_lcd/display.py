import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import display, sensor, binary_sensor, text_sensor, time as time_comp
from esphome.const import CONF_ID

DEPENDENCIES = []

box3_lcd_ns = cg.esphome_ns.namespace("box3_lcd")
Box3LCD = box3_lcd_ns.class_("Box3LCD", display.DisplayBuffer)

CONF_TEMPERATURE = "temperature"
CONF_HUMIDITY = "humidity"
CONF_RADAR = "radar"
CONF_STOCK = "stock"
CONF_CALENDAR = "calendar"
CONF_NEWS = "news"
CONF_TIME = "time_id"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_PODCAST = "podcast"

CONFIG_SCHEMA = display.BASIC_DISPLAY_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(Box3LCD),
        cv.Optional(CONF_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_HUMIDITY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_RADAR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_STOCK): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUTDOOR_TEMPERATURE): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_CALENDAR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_NEWS): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_PODCAST): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_TIME): cv.use_id(time_comp.RealTimeClock),
    }
).extend(cv.polling_component_schema("1s"))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield display.register_display(var, config)

    if CONF_TEMPERATURE in config:
        sens = yield cg.get_variable(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_HUMIDITY in config:
        sens = yield cg.get_variable(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(sens))

    if CONF_RADAR in config:
        bs = yield cg.get_variable(config[CONF_RADAR])
        cg.add(var.set_radar_sensor(bs))

    if CONF_STOCK in config:
        sens = yield cg.get_variable(config[CONF_STOCK])
        cg.add(var.set_stock_sensor(sens))

    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = yield cg.get_variable(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temperature_sensor(sens))

    if CONF_CALENDAR in config:
        ts = yield cg.get_variable(config[CONF_CALENDAR])
        cg.add(var.set_calendar_sensor(ts))

    if CONF_NEWS in config:
        ts = yield cg.get_variable(config[CONF_NEWS])
        cg.add(var.set_news_sensor(ts))

    if CONF_PODCAST in config:
        ts = yield cg.get_variable(config[CONF_PODCAST])
        cg.add(var.set_podcast_sensor(ts))

    if CONF_TIME in config:
        t = yield cg.get_variable(config[CONF_TIME])
        cg.add(var.set_time(t))
