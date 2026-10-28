import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv

from . import CONF_PPF_R361S_ID, PPFR361SComponent

DEPENDENCIES = ["ppf_r361s"]

CONF_STATUS = "status"
CONF_NEXT_MEAL = "next_meal"
CONF_SCHEDULE_SUMMARY = "schedule_summary"
CONF_LAST_MEAL_TIME = "last_meal_time"
CONF_LAST_ADJUSTED_MEAL = "last_adjusted_meal"
CONF_LAST_ADJUSTED_MEAL_NOTES = "last_adjusted_meal_notes"
CONF_FOOD_TANK_CATEGORY = "food_tank_category"
CONF_LAST_REFILL_TIME = "last_refill_time"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PPF_R361S_ID): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
            icon="mdi:information-outline",
        ),
        cv.Optional(CONF_NEXT_MEAL): text_sensor.text_sensor_schema(
            icon="mdi:clock-outline",
        ),
        cv.Optional(CONF_SCHEDULE_SUMMARY): text_sensor.text_sensor_schema(
            icon="mdi:calendar-clock",
        ),
        cv.Optional(CONF_LAST_MEAL_TIME): text_sensor.text_sensor_schema(
            icon="mdi:clock-check-outline",
        ),
        cv.Optional(CONF_LAST_ADJUSTED_MEAL): text_sensor.text_sensor_schema(
            icon="mdi:clock-alert-outline",
        ),
        cv.Optional(CONF_LAST_ADJUSTED_MEAL_NOTES): text_sensor.text_sensor_schema(
            icon="mdi:note-text-outline",
        ),
        cv.Optional(CONF_FOOD_TANK_CATEGORY): text_sensor.text_sensor_schema(
            icon="mdi:gauge",
        ),
        cv.Optional(CONF_LAST_REFILL_TIME): text_sensor.text_sensor_schema(
            icon="mdi:calendar-refresh",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PPF_R361S_ID])
    if status_config := config.get(CONF_STATUS):
        sens = await text_sensor.new_text_sensor(status_config)
        cg.add(hub.set_status_text_sensor(sens))
    if next_meal_config := config.get(CONF_NEXT_MEAL):
        sens = await text_sensor.new_text_sensor(next_meal_config)
        cg.add(hub.set_next_meal_sensor(sens))
    if summary_config := config.get(CONF_SCHEDULE_SUMMARY):
        sens = await text_sensor.new_text_sensor(summary_config)
        cg.add(hub.set_schedule_summary_sensor(sens))
    if last_meal_time_config := config.get(CONF_LAST_MEAL_TIME):
        sens = await text_sensor.new_text_sensor(last_meal_time_config)
        cg.add(hub.set_last_meal_time_sensor(sens))
    if last_adj_config := config.get(CONF_LAST_ADJUSTED_MEAL):
        sens = await text_sensor.new_text_sensor(last_adj_config)
        cg.add(hub.set_last_adjusted_meal_sensor(sens))
    if last_adj_notes_config := config.get(CONF_LAST_ADJUSTED_MEAL_NOTES):
        sens = await text_sensor.new_text_sensor(last_adj_notes_config)
        cg.add(hub.set_last_adjusted_meal_notes_sensor(sens))
    if food_tank_cat_config := config.get(CONF_FOOD_TANK_CATEGORY):
        sens = await text_sensor.new_text_sensor(food_tank_cat_config)
        cg.add(hub.set_food_tank_category_sensor(sens))
    if last_refill_config := config.get(CONF_LAST_REFILL_TIME):
        sens = await text_sensor.new_text_sensor(last_refill_config)
        cg.add(hub.set_last_refill_time_sensor(sens))
