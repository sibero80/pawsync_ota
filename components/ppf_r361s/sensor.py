import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_WEIGHT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_PERCENT,
)

from . import CONF_PPF_R361S_ID, PPFR361SComponent

DEPENDENCIES = ["ppf_r361s"]

CONF_BOWL_WEIGHT = "bowl_weight"
CONF_BATTERY_LEVEL = "battery_level"
CONF_LAST_FEED_WEIGHT = "last_feed_weight"
CONF_PORTIONS_TODAY = "portions_today"
CONF_DESICCANT_DAYS = "desiccant_days"
CONF_CLEANING_DAYS = "cleaning_days"
CONF_LAST_MEAL_CONSUMED = "last_meal_consumed"
CONF_LAST_MEAL_DURATION = "last_meal_duration"
CONF_EATING_SPEED = "eating_speed"
CONF_TOTAL_EATEN_TODAY = "total_eaten_today"
CONF_FOOD_IN_TANK = "food_in_tank"
CONF_FOOD_TANK_LEVEL = "food_tank_level"
CONF_DAYS_TO_EMPTY_TANK = "days_to_empty_tank"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PPF_R361S_ID): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_BOWL_WEIGHT): sensor.sensor_schema(
            unit_of_measurement="g",
            icon="mdi:scale",
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_WEIGHT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_LAST_FEED_WEIGHT): sensor.sensor_schema(
            unit_of_measurement="g",
            icon="mdi:food-drumstick",
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_WEIGHT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PORTIONS_TODAY): sensor.sensor_schema(
            unit_of_measurement="portions",
            icon="mdi:counter",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_DESICCANT_DAYS): sensor.sensor_schema(
            unit_of_measurement="d",
            icon="mdi:calendar-clock",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_CLEANING_DAYS): sensor.sensor_schema(
            unit_of_measurement="d",
            icon="mdi:broom",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_LAST_MEAL_CONSUMED): sensor.sensor_schema(
            unit_of_measurement="g",
            icon="mdi:silverware-fork-knife",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_WEIGHT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_LAST_MEAL_DURATION): sensor.sensor_schema(
            unit_of_measurement="s",
            icon="mdi:timer-outline",
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EATING_SPEED): sensor.sensor_schema(
            unit_of_measurement="g/min",
            icon="mdi:speedometer",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TOTAL_EATEN_TODAY): sensor.sensor_schema(
            unit_of_measurement="g",
            icon="mdi:chart-bell-curve-cumulative",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_WEIGHT,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_FOOD_IN_TANK): sensor.sensor_schema(
            unit_of_measurement="g",
            icon="mdi:weight-gram",
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_WEIGHT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FOOD_TANK_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon="mdi:gauge",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DAYS_TO_EMPTY_TANK): sensor.sensor_schema(
            unit_of_measurement="d",
            icon="mdi:calendar-clock",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PPF_R361S_ID])
    if bowl_weight_config := config.get(CONF_BOWL_WEIGHT):
        sens = await sensor.new_sensor(bowl_weight_config)
        cg.add(hub.set_bowl_weight_sensor(sens))
    if battery_level_config := config.get(CONF_BATTERY_LEVEL):
        sens = await sensor.new_sensor(battery_level_config)
        cg.add(hub.set_battery_level_sensor(sens))
    if last_feed_config := config.get(CONF_LAST_FEED_WEIGHT):
        sens = await sensor.new_sensor(last_feed_config)
        cg.add(hub.set_last_feed_weight_sensor(sens))
    if portions_today_config := config.get(CONF_PORTIONS_TODAY):
        sens = await sensor.new_sensor(portions_today_config)
        cg.add(hub.set_portions_today_sensor(sens))
    if desiccant_days_config := config.get(CONF_DESICCANT_DAYS):
        sens = await sensor.new_sensor(desiccant_days_config)
        cg.add(hub.set_desiccant_days_sensor(sens))
    if cleaning_days_config := config.get(CONF_CLEANING_DAYS):
        sens = await sensor.new_sensor(cleaning_days_config)
        cg.add(hub.set_cleaning_days_sensor(sens))
    if last_meal_consumed_config := config.get(CONF_LAST_MEAL_CONSUMED):
        sens = await sensor.new_sensor(last_meal_consumed_config)
        cg.add(hub.set_last_meal_consumed_sensor(sens))
    if last_meal_duration_config := config.get(CONF_LAST_MEAL_DURATION):
        sens = await sensor.new_sensor(last_meal_duration_config)
        cg.add(hub.set_last_meal_duration_sensor(sens))
    if eating_speed_config := config.get(CONF_EATING_SPEED):
        sens = await sensor.new_sensor(eating_speed_config)
        cg.add(hub.set_eating_speed_sensor(sens))
    if total_eaten_today_config := config.get(CONF_TOTAL_EATEN_TODAY):
        sens = await sensor.new_sensor(total_eaten_today_config)
        cg.add(hub.set_total_eaten_today_sensor(sens))
    if food_in_tank_config := config.get(CONF_FOOD_IN_TANK):
        sens = await sensor.new_sensor(food_in_tank_config)
        cg.add(hub.set_food_in_tank_sensor(sens))
    if food_tank_level_config := config.get(CONF_FOOD_TANK_LEVEL):
        sens = await sensor.new_sensor(food_tank_level_config)
        cg.add(hub.set_food_tank_level_sensor(sens))
    if days_to_empty_tank_config := config.get(CONF_DAYS_TO_EMPTY_TANK):
        sens = await sensor.new_sensor(days_to_empty_tank_config)
        cg.add(hub.set_days_to_empty_tank_sensor(sens))
