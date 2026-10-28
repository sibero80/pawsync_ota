import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_PLUG,
    DEVICE_CLASS_PROBLEM,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import CONF_PPF_R361S_ID, PPFR361SComponent

DEPENDENCIES = ["ppf_r361s"]

CONF_POWER_ADAPTER = "power_adapter"
CONF_FOOD_LOW = "food_low"
CONF_FOOD_EMPTY = "food_empty"
CONF_MOTOR_JAMMED = "motor_jammed"
CONF_BOWL_OVERFLOW = "bowl_overflow"
CONF_SCALE_OVERLOAD = "scale_overload"
CONF_BOWL_CONNECTED = "bowl_connected"
CONF_SCALE_TRAY_CONNECTED = "scale_tray_connected"
CONF_BOWL_PRESENT = "bowl_present"
CONF_BATTERY_INSTALLED = "battery_installed"
CONF_BATTERY_LOW = "battery_low"
CONF_CLEANING_DUE = "cleaning_due"
CONF_PET_EATING = "pet_eating"
CONF_ABNORMAL_EATING = "abnormal_eating"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PPF_R361S_ID): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_POWER_ADAPTER): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PLUG,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:power-plug",
        ),
        cv.Optional(CONF_FOOD_LOW): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:tray-alert",
        ),
        cv.Optional(CONF_FOOD_EMPTY): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:tray-remove",
        ),
        cv.Optional(CONF_MOTOR_JAMMED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:alert-octagon",
        ),
        cv.Optional(CONF_BOWL_OVERFLOW): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:tray-full",
        ),
        cv.Optional(CONF_SCALE_OVERLOAD): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:weight",
        ),
        cv.Optional(CONF_BOWL_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:tray",
        ),
        cv.Optional(CONF_SCALE_TRAY_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:dock-bottom",
        ),
        cv.Optional(CONF_BOWL_PRESENT): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:silverware-fork-knife",
        ),
        cv.Optional(CONF_BATTERY_INSTALLED): binary_sensor.binary_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:battery-alert",
        ),
        cv.Optional(CONF_BATTERY_LOW): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:battery-alert-variant-outline",
        ),
        cv.Optional(CONF_CLEANING_DUE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:broom",
        ),
        cv.Optional(CONF_PET_EATING): binary_sensor.binary_sensor_schema(
            icon="mdi:cat",
        ),
        cv.Optional(CONF_ABNORMAL_EATING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            icon="mdi:alert-circle-outline",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PPF_R361S_ID])
    if power_adapter_config := config.get(CONF_POWER_ADAPTER):
        sens = await binary_sensor.new_binary_sensor(power_adapter_config)
        cg.add(hub.set_power_adapter_sensor(sens))
    if food_low_config := config.get(CONF_FOOD_LOW):
        sens = await binary_sensor.new_binary_sensor(food_low_config)
        cg.add(hub.set_food_low_sensor(sens))
    if food_empty_config := config.get(CONF_FOOD_EMPTY):
        sens = await binary_sensor.new_binary_sensor(food_empty_config)
        cg.add(hub.set_food_empty_sensor(sens))
    if motor_jammed_config := config.get(CONF_MOTOR_JAMMED):
        sens = await binary_sensor.new_binary_sensor(motor_jammed_config)
        cg.add(hub.set_motor_jammed_sensor(sens))
    if bowl_overflow_config := config.get(CONF_BOWL_OVERFLOW):
        sens = await binary_sensor.new_binary_sensor(bowl_overflow_config)
        cg.add(hub.set_bowl_overflow_sensor(sens))
    if scale_overload_config := config.get(CONF_SCALE_OVERLOAD):
        sens = await binary_sensor.new_binary_sensor(scale_overload_config)
        cg.add(hub.set_scale_overload_sensor(sens))
    if bowl_connected_config := config.get(CONF_BOWL_CONNECTED):
        sens = await binary_sensor.new_binary_sensor(bowl_connected_config)
        cg.add(hub.set_bowl_connected_sensor(sens))
    if scale_tray_connected_config := config.get(CONF_SCALE_TRAY_CONNECTED):
        sens = await binary_sensor.new_binary_sensor(scale_tray_connected_config)
        cg.add(hub.set_scale_tray_connected_sensor(sens))
    if bowl_present_config := config.get(CONF_BOWL_PRESENT):
        sens = await binary_sensor.new_binary_sensor(bowl_present_config)
        cg.add(hub.set_bowl_present_sensor(sens))
    if battery_installed_config := config.get(CONF_BATTERY_INSTALLED):
        sens = await binary_sensor.new_binary_sensor(battery_installed_config)
        cg.add(hub.set_battery_installed_sensor(sens))
    if battery_low_config := config.get(CONF_BATTERY_LOW):
        sens = await binary_sensor.new_binary_sensor(battery_low_config)
        cg.add(hub.set_battery_low_sensor(sens))
    if cleaning_due_config := config.get(CONF_CLEANING_DUE):
        sens = await binary_sensor.new_binary_sensor(cleaning_due_config)
        cg.add(hub.set_cleaning_due_sensor(sens))
    if pet_eating_config := config.get(CONF_PET_EATING):
        sens = await binary_sensor.new_binary_sensor(pet_eating_config)
        cg.add(hub.set_pet_eating_sensor(sens))
    if abnormal_eating_config := config.get(CONF_ABNORMAL_EATING):
        sens = await binary_sensor.new_binary_sensor(abnormal_eating_config)
        cg.add(hub.set_abnormal_eating_sensor(sens))
