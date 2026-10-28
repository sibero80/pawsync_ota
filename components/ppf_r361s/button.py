import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv

from . import CONF_PPF_R361S_ID, PPFR361SComponent, ppf_r361s_ns

DEPENDENCIES = ["ppf_r361s"]

CONF_TARE = "tare"
CONF_FEED = "feed"
CONF_CALIBRATE_PORTION = "calibrate_portion"
CONF_RESET_DESICCANT = "reset_desiccant"
CONF_RESET_CLEANING = "reset_cleaning"
CONF_RESET_TANK_FOOD = "reset_tank_food"
CONF_PORTIONS = "portions"
CONF_SOUND = "sound"

PPFR361STareButton = ppf_r361s_ns.class_("PPFR361STareButton", button.Button)
PPFR361SFeedButton = ppf_r361s_ns.class_("PPFR361SFeedButton", button.Button)
PPFR361SCalibrateButton = ppf_r361s_ns.class_("PPFR361SCalibrateButton", button.Button)
PPFR361SResetDesiccantButton = ppf_r361s_ns.class_("PPFR361SResetDesiccantButton", button.Button)
PPFR361SResetCleaningButton = ppf_r361s_ns.class_("PPFR361SResetCleaningButton", button.Button)
PPFR361SResetTankFoodButton = ppf_r361s_ns.class_("PPFR361SResetTankFoodButton", button.Button)

FEED_SCHEMA = button.button_schema(
    PPFR361SFeedButton,
    icon="mdi:food-drumstick",
).extend(
    {
        cv.Optional(CONF_PORTIONS, default=1): cv.int_range(min=1, max=20),
        cv.Optional(CONF_SOUND, default=True): cv.boolean,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PPF_R361S_ID): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_TARE): button.button_schema(
            PPFR361STareButton,
            icon="mdi:scale-balance",
        ),
        cv.Optional(CONF_FEED): FEED_SCHEMA,
        cv.Optional(CONF_CALIBRATE_PORTION): button.button_schema(
            PPFR361SCalibrateButton,
            icon="mdi:scale-unbalanced",
        ),
        cv.Optional(CONF_RESET_DESICCANT): button.button_schema(
            PPFR361SResetDesiccantButton,
            icon="mdi:refresh",
        ),
        cv.Optional(CONF_RESET_CLEANING): button.button_schema(
            PPFR361SResetCleaningButton,
            icon="mdi:broom",
        ),
        cv.Optional(CONF_RESET_TANK_FOOD): button.button_schema(
            PPFR361SResetTankFoodButton,
            icon="mdi:refresh-auto",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PPF_R361S_ID])
    if tare_config := config.get(CONF_TARE):
        btn = await button.new_button(tare_config)
        await cg.register_parented(btn, hub)
    if feed_config := config.get(CONF_FEED):
        btn = await button.new_button(feed_config)
        await cg.register_parented(btn, hub)
        cg.add(btn.set_portions(feed_config[CONF_PORTIONS]))
        cg.add(btn.set_sound(feed_config[CONF_SOUND]))
    if calib_config := config.get(CONF_CALIBRATE_PORTION):
        btn = await button.new_button(calib_config)
        await cg.register_parented(btn, hub)
    if desiccant_config := config.get(CONF_RESET_DESICCANT):
        btn = await button.new_button(desiccant_config)
        await cg.register_parented(btn, hub)
    if cleaning_config := config.get(CONF_RESET_CLEANING):
        btn = await button.new_button(cleaning_config)
        await cg.register_parented(btn, hub)
    if tank_refill_config := config.get(CONF_RESET_TANK_FOOD):
        btn = await button.new_button(tank_refill_config)
        await cg.register_parented(btn, hub)
