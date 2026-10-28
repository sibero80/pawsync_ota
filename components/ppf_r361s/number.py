import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv

from . import CONF_PPF_R361S_ID, PPFR361SComponent, ppf_r361s_ns

DEPENDENCIES = ["ppf_r361s"]

CONF_FEED_PORTIONS = "feed_portions"
CONF_TANK_REFILL_VOLUME = "tank_refill_volume"
CONF_TANK_REFILL_WEIGHT = "tank_refill_weight"

PPFR361SManualPortionsNumber = ppf_r361s_ns.class_("PPFR361SManualPortionsNumber", number.Number)
PPFR361STankRefillVolumeNumber = ppf_r361s_ns.class_("PPFR361STankRefillVolumeNumber", number.Number)
PPFR361STankRefillWeightNumber = ppf_r361s_ns.class_("PPFR361STankRefillWeightNumber", number.Number)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PPF_R361S_ID): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_FEED_PORTIONS): number.number_schema(
            PPFR361SManualPortionsNumber,
            icon="mdi:counter",
        ),
        cv.Optional(CONF_TANK_REFILL_VOLUME): number.number_schema(
            PPFR361STankRefillVolumeNumber,
            icon="mdi:cup-water",
            unit_of_measurement="L",
        ),
        cv.Optional(CONF_TANK_REFILL_WEIGHT): number.number_schema(
            PPFR361STankRefillWeightNumber,
            icon="mdi:scale",
            unit_of_measurement="g",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PPF_R361S_ID])
    if num_config := config.get(CONF_FEED_PORTIONS):
        num = await number.new_number(
            num_config,
            min_value=1.0,
            max_value=20.0,
            step=1.0,
        )
        await cg.register_parented(num, hub)
        cg.add(hub.set_manual_feed_portions_number(num))
    if vol_config := config.get(CONF_TANK_REFILL_VOLUME):
        num = await number.new_number(
            vol_config,
            min_value=0.5,
            max_value=4.0,
            step=0.1,
        )
        await cg.register_parented(num, hub)
        cg.add(hub.set_tank_refill_volume_number(num))
    if weight_config := config.get(CONF_TANK_REFILL_WEIGHT):
        num = await number.new_number(
            weight_config,
            min_value=0.0,
            max_value=3000.0,
            step=10.0,
        )
        await cg.register_parented(num, hub)
        cg.add(hub.set_tank_refill_weight_number(num))
