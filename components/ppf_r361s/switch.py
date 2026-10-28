import esphome.codegen as cg
from esphome.components import switch as switch_
import esphome.config_validation as cv

from . import CONF_PPF_R361S_ID, PPFR361SComponent, ppf_r361s_ns

DEPENDENCIES = ["ppf_r361s"]

CONF_CHILD_LOCK = "child_lock"
CONF_SLOW_FEED = "slow_feed"
CONF_SLEEP_MODE = "sleep_mode"
CONF_INTELLIGENT_FEEDING = "intelligent_feeding"
CONF_STOCK_CALIBRATION = "stock_calibration"
CONF_TANK_PROJECTION_MODE = "tank_projection_mode"
CONF_MANUAL_FEED_CHIME = "manual_feed_chime"
CONF_SCHEDULED_FEED_CHIME = "scheduled_feed_chime"
CONF_PRECISION_MODE = "precision_mode"

PPFR361SChildLockSwitch = ppf_r361s_ns.class_("PPFR361SChildLockSwitch", switch_.Switch)
PPFR361SSlowFeedSwitch = ppf_r361s_ns.class_("PPFR361SSlowFeedSwitch", switch_.Switch)
PPFR361SSleepModeSwitch = ppf_r361s_ns.class_("PPFR361SSleepModeSwitch", switch_.Switch)
PPFR361SIntelligentFeedingSwitch = ppf_r361s_ns.class_("PPFR361SIntelligentFeedingSwitch", switch_.Switch)
PPFR361SStockCalibrationSwitch = ppf_r361s_ns.class_("PPFR361SStockCalibrationSwitch", switch_.Switch)
PPFR361STankProjectionModeSwitch = ppf_r361s_ns.class_("PPFR361STankProjectionModeSwitch", switch_.Switch)
PPFR361SManualFeedChimeSwitch = ppf_r361s_ns.class_("PPFR361SManualFeedChimeSwitch", switch_.Switch)
PPFR361SScheduledFeedChimeSwitch = ppf_r361s_ns.class_("PPFR361SScheduledFeedChimeSwitch", switch_.Switch)
PPFR361SPrecisionModeSwitch = ppf_r361s_ns.class_("PPFR361SPrecisionModeSwitch", switch_.Switch)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PPF_R361S_ID): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_CHILD_LOCK): switch_.switch_schema(
            PPFR361SChildLockSwitch,
            icon="mdi:lock",
        ),
        cv.Optional(CONF_SLOW_FEED): switch_.switch_schema(
            PPFR361SSlowFeedSwitch,
            icon="mdi:speedometer-slow",
        ),
        cv.Optional(CONF_SLEEP_MODE): switch_.switch_schema(
            PPFR361SSleepModeSwitch,
            icon="mdi:power-sleep",
        ),
        cv.Optional(CONF_INTELLIGENT_FEEDING): switch_.switch_schema(
            PPFR361SIntelligentFeedingSwitch,
            icon="mdi:scale-balance",
        ),
        cv.Optional(CONF_STOCK_CALIBRATION): switch_.switch_schema(
            PPFR361SStockCalibrationSwitch,
            icon="mdi:rotate-right",
        ),
        cv.Optional(CONF_TANK_PROJECTION_MODE): switch_.switch_schema(
            PPFR361STankProjectionModeSwitch,
            icon="mdi:chart-timeline-variant",
        ),
        cv.Optional(CONF_MANUAL_FEED_CHIME): switch_.switch_schema(
            PPFR361SManualFeedChimeSwitch,
            icon="mdi:bell-ring",
        ),
        cv.Optional(CONF_SCHEDULED_FEED_CHIME): switch_.switch_schema(
            PPFR361SScheduledFeedChimeSwitch,
            icon="mdi:bell-badge",
        ),
        cv.Optional(CONF_PRECISION_MODE): switch_.switch_schema(
            PPFR361SPrecisionModeSwitch,
            icon="mdi:target-variant",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PPF_R361S_ID])
    if child_lock_config := config.get(CONF_CHILD_LOCK):
        sw = await switch_.new_switch(child_lock_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_child_lock_switch(sw))
    if slow_feed_config := config.get(CONF_SLOW_FEED):
        sw = await switch_.new_switch(slow_feed_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_slow_feed_switch(sw))
    if sleep_mode_config := config.get(CONF_SLEEP_MODE):
        sw = await switch_.new_switch(sleep_mode_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_sleep_mode_switch(sw))
    if intelligent_feeding_config := config.get(CONF_INTELLIGENT_FEEDING):
        sw = await switch_.new_switch(intelligent_feeding_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_intelligent_feeding_switch(sw))
    if stock_calib_config := config.get(CONF_STOCK_CALIBRATION):
        sw = await switch_.new_switch(stock_calib_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_stock_calibration_switch(sw))
    if projection_mode_config := config.get(CONF_TANK_PROJECTION_MODE):
        sw = await switch_.new_switch(projection_mode_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_tank_projection_mode_switch(sw))
    if manual_chime_config := config.get(CONF_MANUAL_FEED_CHIME):
        sw = await switch_.new_switch(manual_chime_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_manual_feed_chime_switch(sw))
    if sched_chime_config := config.get(CONF_SCHEDULED_FEED_CHIME):
        sw = await switch_.new_switch(sched_chime_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_scheduled_feed_chime_switch(sw))
    if precision_mode_config := config.get(CONF_PRECISION_MODE):
        sw = await switch_.new_switch(precision_mode_config)
        await cg.register_parented(sw, hub)
        cg.add(hub.set_precision_mode_switch(sw))
