import esphome.codegen as cg
from esphome.components import uart, time as time_
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TIME_ID
from esphome import automation
from esphome.automation import maybe_simple_id

CODEOWNERS = ["@pawsync-re"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

ppf_r361s_ns = cg.esphome_ns.namespace("ppf_r361s")
PPFR361SComponent = ppf_r361s_ns.class_("PPFR361SComponent", cg.Component, uart.UARTDevice)

CONF_PPF_R361S_ID = "ppf_r361s_id"
CONF_ON_SCHEDULE_TRIGGER = "on_schedule_trigger"
CONF_ON_MEAL_EATEN = "on_meal_eaten"
CONF_ON_FEED_COMPLETED = "on_feed_completed"
CONF_ON_TANK_LOW = "on_tank_low"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PPFR361SComponent),
            cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
            cv.Optional(CONF_ON_SCHEDULE_TRIGGER): automation.validate_automation({}),
            cv.Optional(CONF_ON_MEAL_EATEN): automation.validate_automation({}),
            cv.Optional(CONF_ON_FEED_COMPLETED): automation.validate_automation({}),
            cv.Optional(CONF_ON_TANK_LOW): automation.validate_automation({}),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ppf_r361s",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)

_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_SCHEDULE_TRIGGER,
        "add_on_schedule_trigger_callback",
    ),
    automation.CallbackAutomation(
        CONF_ON_MEAL_EATEN,
        "add_on_meal_eaten_callback",
        [(cg.float_, "consumed_g"), (cg.uint32, "duration_s"), (cg.float_, "speed_g_per_min")],
    ),
    automation.CallbackAutomation(
        CONF_ON_FEED_COMPLETED,
        "add_on_feed_completed_callback",
        [(cg.uint16, "portions"), (cg.float_, "weight_g")],
    ),
    automation.CallbackAutomation(
        CONF_ON_TANK_LOW,
        "add_on_tank_low_callback",
        [(cg.float_, "remaining_g"), (cg.float_, "days_left")],
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_TIME_ID in config:
        time_comp = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(time_comp))

    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)


# Automation Actions
CONF_PORTIONS = "portions"
CONF_SOUND = "sound"
CONF_EPOCH = "epoch"
CONF_PLAN = "plan"
CONF_MEAL = "meal"
CONF_ENABLED = "enabled"
CONF_HOUR = "hour"
CONF_MINUTE = "minute"
CONF_DAYS = "days"

PPFR361SScheduleTrigger = ppf_r361s_ns.class_("PPFR361SScheduleTrigger", automation.Trigger.template())
PPFR361SFeedAction = ppf_r361s_ns.class_("PPFR361SFeedAction", automation.Action)
PPFR361STareAction = ppf_r361s_ns.class_("PPFR361STareAction", automation.Action)
PPFR361SSyncClockAction = ppf_r361s_ns.class_("PPFR361SSyncClockAction", automation.Action)
PPFR361SSyncSchedulesAction = ppf_r361s_ns.class_("PPFR361SSyncSchedulesAction", automation.Action)
PPFR361SSetScheduleMealAction = ppf_r361s_ns.class_("PPFR361SSetScheduleMealAction", automation.Action)
PPFR361SSetActivePlanAction = ppf_r361s_ns.class_("PPFR361SSetActivePlanAction", automation.Action)

FEED_ACTION_SCHEMA = cv.maybe_simple_value(
    {
        cv.GenerateID(): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_PORTIONS, default=1): cv.templatable(cv.int_range(min=1, max=20)),
        cv.Optional(CONF_SOUND, default=True): cv.templatable(cv.boolean),
    },
    key=CONF_PORTIONS,
)


@automation.register_action("ppf_r361s.feed", PPFR361SFeedAction, FEED_ACTION_SCHEMA, synchronous=True)
async def feed_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, paren)
    template_portions = await cg.templatable(config[CONF_PORTIONS], args, cg.uint16)
    cg.add(var.set_portions(template_portions))
    template_sound = await cg.templatable(config[CONF_SOUND], args, cg.bool_)
    cg.add(var.set_sound(template_sound))
    return var


TARE_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(PPFR361SComponent),
    }
)


@automation.register_action("ppf_r361s.tare", PPFR361STareAction, TARE_ACTION_SCHEMA, synchronous=True)
async def tare_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, paren)
    return var


SYNC_CLOCK_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_EPOCH, default=0): cv.templatable(cv.uint32_t),
    }
)


@automation.register_action("ppf_r361s.sync_clock", PPFR361SSyncClockAction, SYNC_CLOCK_ACTION_SCHEMA, synchronous=True)
async def sync_clock_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, paren)
    template_epoch = await cg.templatable(config[CONF_EPOCH], args, cg.uint32)
    cg.add(var.set_epoch(template_epoch))
    return var


SET_SCHEDULE_MEAL_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(PPFR361SComponent),
        cv.Optional(CONF_PLAN, default=1): cv.templatable(cv.int_range(min=1, max=7)),
        cv.Optional(CONF_MEAL, default=1): cv.templatable(cv.int_range(min=1, max=10)),
        cv.Optional(CONF_ENABLED, default=True): cv.templatable(cv.boolean),
        cv.Required(CONF_HOUR): cv.templatable(cv.int_range(min=0, max=23)),
        cv.Required(CONF_MINUTE): cv.templatable(cv.int_range(min=0, max=59)),
        cv.Optional(CONF_PORTIONS, default=1): cv.templatable(cv.int_range(min=1, max=20)),
        cv.Optional(CONF_DAYS, default=127): cv.templatable(cv.int_range(min=1, max=127)),
        cv.Optional(CONF_SOUND, default=True): cv.templatable(cv.boolean),
    }
)


@automation.register_action("ppf_r361s.set_schedule_meal", PPFR361SSetScheduleMealAction, SET_SCHEDULE_MEAL_SCHEMA, synchronous=True)
async def set_schedule_meal_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, paren)

    tpl_plan = await cg.templatable(config[CONF_PLAN], args, cg.uint8)
    cg.add(var.set_plan(tpl_plan))
    tpl_meal = await cg.templatable(config[CONF_MEAL], args, cg.uint8)
    cg.add(var.set_meal(tpl_meal))
    tpl_en = await cg.templatable(config[CONF_ENABLED], args, cg.bool_)
    cg.add(var.set_enabled(tpl_en))
    tpl_h = await cg.templatable(config[CONF_HOUR], args, cg.uint8)
    cg.add(var.set_hour(tpl_h))
    tpl_m = await cg.templatable(config[CONF_MINUTE], args, cg.uint8)
    cg.add(var.set_minute(tpl_m))
    tpl_p = await cg.templatable(config[CONF_PORTIONS], args, cg.uint8)
    cg.add(var.set_portions(tpl_p))
    tpl_d = await cg.templatable(config[CONF_DAYS], args, cg.uint8)
    cg.add(var.set_days(tpl_d))
    tpl_s = await cg.templatable(config[CONF_SOUND], args, cg.bool_)
    cg.add(var.set_sound(tpl_s))
    return var


SET_ACTIVE_PLAN_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(PPFR361SComponent),
        cv.Required(CONF_PLAN): cv.templatable(cv.int_range(min=1, max=7)),
    }
)


@automation.register_action("ppf_r361s.set_active_plan", PPFR361SSetActivePlanAction, SET_ACTIVE_PLAN_SCHEMA, synchronous=True)
async def set_active_plan_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, paren)
    tpl_plan = await cg.templatable(config[CONF_PLAN], args, cg.uint8)
    cg.add(var.set_plan(tpl_plan))
    return var


SYNC_SCHEDULES_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(PPFR361SComponent),
    }
)


@automation.register_action("ppf_r361s.sync_schedules", PPFR361SSyncSchedulesAction, SYNC_SCHEDULES_ACTION_SCHEMA, synchronous=True)
async def sync_schedules_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, paren)
    return var

