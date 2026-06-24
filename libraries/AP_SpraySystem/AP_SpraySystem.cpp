#include "AP_SpraySystem.hpp"

uint8_t flow_sensor_data[sizeof(AP_SpraySystem_FlowSensor)];
uint8_t spray_nozzle_data[sizeof(AP_SpraySystem_Nozzle)];
uint8_t pump_data[sizeof(AP_SpraySystem_Pump)];

AP_SpraySystem *AP_SpraySystem::_singleton = nullptr;

AP_SpraySystem::AP_SpraySystem()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_SpraySystem must be singleton");
    }
    _singleton = this;
}

void AP_SpraySystem::init()
{
    /* Initialise default parameters */
    AP_Param::setup_object_defaults(this, var_info);

    /* Initialise flow sensor */
    flow_sensor = new(flow_sensor_data)AP_SpraySystem_FlowSensor();
    flow_sensor->init(&FLOW_SENSE_ICU_TIMER,FLOW_SENSE_ICU_CHANNEL, _flow_sense_pulse_ul);
    flow_sensor->set_enabled(true);

    /* Initialise spray nozzle */
    spray_nozzle = new(spray_nozzle_data)AP_SpraySystem_Nozzle(6, 50);

    /* Initialise pump */
    pump = new(pump_data)AP_SpraySystem_Pump(&PWMD3, 0);

    /* Initialise pressure sensor */
    pressure_sensor = new(pressure_sensor_data)AP_SpraySystem_PressureSensor(0);
}

void AP_SpraySystem::update()
{
    uint32_t now = AP_HAL::millis();

    if (now - nozzle_last_update_ms >= NOZZLE_UPDATE_PERIOD_MS) {
        spray_nozzle->update();
        nozzle_last_update_ms = now;
    }

    // TODO: Implement PID control step here
}

uint32_t AP_SpraySystem::get_current_flow_rate_ml_min() {
    return flow_sensor->get_flow_rate_ml();
}

uint32_t AP_SpraySystem::get_current_pressure_mbar()
{
    return pressure_sensor->get_pressure_mbar();
}

uint32_t AP_SpraySystem::get_current_temperature_c()
{
    return pressure_sensor->get_temperature_c();
}

/* Parameters */
const AP_Param::GroupInfo AP_SpraySystem::var_info[] = {
        // @Param: FLOW_SENSE_PULSE_UL
        // @DisplayName: Flow Pulse ul
        // @Description: Number of ul per flow sensor pulse
        // @User Standard
        // @RebootRequired: True
        AP_GROUPINFO("PULSE_UL", 1, AP_SpraySystem, _flow_sense_pulse_ul, FLOW_SENSE_UL_PER_PULSE),

        AP_GROUPEND
};