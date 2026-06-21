#include "AP_SpraySystem.hpp"

uint8_t flow_sensor_data[sizeof(AP_SpraySystem_FlowSensor)];

void AP_SpraySystem::init() {
    flow_sensor = new(flow_sensor_data)AP_SpraySystem_FlowSensor();
}

void AP_SpraySystem::update() {
    flow_sensor->update();
}

uint32_t AP_SpraySystem::get_current_flow_rate_ml_min() {
    return flow_sensor->get_flow_rate_ml();
}