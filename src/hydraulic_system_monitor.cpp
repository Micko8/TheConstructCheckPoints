#include "robot_info/hydraulic_system_monitor.h"

HydraulicSystemMonitor::HydraulicSystemMonitor(void) {
  hydraulic_oil_temperature = "hydraulic_oil_temperature: 45C";
  hydraulic_oil_tank_fill_level = "hydraulic_oil_tank_fill_level: 100%";
  hydraulic_oil_pressure = "hydraulic_oil_pressure: 250 bar";
}

std::string HydraulicSystemMonitor::getOne(std::string member_name) {

  std::map<std::string, std::string> hydro_data = {
      {"hydraulic_oil_temperature", hydraulic_oil_temperature},
      {"hydraulic_oil_tank_fill_level", hydraulic_oil_tank_fill_level},
      {"hydraulic_oil_pressure", hydraulic_oil_pressure}};

  auto it = hydro_data.find(member_name);
  if (it != hydro_data.end())
    return it->second;
  return "ERROR: member not found";
}

std::vector<std::string> HydraulicSystemMonitor::getAll() {
  return {hydraulic_oil_temperature, hydraulic_oil_tank_fill_level,
          hydraulic_oil_pressure};
}