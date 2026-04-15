#ifndef HYDRAULIC_SYSTEM_MONITOR_H
#define HYDRAULIC_SYSTEM_MONITOR_H
#include <map>
#include <string>
#include <vector>

class HydraulicSystemMonitor {
public:
  HydraulicSystemMonitor(void);
  std::string hydraulic_oil_temperature;
  std::string hydraulic_oil_tank_fill_level;
  std::string hydraulic_oil_pressure;

  std::vector<std::string> getAll();
  std::string getOne(std::string member);
};

#endif