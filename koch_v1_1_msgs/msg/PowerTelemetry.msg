# PowerTelemetry.msg - Dual rail power monitoring
std_msgs/Header header

# 12V Rail INA219
float32 bus_12v_voltage        # V
float32 shunt_12v_voltage      # mV
float32 current_12v            # A
float32 power_12v              # W

# 5V Rail INA219
float32 bus_5v_voltage         # V
float32 shunt_5v_voltage       # mV
float32 current_5v             # A
float32 power_5v               # W

# 5V Rail INA226
float32 bus_5v_voltage_226     # V
float32 shunt_5v_voltage_226   # mV
float32 current_5v_226         # A
float32 power_5v_226           # W


# Total system power
float32 total_power            # W

