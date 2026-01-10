# PowerTelemetry.msg - Dual rail power monitoring
std_msgs/Header header

# 12V Rail
float32 bus_12v_voltage        # V
float32 shunt_12v_voltage      # mV
float32 current_12v            # A
float32 power_12v              # W

# 5V Rail
float32 bus_5v_voltage         # V
float32 shunt_5v_voltage       # mV
float32 current_5v             # A
float32 power_5v               # W

# Total system power
float32 total_power            # W

