# PowerTelemetry.msg - Dual rail power monitoring
# Hardware layout:
#   I2C Bus 0 (GPIO1/2):   INA219 12V @ 0x40, INA219 5V @ 0x41  -- legacy, to be removed
#   I2C Bus 1 (GPIO21/47): INA226 5V  @ 0x44, INA226 12V @ 0x45  -- primary
#                          NOTE: 0x45 is a placeholder - scan actual module before flashing
std_msgs/Header header

# --- 12V Rail INA219 (Bus 0, legacy) ---
float32 bus_12v_voltage        # V
float32 shunt_12v_voltage      # mV
float32 current_12v            # A
float32 power_12v              # W

# --- 5V Rail INA219 (Bus 0, legacy) ---
float32 bus_5v_voltage         # V
float32 shunt_5v_voltage       # mV
float32 current_5v             # A
float32 power_5v               # W

# --- 12V Rail INA226 (Bus 1, primary) ---
float32 bus_12v_voltage_226    # V
float32 shunt_12v_voltage_226  # mV
float32 current_12v_226        # A
float32 power_12v_226          # W

# --- 5V Rail INA226 (Bus 1, primary) ---
float32 bus_5v_voltage_226     # V
float32 shunt_5v_voltage_226   # mV
float32 current_5v_226         # A
float32 power_5v_226           # W

# --- 5V sensor cross-check (keep until INA219 5V retired) ---
float32 current_5v_diff_ma     # mA  INA219_5V - INA226_5V
float32 current_5v_avg_ma      # mA  (INA219_5V + INA226_5V) / 2

# --- Totals ---
float32 total_power            # W   INA226 12V + INA226 5V  (primary)
float32 total_power_219        # W   INA219 12V + INA219 5V  (legacy cross-check)
float32 total_servo_current_5v # mA  sum of all XL330 servo currents (IDs 3-6), filled by power_logger
