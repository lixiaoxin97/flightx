# Quadrotor control command

# Control mode enums
# SRT: single-rotor thrust
# CBTR: collective thrust and bodyrates
uint8 SRT=0
uint8 CTBR=1

# Header
std_msgs/Header header

# Flag whether the low level controller is connected
bool connection_state

# Flag whether the control command is active
bool active

# Control mode as defined above
uint8 control_mode

# Time at which this command should be executed
builtin_interfaces/Time expected_execution_time

# SRT: Single rotor thrusts [N]
float64[] rotor_thrusts

# CBTR: Body rates in body frame [rad/s]
geometry_msgs/Vector3 bodyrates

# CBTR: Collective mass normalized thrust [m/s^2]
float64 collective_thrust