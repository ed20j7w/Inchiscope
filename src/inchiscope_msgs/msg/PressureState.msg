string ab_id
float32 pressure_kpa
float32 target_pressure_kpa   # only meaningful when mode == "pid"
bool at_target                # only meaningful when mode == "pid"
string mode                   # "open_loop" | "pid" -- ground truth reported by firmware,
                               # driven by which command (VALVE vs AB_PID) was sent last
bool sensor_connected         # false if this AB's pressure sensor wasn't detected at boot;
                               # AB_PID requests for this ab_id are refused by firmware while false
