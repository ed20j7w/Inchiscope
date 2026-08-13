# Sets a piston's step rate. Also affects the next HOME's blind-retract
# duration, since that's computed from the currently configured speed --
# set speed before homing, not after, to get an accurate homing time.
# Firmware clamps to its own [MIN_PISTON_SPEED_MM_S, MAX_PISTON_SPEED_MM_S]
# (the Actuonix S20-38's full-step load-curve range, ~0.1-120mm/s).
string id                  # p1 | p2 | p3 | d1 | d2 | d3 | ALL
float32 mm_per_s
