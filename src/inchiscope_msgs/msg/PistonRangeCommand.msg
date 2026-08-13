# Sets a piston's usable travel range. Affects PISTON target clamping AND
# how long the next HOME blind-retract runs for (computed from the range at
# HOME time, not fixed) -- see inchiscope_serial_bridge/README.md.
string id                  # p1 | p2 | p3 | d1 | d2 | d3 | ALL
float32 min_mm
float32 max_mm
