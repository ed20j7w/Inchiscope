string ab_id               # proximal | central | distal
# Open-loop 3-way valve position, signed. +100 = fully toward the positive
# (inflate) regulator line, -100 = fully toward the negative (deflate/vacuum)
# line, 0 = balanced/switching. Sending this command always switches this AB
# to open-loop mode on the firmware, overriding any active AB_PID hold.
float32 duty_pct           # -100..100
