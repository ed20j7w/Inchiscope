# Sets the kPa<->DAC-voltage mapping range for one of the two shared
# analog regulators. Defaults on the firmware side already match the
# regulators' physical limits (0..100 kPa positive, -100..0 kPa negative);
# this is only needed if that hardware is ever swapped for a different range.
string regulator_id        # positive | negative
float32 min_kpa
float32 max_kpa
