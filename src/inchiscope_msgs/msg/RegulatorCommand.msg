# Sets one of the two shared analog pressure regulators that feed all three
# ABs. Open-loop: the Mega just outputs a PWM/DAC signal that IS the
# setpoint -- the regulator hardware itself closes the loop, so there is no
# feedback sensor on these lines to report back.
string regulator_id        # positive | negative
float32 target_kpa         # positive line: 0..REG_POS_MAX_KPA (firmware-clamped)
                            # negative line: REG_NEG_MIN_KPA..0 (firmware-clamped, vacuum is negative)
