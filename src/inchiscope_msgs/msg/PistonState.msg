string id
float32 length_mm
float32 target_length_mm
bool at_target
bool homed                 # false until a HOME command has completed for this piston;
                            # PISTON targets are rejected by firmware while false
