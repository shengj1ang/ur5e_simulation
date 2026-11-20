Header header
InertiaExperimentSetting setting  # Active override, acknowledged only after the offset has decayed
bool model_valid
float64 sigma_min
float64 raw_axis_offset          # Linearized equivalent position offset along the acceleration, m
float64 mapped_axis_offset       # The same component after optional projection, m
float64[] raw_torque             # Inverse-dynamics result, N*m
float64[] mapped_torque          # After optional projection, before gain, N*m
float64[] requested_torque       # After gain and command-lead gating, before clipping, N*m
