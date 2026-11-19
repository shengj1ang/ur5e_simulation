Header header
string[] joint_names

# A usable acceleration and unconstrained IK produced a model torque.
bool active
float64[] inertial_offset    # position offset added to the command, rad
float64[] inertial_torque    # model torque before gating per joint or clipping, N*m
float64[] commanded_torque  # bounded input to the position mapping, N*m
uint32[] torque_clip_count  # per-joint clipping events since the controller started
bool static_mapping        # experimental torque / P conversion is selected
