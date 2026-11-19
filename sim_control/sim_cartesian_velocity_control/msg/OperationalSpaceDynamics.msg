# Operational-space (task-space) dynamics of the arm at the configuration this
# message was stamped for, expressed in the controller's root frame with the
# end effector at the tip. The intended use is the operational-space form
#
#   F = inertia * xddot_desired + coriolis + gravity
#
# where F is the wrench at the tip that produces that task acceleration.
#
# The quantities come from the URDF this controller was given, which under
# Gazebo is the same description the simulator integrates, so the model matches
# the simulated plant. It does not match a real UR5e: the link inertia tensors
# in that description are cylinder approximations, only the masses are the
# manufacturer's.
Header header

string root_name            # frame everything below is expressed in
string tip_name             # end effector the chain ends at
string[] joint_names        # chain order, root to tip
float64[] position          # q
float64[] velocity          # qdot, differentiated from position

# J M^-1 J^T, 6x6 row-major, linear part first. Always finite, including at a
# singularity, where it simply loses rank.
float64[36] inertia_inv

# Lambda = (J M^-1 J^T)^+, 6x6 row-major. This is the pseudo-inverse, not the
# inverse: in a direction the arm cannot accelerate the tip, Lambda comes back
# as zero rather than as infinity. That keeps the message finite everywhere,
# but it is not the operational-space inertia there - check `valid` first.
float64[36] inertia

# mu = Lambda (J M^-1 C(q,qdot) qdot - Jdot qdot), the centrifugal and Coriolis
# wrench at the tip. Zero when the arm is not moving.
float64[6] coriolis

# p = Lambda J M^-1 G(q), the wrench at the tip that holds the arm against
# gravity.
float64[6] gravity

# Smallest singular value of the Jacobian: how far this configuration is from a
# singularity, in the same terms as the IK solver's ik_sigma_min.
float64 jacobian_sigma_min

# How many singular values of inertia_inv were kept when inverting it. 6 is
# full rank; less means Lambda is a projection onto what the arm can do.
uint8 inertia_rank

# True when jacobian_sigma_min is above the controller's sigma_min and
# inertia_rank is 6, i.e. when inertia is a genuine inverse. Consumers should
# fall back to whatever they did without a model when this is false.
bool valid
