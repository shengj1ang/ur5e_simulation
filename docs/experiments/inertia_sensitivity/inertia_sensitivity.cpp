// Static equivalent along-axis offset of the inertial feedforward, and its
// sensitivity to the model's own parameters.
//
// The second argument is printed verbatim at the head of every data row, so the
// caller decides what the sweep is over: run.sh passes a bare inertia scale for
// the published sensitivity.csv and 'family,value,unit' for the parameter sweep.
//
// Reproduces the controller's own path: the same chain, the same tip payload,
// the same damped IK parameters and the same Gazebo joint P gains, so the
// numbers are the ones computeInertialTorque() would produce at zero joint
// velocity for a base acceleration of 1 m/s^2 along one base_link axis.
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chaindynparam.hpp>

#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <sim_cartesian_velocity_control/distal_inertia.h>

using controller_interface::chainWithTipPayload;

// gazebo_joint_pid_gains.yaml, root to tip.
static const double P_GAIN[6] = {3120.0, 2925.0, 940.0, 70.0, 50.0, 19.0};
static const double IK_SIGMA_MIN = 0.05;
static const double IK_LAMBDA = 0.2;

struct Pose { const char* name; double q[6]; };
static const Pose POSES[] = {
    {"init",           { 0.0, -1.57, 1.57,  0.0,   1.57, 1.57}},
    {"1st_reco_start", { 0.0, -1.00, 1.00, -1.57, -1.57, 0.00}},
};

int main(int argc, char** argv)
{
  if (argc < 3) { std::fprintf(stderr, "usage: %s <urdf> <csv-row-prefix>\n", argv[0]); return 2; }
  const std::string path = argv[1];
  const std::string label = argv[2];   // printed at the head of each data row

  urdf::Model model;
  if (!model.initFile(path)) { std::fprintf(stderr, "cannot parse %s\n", path.c_str()); return 1; }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) { std::fprintf(stderr, "no kdl tree\n"); return 1; }
  KDL::Chain chain;
  if (!tree.getChain("base_link", "ee_link", chain)) { std::fprintf(stderr, "no chain\n"); return 1; }

  double payload_mass = 0.0;
  KDL::Vector payload_com;
  const KDL::Chain dyn = chainWithTipPayload(chain, model, "ee_link", &payload_mass, &payload_com);

  const unsigned n = chain.getNrOfJoints();
  KDL::ChainJntToJacSolver jac_solver(chain);
  KDL::ChainIkSolverVel_wdls ik(chain, IK_SIGMA_MIN);
  ik.setLambda(IK_LAMBDA);
  KDL::ChainDynParam weight(dyn, KDL::Vector(0.0, 0.0, -9.81));
  // KDL gravity torques are linear in the gravity vector, so the uniform
  // -a_base inertial load of a translating root is G(q; -e_axis) scaled by the
  // acceleration along that axis. One model per axis, as the controller does.
  std::vector<KDL::ChainDynParam*> unit;
  for (int axis = 0; axis < 3; ++axis)
  {
    KDL::Vector apparent = KDL::Vector::Zero();
    apparent(axis) = -1.0;
    unit.push_back(new KDL::ChainDynParam(dyn, apparent));
  }

  std::printf("# payload_mass_kg %.4f cog %.4f %.4f %.4f\n",
              payload_mass, payload_com.x(), payload_com.y(), payload_com.z());

  for (unsigned p = 0; p < sizeof(POSES)/sizeof(POSES[0]); ++p)
  {
    KDL::JntArray q(n), zero(n);
    for (unsigned i = 0; i < n; ++i) q(i) = POSES[p].q[i];
    KDL::SetToZero(zero);

    KDL::Jacobian J(n);
    jac_solver.JntToJac(q, J);
    KDL::JntSpaceInertiaMatrix M(n);
    weight.JntToMass(q, M);
    KDL::JntArray G(n);
    weight.JntToGravity(q, G);

    // Validation quantities, against the figures published for this model.
    const Eigen::MatrixXd Minv = M.data.inverse();
    const Eigen::MatrixXd Linv = J.data * Minv * J.data.transpose();
    const Eigen::MatrixXd L = Linv.inverse();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J.data);
    const Eigen::VectorXd wrench = L * J.data * Minv * G.data;
    std::printf("# %s gravity_torque_nm %.4f %.4f %.4f %.4f %.4f %.4f\n", POSES[p].name,
                G(0), G(1), G(2), G(3), G(4), G(5));
    std::printf("# %s apparent_mass_kg %.4f %.4f %.4f  gravity_wrench_n %.4f"
                "  sigma_min %.4f\n", POSES[p].name, L(0,0), L(1,1), L(2,2),
                wrench.head(3).norm(), svd.singularValues()(5));

    for (int axis = 0; axis < 3; ++axis)
    {
      KDL::Vector a = KDL::Vector::Zero();
      a(axis) = 1.0;                       // 1 m/s^2 along this base_link axis
      // Hold the tip still in world: J qdd = -a_base at zero joint velocity.
      KDL::JntArray qdd(n);
      ik.CartToJnt(q, KDL::Twist(-a, KDL::Vector::Zero()), qdd);
      KDL::JntArray Gax(n);
      unit[axis]->JntToGravity(q, Gax);

      const Eigen::VectorXd tau_inertia = M.data * qdd.data;
      const Eigen::VectorXd tau_gravity = Gax.data;
      const Eigen::VectorXd tau = tau_inertia + tau_gravity;

      // Equivalent static servo offset, carried to the tip and projected on
      // the disturbance direction. Translational Jacobian rows only.
      const Eigen::Matrix<double, 3, 1> dir(a(0), a(1), a(2));
      auto along = [&](const Eigen::VectorXd& t) {
        Eigen::Vector3d d = Eigen::Vector3d::Zero();
        for (unsigned i = 0; i < n; ++i)
          d += J.data.block<3,1>(0, i) * (t(i) / P_GAIN[i]);
        return dir.dot(d);
      };
      std::printf("%s,%s,%c,%.6f,%.6f,%.6f,%.6f,%.6f\n", label.c_str(), POSES[p].name,
                  "xyz"[axis], 1000.0*along(tau), 1000.0*along(tau_inertia),
                  1000.0*along(tau_gravity), tau_inertia.norm(), tau_gravity.norm());
    }
  }
  for (unsigned i = 0; i < unit.size(); ++i) delete unit[i];
  return 0;
}
