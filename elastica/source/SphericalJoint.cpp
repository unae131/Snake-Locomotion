#include "SphericalJoint.h"

/*
Spherical joint

        This example demonstrates how we connect two Cosserat rods together
using a spherical joint like so, with a force at the tip

// clang-format off

             * Force
              \
               \
                ============================= (o) =============================
                                Rod1    Spherical joint             Rod 2

         ^ z
         |
         |
         o — — —> y
        /
       /
      x

// clang-format on

        No other forces (like gravity) are considered.

        The spherical joint allows for relative rod rotations about it (across
any plane), but prevents relative displacements. That means Rod2 in this case
can have any orientation wrt Rod1 (due to the influence of the force ) but is
always connected to Rod1. In this case, initially the rods are aligned along the
y-axis, and a force with time-varying direction (but fixed magnitude) acts only
along the x-z direction (after an initial transient). The torque due to this
force then brings Rod2 into the x-z plane passing through the spherical joint
(which only counteracts all forces), at which point no out-of-plane torques
exist. From thereon, Rod2 lies in the x-z plane and rotates about the joint in a
circular fashion. In our python script, we also visualize the time-varying force
that acts on the tip so that the user can intuit the rod-motion.

        Note that the rods and joints are minimally damped in this case and
hence you might observe oscillations in the data—for example, the rod does not
perfectly lie on the x-z plane, but rather oscillates about it.

*/

SphericalJoint::SphericalJoint(const int argc, const char **argv)
    : amp(0.0), w(0.0), v(0.0) {}

// Units in this case are mm/g/s

vector<REAL> SphericalJoint::_sphericaljointRun() {
  vector<Rod *> rodPtrs;

  // Dumping frequencies (number of frames/dumps per unit time)
  const REAL diagPerUnitTime = 120;
  const REAL povrayPerUnitTime = 50;
  const REAL dt = 1e-6;
  const REAL timeSimulation = (10.0);

  //-----------------------------------------------------------------------------------------------------------------
  // Define Links
  // Driving parameters
  // Link One shape
  const int n = 10;
  const REAL density = 1.75e-3;  // 1.75g/cm^3
  const REAL L0 = 200.0;
  const REAL r0 = 7.0;
  const REAL E = 3e7;
  const REAL totalMass = density * M_PI * r0 * r0 * L0;

  const REAL dL0 = L0 / (double)n;  // length of cross-section element
  const REAL poissonRatio = 0.5;    // Incompressible
  const REAL G = E / (poissonRatio + 1.0);

  // Define rod
  const Vector3 Linkdirection = Vector3(0.0, -1.0, 0.0);
  const Vector3 Linknormal = Vector3(0.0, 0.0, 1.0);
  const Vector3 Linkoneorigin = Vector3(0.0, L0, L0);
  const Vector3 Linktwoorigin = Linkoneorigin + L0 * Linkdirection;

  // Second moment of area for disk cross section
  const REAL A0 = M_PI * r0 * r0;
  const REAL I0_1 = A0 * A0 / (4.0 * M_PI);
  const REAL I0_2 = I0_1;
  const REAL I0_3 = 2.0 * I0_1;
  const Matrix3 I0 = Matrix3(I0_1, 0.0, 0.0, 0.0, I0_2, 0.0, 0.0, 0.0, I0_3);
  // Mass inertia matrix for disk cross section
  const Matrix3 J0 = density * dL0 * I0;

  // Bending matrix
  Matrix3 B0 =
      Matrix3(E * I0_1, 0.0, 0.0, 0.0, E * I0_2, 0.0, 0.0, 0.0, G * I0_3);
  // Shear matrix
  Matrix3 S0 = Matrix3((4.0 / 3.0) * G * A0, 0.0, 0.0, 0.0,
                       (4.0 / 3.0) * G * A0, 0.0, 0.0, 0.0, E * A0);

  // Initialize straight rod and pack it into a vector of pointers to rod
  const REAL initialTotalTwist = 0.0;
  const REAL nu = 0.1;
  const REAL relaxationNu = 0.0;
  const bool useSelfContact = false;

  Rod *rod1 = RodInitialConfigurations::straightRod(
      n, totalMass, r0, J0, B0, S0, L0, initialTotalTwist, Linkoneorigin,
      Linkdirection, Linknormal, nu, relaxationNu, useSelfContact);
  rodPtrs.push_back(rod1);
  rod1->update(0.0);
  Rod *rod2 = RodInitialConfigurations::straightRod(
      n, totalMass, r0, J0, B0, S0, L0, initialTotalTwist, Linktwoorigin,
      Linkdirection, Linknormal, nu, relaxationNu, useSelfContact);
  rodPtrs.push_back(rod2);
  rod2->update(0.0);

  //-----------------------------------------------------------------------------------------------------------------
  // Pack boundary conditions
  vector<RodBC *> boundaryConditionsPtrs;
  // Link One
  FixedBC fixed = FixedBC(rodPtrs[0]);
  boundaryConditionsPtrs.push_back(&fixed);
  // Link Two
  FreeBC freeBC = FreeBC();
  boundaryConditionsPtrs.push_back(&freeBC);

  // Pack all forces together (no forces applied)
  vector<ExternalForces *> externalForcesPtrs;

  // Gravity
  MultipleForces multipleForces1;
  GravityForce gravity = GravityForce(Vector3(0.0, 0.0, 0.0));
  multipleForces1.add(&gravity);
  MultipleForces *multipleForcesPtr1 = multipleForces1.get();
  for (unsigned int i = 0; i < 2; i++) {
    externalForcesPtrs.push_back(multipleForcesPtr1);
  }

  vector<Interaction *> substrateInteractionsPtrs;

  // Set up External Contact -- This is for the five cases in the paper, not
  // used in this case
  vector<pair<int, int>> attachpoint;
  vector<ExternalContact *> externalcontactPtrs;
  /* The second and third argument are unimportant, but
         are preserved here for legacy purposes. Hence we simply
         set it to 0.0
  */
  ExternalContact externalcontact =
      ExternalContact(rodPtrs, 0.0, 0.0, attachpoint);
  externalcontactPtrs.push_back(&externalcontact);

  // Set up Simple Connection
  vector<SimpleConnection *> simpleconnectionPtrs;
  SimpleConnection simpleconnection = SimpleConnection(rodPtrs);
  simpleconnectionPtrs.push_back(&simpleconnection);
  //-----------------------------------------------------------------------------------------------------------------
  // Set up integrator (define integration order)
  PolymerIntegrator *integrator = new PositionVerlet2nd(
      rodPtrs, externalForcesPtrs, boundaryConditionsPtrs,
      substrateInteractionsPtrs, externalcontactPtrs, simpleconnectionPtrs);

  // Instantiate simulator
  Polymer poly = Polymer(integrator);

  // I am goint go collect data over this time window
  poly.setWindowStats(1.0, 2.0);

  // Run simulation
  string outfileName = string("prova");
  const bool goodRun = poly.simulate(timeSimulation, dt, diagPerUnitTime,
                                     povrayPerUnitTime, outfileName);

  // Throw exception if something went wrong
  if (!goodRun)
    throw "not good run in localized helical buckling, what is going on?";

  const vector<Vector3> avgVel = poly.getAverageVelocity();

  vector<REAL> fwdAvgVel;
  for (unsigned int i = 0; i < 1; i++) {
    fwdAvgVel.push_back(avgVel[i] % Linkdirection);
  }
  const vector<REAL> fitness = fwdAvgVel;

  return (fitness);
}

void SphericalJoint::run() {
  const vector<REAL> fitness = _sphericaljointRun();
  exit(0);
}
