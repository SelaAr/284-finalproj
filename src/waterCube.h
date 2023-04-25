#ifndef WATERCUBE_H
#define WATERCUBE_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"

#include "collision/collisionObject.h"
#include "spring.h"

using namespace CGL;
using namespace std;


struct WaterCubeParameters {
  WaterCubeParameters() {}
  WaterCubeParameters(double damping,
                  double density, double ks)
      :damping(damping), density(density), ks(ks) {}
  ~WaterCubeParameters() {}

  // Global simulation parameters

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct WaterCube {
  WaterCube() {}
  WaterCube(double width, double height, int num_particles);
  ~WaterCube();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, WaterCubeParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  void buildWaterCubeMesh();

  void build_spatial_map();
  void self_collide(Particle &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  //kernels
    void isotropicKernel();

    //Incompressibility Functions


    //For density estimation
    double poly6Kernel(Vector3D r, double h);
    //For gradient calculation
    double spikyKernel(Vector3D r, double h);

    double sphDensityEstimatorPoly(Particle pi, double h);

    double sphDensityEstimatorSpiky(Particle pi, double h);

    double gradientOfConstraint(Particle pi, double h, Particle pk, double rho_0);

    double calcuateLambdaI(Particle pi, double e, double rho_0, double h);

    Vector3D positionUpdate(Particle pi, double e, double rho_0, double h);

    //Tensile Instability
    double artificialPressure(Particle pi, Particle pj, double k, int n, Vector3D delta_q, double h);

  // WaterCube properties
  double width;
  double height;
  int num_particles;


  // WaterCube components
  vector<Particle> water_particles;

  // Spatial hashing
  unordered_map<float, vector<Particle *> *> map;
};

#endif /* WATERCUBE_H */
