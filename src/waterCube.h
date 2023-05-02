#ifndef WATERCUBE_H
#define WATERCUBE_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"

#include "collision/collisionObject.h"
#include "spring.h"
#include "misc/sphere_drawing.h"
#include "collision/plane.h"

using namespace CGL;
using namespace std;


struct CubePoint{
    CubePoint(){}
    CubePoint(Vector3D pos)
        :pos(pos){}
        ~CubePoint(){}

        Vector3D pos;

    double radius;
    int sphere_num_lat;
    int sphere_num_lon;
    Misc::SphereMesh mesh;

};

struct Cube{
    Cube(){}
    Cube(CubePoint p1, CubePoint p2, CubePoint p3, CubePoint p4,
         CubePoint p5, CubePoint p6, CubePoint p7, CubePoint p8)
         :p1(p1), p2(p2), p3(p3), p4(p4), p5(p5), p6(p6), p7(p7), p8(p8){}
         ~Cube(){}

    CubePoint p1;
    CubePoint p2;
    CubePoint p3;
    CubePoint p4;
    CubePoint p5;
    CubePoint p6;
    CubePoint p7;
    CubePoint p8;
};

struct WaterCubeParameters {
  WaterCubeParameters() {}
  WaterCubeParameters(double damping,
                  double density, double ks, double relaxation_e, double rest_density, double smoothing_length)
      :damping(damping), density(density), ks(ks), relaxation_e(relaxation_e), rest_density(rest_density), smoothing_length(smoothing_length) {}
  ~WaterCubeParameters() {}

  // Global simulation parameters

  double damping;

  // Mass-spring parameters
  double density;
  double ks;

  //Updated parameters
  double relaxation_e; //e
  double rest_density; //rho_0
  double smoothing_length; //h

};

struct WaterCube {
  WaterCube() {}
  WaterCube(Vector3D cube_origin, double cube_width, double cube_height, int num_particles, Cube wCube, std::vector<Plane *> * borders);
  ~WaterCube();

  void generateParticles();
  void addParticles(int num_new_particles, Vector3D velocity);
  void respawn_particle(Particle *pm, Vector3D new_velocity);

  void simulate(double frames_per_sec, double simulation_steps, WaterCubeParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  void buildWaterCubeMesh(GLShader &shader);

  void build_spatial_map();
  void self_collide(Particle &pm, double simulation_steps);
  float hash_position(Vector3D pos);
  void handleCollision(Particle p);
  void getNeighbors(double h, int H);

  // Cube
  Cube wCube;

  // WaterCube properties
  Vector3D cube_origin;
  double cube_width;
  double cube_height;
  int num_particles;

  vector<Particle> water_particles;
  std::vector<Plane *> * borders;

  double width;
  double height;

  // Spatial hashing
  unordered_map<float, vector<Particle *> *> map;
};

#endif /* WATERCUBE_H */
