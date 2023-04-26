#ifndef PARTICLE_H
#define PARTICLE_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"
#include "misc/sphere_drawing.h"

using namespace CGL;

// Forward declarations
class Halfedge;

struct Particle {
  Particle(Vector3D position)
      : start_position(position), position(position),
        last_position(position){}

  Vector3D normal();
  Vector3D velocity(double delta_t) {
    return (position - last_position) / delta_t;
  }

  //TODO: change doubles to float?

  // static values
  Vector3D start_position;
  double mass;


  // dynamic values
  Vector3D position;
  Vector3D last_position;
  Vector3D forces;

  double density;
  double pressure;
  double viscosity;
  double vorticity;

    //since it is a sphere
    double radius;
    int sphere_num_lat;
    int sphere_num_lon;

    // mesh reference
    Misc::SphereMesh mesh;

    // collisions with other water particles
    // vector<WaterParticle *> objects;
    std::vector<Particle *> neighbors;
};

#endif /* PARTICLE_H */
