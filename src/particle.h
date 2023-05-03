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
    Particle(Vector3D position, Vector3D velocity)
            : start_position(position), position(position),
              last_position(position), velocity(velocity){}
    //, mass(mass), size(size)

    Vector3D normal();
//  Vector3D velocity(double delta_t) {
//    return (position - last_position) / delta_t;
//  }


    // static values
    Vector3D start_position;
//  double mass;
//  double size;

    // dynamic values
    Vector3D position;
    Vector3D last_position;
    Vector3D forces;
    Vector3D velocity;

    long time_of_birth = std::chrono::duration_cast<std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();

    double density;
    double pressure;
    double viscosity;
    double vorticity;

    //since it is a sphere

    // mesh reference
    Misc::SphereMesh mesh;

    // collisions with other water particles
    std::vector<Particle *> neighbors;

    // TODO: Delete
    // mesh reference
    Halfedge *halfedge;
};

#endif /* PARTICLE_H */