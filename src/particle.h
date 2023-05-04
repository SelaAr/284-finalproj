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
    Particle(Vector3D position, Vector3D velocity, int id, float mass, float density, double radius)
            : start_position(position), position(position),
              last_position(position), velocity(velocity),
              id(id), mass(mass), density(density), radius(radius){}

    Vector3D start_position;
    Vector3D position;
    Vector3D last_position;
    Vector3D forces;
    Vector3D velocity_minus; //Velocity of t minus 1/2 timestep
    Vector3D velocity_plus;  //Velocity of t plus 1/2 timestep
    Vector3D velocity; //(velocity_minus + velocity_plus)/2

    Vector3D internal_forces;
    Vector3D external_forces;


    int id;
    float mass;
    float density;
    float pressure;


    // mesh reference
    Misc::SphereMesh mesh;
    double radius;

    // collisions with other water particles
    std::vector<Particle *> neighbors;

    // For rain simulation
    //long time_of_birth = std::chrono::duration_cast<std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();
};

#endif /* PARTICLE_H */