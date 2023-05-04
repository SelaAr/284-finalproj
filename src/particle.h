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
    Particle(Vector3D position, Vector3D velocity, Vector3D new_position, int id, double mass, double radius)
            : start_position(position), position(position),
              last_position(position), velocity(velocity), id(id),
              mass(mass), radius(radius){}


    // static values
    Vector3D start_position;
    int id;
    double mass;

    // mesh reference
    Misc::SphereMesh mesh;
    double radius;


    // dynamic values
    Vector3D position;
    Vector3D last_position;
    Vector3D forces;
    Vector3D velocity;


    // collisions with other water particles
    std::vector<Particle *> neighbors;
};

#endif /* PARTICLE_H */