//
// Created by Riddhi Bagadiaa on 26/04/23.
//

#ifndef CLOTHSIM_RUBBER_DUCK_H
#define CLOTHSIM_RUBBER_DUCK_H

#include "../clothMesh.h"
#include "../misc/duck_drawing.h"
#include "collisionObject.h"

using namespace CGL;
using namespace std;

struct Duck : public CollisionObject {
public:
    Duck(const Vector3D &origin, double friction, Misc::DuckMesh* d)
            : origin(origin),
              friction(friction), m_duck_mesh(d) {}

    void render(GLShader &shader);
    void collide(PointMass &pm);

private:
    Vector3D origin;

    double friction;

    Misc::DuckMesh* m_duck_mesh;
};
#endif //CLOTHSIM_RUBBER_DUCK_H
