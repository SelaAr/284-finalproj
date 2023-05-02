//
// Created by Riddhi Bagadiaa on 02/05/23.
//

#ifndef CLOTHSIM_RIDDHI_H
#define CLOTHSIM_RIDDHI_H

#include "CGL/color.h"
#include "CGL/vector3D.h"
#include "../clothMesh.h"

using namespace CGL;
using namespace std;

class Riddhi {
public:
    // Supply the desired number of vertices
    Riddhi(Vector3D vertex1, Vector3D vertex2, Vector3D vertex3)
    : vertex1(vertex1), vertex2(vertex2), vertex3(vertex3) {
//      draw_normal1();
    }

    Vector3D vertex1;
    Vector3D vertex2;
    Vector3D vertex3;

    Vector3D normal;
    double friction = 0.3;

    void collide1(PointMass &pm);
    void draw_normal1();

private:
};


#endif //CLOTHSIM_RIDDHI_H
