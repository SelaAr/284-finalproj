//
// Created by Riddhi Bagadiaa on 01/05/23.
//

#ifndef CLOTHSIM_TRIANGLE_FACE_H
#define CLOTHSIM_TRIANGLE_FACE_H

#include "nanogui/nanogui.h"
#include "CGL/color.h"
#include "CGL/vector3D.h"
#include "clothMesh.h"
#include "misc/duck_drawing.h"
using namespace CGL;
using namespace std;

    struct TriangleFace{
    public:
        // Supply the desired number of vertices
        TriangleFace(Vector3D vertex1, Vector3D vertex2, Vector3D vertex3)
        : vertex1(vertex1), vertex2(vertex2), vertex3(vertex3) {
          draw_normal();
        }

        Vector3D vertex1;
        Vector3D vertex2;
        Vector3D vertex3;

        Vector3D normal;
        double friction = 0.3;

        void collide(PointMass &pm);

    private:
        void draw_normal();

    };

#endif //CLOTHSIM_TRIANGLE_FACE_H