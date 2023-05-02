//
// Created by Riddhi Bagadiaa on 01/05/23.
//

#include "nanogui/nanogui.h"
#include "triangle_face.h"
#include "CGL/vector3D.h"
#include "misc/duck_drawing.h"
using namespace CGL;
using namespace std;

#define SURFACE_OFFSET 0.0001

    void TriangleFace::draw_normal() {
      Vector3D normal = cross((vertex2 - vertex1), (vertex3 - vertex1));
      normal.normalize();
    }

    void TriangleFace::collide(PointMass &pm) {
      Vector3D tangent = pm.position - (dot(pm.position - vertex1, normal) * normal);
      Vector3D direction = pm.last_position - tangent;
      double t = dot((vertex1 - pm.position), normal) / (dot(direction, normal));
      if (t >= 0) {
        Vector3D correction = (tangent + (SURFACE_OFFSET * normal)) - pm.last_position;
        pm.position = pm.last_position + (correction * (1 - friction));
      }
    }