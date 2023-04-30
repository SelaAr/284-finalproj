//
// Created by Riddhi Bagadiaa on 26/04/23.
//

#include "rubber_duck.h"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/duck_drawing.h"

using namespace nanogui;
using namespace CGL;

// Not using it right now
// TODO: modify it for duck
void Duck::collide(PointMass &pm) {
//  if ((pm.position - origin).norm() <= radius) {
//    // If it is inside the sphere
//    Vector3D direction = (pm.position - origin).unit();
//    Vector3D tangent = origin + direction * radius;
//    Vector3D correction = tangent - pm.last_position;
//    pm.position = pm.last_position + (correction * (1 - friction));
//  }
}

void Duck::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_duck_mesh->draw_duck(shader, origin);
}
