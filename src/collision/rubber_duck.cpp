//
// Created by Riddhi Bagadiaa on 26/04/23.
//

#include "rubber_duck.h"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/duck_drawing.h"
#include "../misc/TriFace.h"

using namespace nanogui;
using namespace CGL;

void Duck::collide(PointMass &pm) {
  vector<TriFace> faces = m_duck_mesh->faces;
  for (auto &triangle_face: faces) {
    triangle_face.collide(pm);
  }
}

void Duck::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_duck_mesh->draw_duck(shader, origin);
}
