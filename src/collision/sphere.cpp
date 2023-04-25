#include <nanogui/nanogui.h>

#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(Particle &pm) {
  // TODO (Part 3): Handle collisions with spheres.
    Vector3D tangent = origin + (radius * (pm.position - origin).unit());
    if((origin - pm.position).norm() <= radius){
        Vector3D correction = tangent - pm.last_position;
        pm.position = pm.last_position + (correction * (1 - friction));
    }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
