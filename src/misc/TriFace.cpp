//
// Created by Riddhi Bagadiaa on 02/05/23.
//

#include "TriFace.h"
#include "CGL/vector3D.h"
#include "../CubeMesh.h"
#include "../particle.h"
#include <iostream>

using namespace CGL;
using namespace std;

#define SURFACE_OFFSET 0.001

void TriFace::draw_normal() {

  this->normal = cross((vertex2 - vertex1), (vertex3 - vertex1));
  normal.normalize();
    this->normal = this->normal * -1.0;
}

void TriFace::collide(Particle &pm) {
    Vector3D tangent = pm.position - (dot(pm.position - this->vertex1, this->normal) * this->normal);
    Vector3D direction = pm.last_position - tangent;

    //cout << pm.position << endl;
    double t = dot((this->vertex1 - pm.position), this->normal)/(dot(direction, this->normal));
    if(t >= 0){
        Vector3D v1ToPoint = pm.position - this->vertex1;
        Vector3D v2ToPoint = pm.position - this->vertex2;
        Vector3D v3ToPoint = pm.position - this->vertex3;
        
        // Calculate angles between the vectors
        float angle1 = acos(dot(v1ToPoint, v2ToPoint) / (v1ToPoint.norm() * v2ToPoint.norm()));
        float angle2 = acos(dot(v2ToPoint, v3ToPoint) / (v2ToPoint.norm() * v3ToPoint.norm()));
        float angle3 = acos(dot(v3ToPoint, v1ToPoint) / (v3ToPoint.norm() * v1ToPoint.norm()));
        
        // Calculate the sum of the angles
        float angleSum = angle1 + angle2 + angle3;
        
        cout << angleSum << endl;
        // Check if the angles add up to 2 * pi radians (360 degrees)
        if (angleSum >= 6.2) {
            Vector3D correction = (tangent + (SURFACE_OFFSET * normal)) - pm.last_position;
            pm.position = pm.last_position + (correction * (1 - friction));
        }
    }
}
