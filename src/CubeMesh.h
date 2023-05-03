//
// Created by Riddhi Bagadiaa on 02/05/23.
//

#ifndef CLOTHSIM_CUBEMESH_H
#define CLOTHSIM_CUBEMESH_H

#include <vector>

#include "CGL/CGL.h"
#include "particle.h"
#include "CGL/misc.h"

using namespace CGL;
using namespace std;

class Triangle {
public:
    Triangle(Vector3D uv1, Vector3D uv2, Vector3D uv3)
            : uv1(uv1), uv2(uv2), uv3(uv3) {}

    // UV values for each of the points.
    // Uses Vector3D for convenience. This means that the z dimension
    // is not used, and xy corresponds to uv.
    Vector3D uv1;
    Vector3D uv2;
    Vector3D uv3;

    Halfedge *halfedge;
}; // struct Triangle

class Edge {
public:
    Halfedge *halfedge;
}; // struct Edge

class Halfedge {
public:
    Edge *edge;
    Halfedge *next;
    Halfedge *twin;
    Triangle *triangle;
    Particle *pm;
}; // struct Halfedge

class CubeMesh {
public:
    ~CubeMesh() {}

    vector<Triangle *> triangles;
}; // struct ClothMesh

#endif //CLOTHSIM_CUBEMESH_H
