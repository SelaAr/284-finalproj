//
// Created by Riddhi Bagadiaa on 02/05/23.
//

#ifndef CLOTHSIM_WATERCUBE_H
#define CLOTHSIM_WATERCUBE_H
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"

#include "collision/collisionObject.h"
#include "spring.h"
#include "misc/sphere_drawing.h"
#include "collision/plane.h"

using namespace CGL;
using namespace std;


struct CubePoint{
    CubePoint(){}
    CubePoint(Vector3D pos)
            :pos(pos){}
    ~CubePoint(){}

    Vector3D pos;

    double radius;
    int sphere_num_lat;
    int sphere_num_lon;
    Misc::SphereMesh mesh;

};

struct Cube{
    Cube(){}
    Cube(CubePoint p1, CubePoint p2, CubePoint p3, CubePoint p4,
         CubePoint p5, CubePoint p6, CubePoint p7, CubePoint p8)
            :p1(p1), p2(p2), p3(p3), p4(p4), p5(p5), p6(p6), p7(p7), p8(p8){}
    ~Cube(){}

    CubePoint p1;
    CubePoint p2;
    CubePoint p3;
    CubePoint p4;
    CubePoint p5;
    CubePoint p6;
    CubePoint p7;
    CubePoint p8;
};

struct WaterCubeParameters {
    WaterCubeParameters() {}
    WaterCubeParameters(double damping,
                        double density, double ks, double relaxation_e, double rest_density, double smoothing_length, double elasticity, int iters)
            :damping(damping), density(density), ks(ks), relaxation_e(relaxation_e), rest_density(rest_density),
            smoothing_length(smoothing_length), elasticity(elasticity), iters(iters) {}
    ~WaterCubeParameters() {}

    // Global simulation parameters

    double damping;

    // Mass-spring parameters
    double density;
    double ks;
    double mass;

    //Updated parameters
    double relaxation_e; //e
    double rest_density; //rho_0
    double smoothing_length; //h
    double elasticity;
    int iters;

};

struct WaterCube {
    WaterCube() {}
    WaterCube(Vector3D cube_origin, double cube_width, double cube_height, int num_particles, Cube wCube, std::vector<Plane *> * borders, int id_count);
    ~WaterCube();

    void generateParticles();
    void addParticles(int num_new_particles, Vector3D velocity);
    void respawn_particle(Particle *pm, Vector3D new_velocity);

    void simulate(double frames_per_sec, double simulation_steps, WaterCubeParameters *cp,
                  vector<Vector3D> external_accelerations,
                  vector<CollisionObject *> *collision_objects);

    void reset();
    void buildWaterCubeMesh(GLShader &shader);

    void build_spatial_map();
    void self_collide(Particle &pm, double simulation_steps);
    float hash_position(Vector3D pos);
    void handleCollision(Particle p);
    void getNeighborsSpatialMap(Particle &pm, WaterCubeParameters *cp);
    int hash_positionNoBounds(Vector3D pos, double h);
    void build_spatial_map_2(WaterCubeParameters *cp);
    bool inBounds(Vector3D p, double h);

    // Cube
    Cube wCube;

    // WaterCube properties
    Vector3D cube_origin;
    double cube_width;
    double cube_height;
    int num_particles;

    vector<Particle> water_particles;
    std::vector<Plane *> * borders;
    int id_count;
    double width;
    double height;

    // Spatial hashing
    unordered_map<float, vector<Particle *> *> map;

    //Kernels
    double poly6Kernel(Vector3D r, float h);
    double spikyKernel(Vector3D r, float h);
    Vector3D poly6GradKernel(Vector3D r, float h);
    Vector3D spikyGradKernel(Vector3D r, float h);

    //Position Based Fluids Functions
    Vector3D calculateGradientConstraint(Particle pi, Particle pk, float h, double rho_0);
    float calcuateLambdaI(Particle pi, double e, double rho_0, float h);
    float artificialPressure(Particle pi, Particle pj, float k, int n, float delta_q, float h);
};
#endif //CLOTHSIM_WATERCUBE_H
