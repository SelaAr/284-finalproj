#ifndef WATERCUBE_H
#define WATERCUBE_H

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
    WaterCubeParameters(float rest_density, float viscosity, float surface_tension, float threshold, float gas_stiffness,
                        float restitution, float smoothing_length)
            :rest_density(rest_density), viscosity(viscosity), surface_tension(surface_tension),
             threshold(threshold),gas_stiffness(gas_stiffness),restitution(restitution), smoothing_length(smoothing_length){}
    ~WaterCubeParameters() {}

    //SPH PARAMETERS
    float rest_density;
    float viscosity;
    float surface_tension;
    float threshold;
    float gas_stiffness;
    float restitution;
    float smoothing_length;
    double damping;
};

struct WaterCube {
    WaterCube() {}
    WaterCube(Vector3D cube_origin, double cube_width, double cube_height, int num_particles, Cube wCube, std::vector<Plane *> * borders, int id_count);
    ~WaterCube();


    void generateParticles();
    void generateFormedParticles();
    void addParticles(int num_new_particles, Vector3D velocity);
    void respawn_particle(Particle *pm, Vector3D new_velocity);

    void simulate(double frames_per_sec, double simulation_steps, WaterCubeParameters *cp,
                  vector<Vector3D> external_accelerations,
                  vector<CollisionObject *> *collision_objects, int currStep);

    void reset();
    void buildWaterCube(GLShader &shader);
    void createCube(Vector3D origin, double width, double height, GLShader &shader);
    void release();

    bool inBounds(Vector3D p, double h);
    int hash_position(Vector3D pos, double h);
    void getNeighbors(Particle &pm, double h);

    void build_spatial_map(WaterCubeParameters *cp);
    unordered_map<int, vector<Particle *> *> map;


    // Spatial hashing
    vector<Vector3D> grid;

    std::vector<int> * cellCounts;
    std::vector<Particle *> * particleMap;

    void buildGrid(double h);
    void buildHashGrid(float h);
    int hash_positionNoBounds(Vector3D pos, double h);

    // For collisions
    void self_collide(Particle &pi, Particle &pj, float h);
    void getNeighborsNaive(WaterCubeParameters *cp, int H);
    void getNeighborsSpatialMap(Particle &pm, WaterCubeParameters *cp);

    // Cube
    Cube wCube;
    Vector3D cube_origin;
    Vector3D larger_cube_origin;
    double cube_width;
    double cube_height;
    double larger_cube_width;
    double larger_cube_height;
    int num_particles;
    int next_prime;

    //radius of particles
    float radius;
    float mass;

    vector<Particle> water_particles;
    std::vector<Plane *> * borders;
    int id_count;
    double width;
    double height;

    //Kernels
    double poly6Kernel(Vector3D r, float h);
    double spikyKernel(Vector3D r, float h);
    Vector3D poly6GradKernel(Vector3D r, float h);
    Vector3D spikyGradKernel(Vector3D r, float h);

    //SPH specific
    double viscWKernel(Vector3D r, float h);
    Vector3D viscGradWKernel(Vector3D r, float h);
    double viscGrad2WKernel(Vector3D r, float h);
    double pressSpikyWKernel(Vector3D r, float h);
    double poly6W2Kernel(Vector3D r, float h);


    //SPH Functions
    float estimate_density(Particle pi, double rho_0, float h);


    //Position Based Fluids Functions
    Vector3D calculateGradientConstraint(Particle pi, Particle pk, float h, double rho_0);
    float calcuateLambdaI(Particle pi, double e, double rho_0, float h);
    float artificialPressure(Particle pi, Particle pj, float k, int n, float delta_q, float h);
};

#endif /* WATERCUBE_H */