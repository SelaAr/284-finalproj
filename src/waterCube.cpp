#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "waterCube.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "CubeMesh.h"
#include "misc/sphere_drawing.h"

#include <chrono>
#include <ctime>

using namespace std;

WaterCube::WaterCube(Vector3D cube_origin, double cube_width, double cube_height, int num_particles, Cube wCube, std::vector<Plane *> * borders) {
    //Boundaries
  this->cube_origin = cube_origin;
  this->cube_width = cube_width;
  this->cube_height = cube_height;
  this->num_particles = num_particles;
  this->wCube = wCube;
  this->borders = borders;

  generateParticles();
}

WaterCube::~WaterCube() {
    water_particles.clear();
}

double generateRanBetween(double min, double max){
    //Generate a small random offset between min and max
    double ran = (double) std::rand();
    double offset = min + (ran/RAND_MAX) * (max - min);
    return offset;
}

void WaterCube::generateParticles() {

    for(int i = 0; i < num_particles; i++){
        //x needs to be in the range from origin.x to origin.x + cube_width
        double x = generateRanBetween(cube_origin.x, cube_origin.x + cube_width);
        //y needs to be in the range from origin.y to origin.y + cube_width
        double y = generateRanBetween(cube_origin.y + (cube_height * 4 / 5), cube_origin.y + cube_height);
        //z needs to be in the range from origin.z to origin.z + cube_height
        double z = generateRanBetween(cube_origin.z, cube_origin.z + cube_width);
        Vector3D pos = Vector3D(x, y, z);
        Vector3D vel = Vector3D(0,-0.0001,0);
        Particle particle = Particle(pos, vel);
        water_particles.push_back(particle);
    }
}

void WaterCube::addParticles(int num_new_particles, Vector3D velocity) {

    for(int i = 0; i < num_new_particles; i++){
        //x needs to be in the range from origin.x to origin.x + cube_width
        double x = generateRanBetween(cube_origin.x, cube_origin.x + cube_width);
        //y needs to be in the range from origin.y to origin.y + cube_width
        double y = cube_origin.y + cube_height - 0.0001;
        //z needs to be in the range from origin.z to origin.z + cube_height
        double z = generateRanBetween(cube_origin.z, cube_origin.z + cube_width);;
        Vector3D pos = Vector3D(x, y, z);
        Vector3D vel = velocity;
        Particle particle = Particle(pos, vel);
        water_particles.push_back(particle);
    }
    
    this->num_particles += num_new_particles;
}

Vector3D getNormalOfPlane(Vector3D p1, Vector3D p2, Vector3D p3){
    Vector3D a = p2 - p1;
    Vector3D b = p3 - p1;
    Vector3D normal = cross(a,b);
    return normal;
}

void WaterCube::handleCollision(Particle p){
    Vector3D pos = p.position;
    double elasticity = 0.5;
    if(pos.x < cube_origin.x){
        //Crosses left side of the cube
        p.velocity.x = p.velocity.x * -1 * elasticity;
        p.position.x = p.position.x + (cube_origin.x - p.position.x) + 2 * 0.005;
    }
    if(pos.x > (cube_origin.x + cube_width)){
        //Crosses right side of the cube
        p.velocity.x = p.velocity.x * -1 * elasticity;
        p.position.x = p.position.x - (p.position.x - (cube_origin.x + cube_width)) - 2 * 0.005;
    }
    if(pos.y < cube_origin.y){
        //Crosses front side of the cube
        p.velocity.y = p.velocity.y * -1 * elasticity;
        p.position.y = p.position.y + (cube_origin.y - p.position.y) + 2 * 0.005;
    }
    if(pos.y > (cube_origin.y + cube_height)){
        //Crosses back side of the cube
        p.velocity.y = p.velocity.y * -1 * elasticity;
        p.position.y = p.position.y - (p.position.y - (cube_origin.y + cube_height)) - 2 * 0.005;
    }
    if(pos.z > (cube_origin.z + cube_width)){
        //Crosses top
        p.velocity.z = p.velocity.z * -1 * elasticity;
        p.position.z = p.position.z - (p.position.z - (cube_origin.z + cube_width)) - 2 * 0.005;
    }
    if(pos.z < cube_origin.z){
        //Crosses bottom
        p.velocity.z = p.velocity.z * -1 * elasticity;
        p.position.z = p.position.z + (cube_origin.z - p.position.z) + 2 * 0.005;
    }
}

//h is the smoothing length
//H is the subdivision factor
void WaterCube::getNeighbors(double h, int H){
    //Partition space equally into bins
    //EX. Cube is 0.5 by 0.5

}

void WaterCube::simulate(double frames_per_sec, double simulation_steps, WaterCubeParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    double volume = std::pow(cube_width, 3);
    double mass = (cp->rest_density *  volume) / num_particles;
    double delta_t = 1.0f / frames_per_sec / simulation_steps;

    Vector3D extern_forces = Vector3D(0.0,0.0,0.0);
    for (int i = 0; i < external_accelerations.size(); i++) {
        extern_forces += external_accelerations[i] * mass;
    }

    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        pm->forces = extern_forces;
    }
    
//    double wind_velocity_x = generateRanBetween(-1, 1);
//    double wind_velocity_y = generateRanBetween(-1, 1);
    
//    auto start = std::chrono::system_clock::now();
//    std::time_t t = std::time(0);
//    double wind_velocity_x = sin(t % 100);
//    double wind_velocity_y = 0;
    
    // use sin() function and current time in milliseconds to generate smooth continuous wind gust velocities for rain particles
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();
    double t = (ms * 1.0) / 1000.0;
//    cout << "t: " << t << endl;
    double wind_velocity_x = (sin(t) + 0.3) / 2;
//    cout << wind_velocity_x << endl;
    double wind_velocity_y = 0;
    Vector3D new_velocity = Vector3D(wind_velocity_x, -0.0001, wind_velocity_y);
    
    if (ms % 50 == 0) {
        addParticles(10, new_velocity);
        cout << "there are " << num_particles << " particles" << endl;
    }
    
//    // maintain variables to count stagnant particles in order to generate new ones
//    int num_active_particles = 0;

    for(int i = 0; i < water_particles.size(); i++){
        Particle *pm = &water_particles[i];
        //Update velocity
        Vector3D acc = (pm->forces/mass);
        pm->velocity = pm->velocity + (acc * delta_t);

        // Update position
        Vector3D curr_position = pm->position;
        pm->position = curr_position + (pm->velocity * delta_t);
        pm->last_position = curr_position;
        
//        cout << pm->velocity << endl;
        
        // restart raindrop at top if hit floor
        if (pm->position.y < cube_origin.y) {
            //x needs to be in the range from origin.x to origin.x + cube_width
            double x = generateRanBetween(cube_origin.x, cube_origin.x + cube_width);
            //y needs to be in the range from origin.y to origin.y + cube_width
            double y = cube_origin.y + cube_height - 0.0001;
            //z needs to be in the range from origin.z to origin.z + cube_height
            double z = generateRanBetween(cube_origin.z, cube_origin.z + cube_width);
            pm->position = Vector3D(x, y, z);
            pm->last_position = pm->position;
            pm->velocity = new_velocity;
        }
        
//        // create new particle if current one is stagnant
//        if (pm->velocity.y > -3) {
//            num_active_particles += 1;
//        }
    }
    
//    // generate new particles
//    cout << num_active_particles << endl;
//    int new_to_add = 120 - num_active_particles;
//    addParticles(new_to_add, new_velocity);
//    cout << "added " << new_to_add << " new particles" << endl;
    


//    build_spatial_map();
//
//    for(int i = 0; i < water_particles.size(); i++) {
//        Particle *pm = &water_particles[i];
//        self_collide(*pm, simulation_steps);
//    }


    //Handle collisions with plane
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        for (Plane *p: *borders) {
            p->collide(*pm);
        }
    }

    // Handle collisions with other primitives.
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        for (CollisionObject *co: *collision_objects) {
            co->collide(*pm);
        }
    }

}

void WaterCube::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        float box = hash_position(pm->position);
        if(map[box] != nullptr){
            map[box]->push_back(pm);
        }else{
            //Not in map
            std::vector<Particle *> * vec = new std::vector<Particle*>();
            vec->push_back(pm);
            map[box] = vec;
        }
        std::vector<Particle *> * n = map[box];
        pm->neighbors = *n;
    }

}

void WaterCube::self_collide(Particle &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    //Determine whether the point mass, and candidates (excluding point mass)
    //is within 2 * thickness distance apart
    Vector3D correction = Vector3D(0.0, 0.0, 0.0);
    Vector3D pair_correction;
    float thickness = 0.0095;
    int count = 0;
    bool collision = false;
    std::vector<Particle *> neighbors = pm.neighbors;
    for(int i = 0; i < neighbors.size(); i++){
        Particle *n = neighbors[i];
        double distance = (pm.position - n->position).norm();
        if(pm.position.x == n->position.x && pm.position.y == n->position.y && pm.position.z == n->position.z){
            continue;
        }else if(distance < (2 * thickness)){
            pair_correction = ((2 * thickness) - distance) * (pm.position - n->position).unit();
            correction += pair_correction;
            count += 1;
            collision = true;
        }
    }

    //Apply correction
    if(collision){
        correction = (correction/count)/simulation_steps;
        pm.position = pm.position + correction;
    }


}

float WaterCube::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

    float box_w =  3.0 * width / (40 * 1.0);
    float box_h = 3.0 * height/ (40 * 1.0);
    float box_t = max(box_w,box_h);
    int box_num_x = floor(pos.x / box_w);
    int box_num_y = floor(pos.y / box_h);
    int box_num_t = floor(pos.z / box_t);
    return (box_num_x * (width + 1)) + (box_num_y * (height + 1)) + (box_num_t);
}



///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void WaterCube::reset() {
  Particle *pm = &water_particles[0];
  for (int i = 0; i < water_particles.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm->velocity = 0;
    pm++;
  }
}

void WaterCube::buildWaterCubeMesh(GLShader &shader) {
  //if (water_particles.size() == 0) return;

  //Plane 1:Bottom Side
    //Front Bottom Left
    CubePoint p1 = CubePoint(cube_origin);
    //Front Bottom Right
    CubePoint p2 = CubePoint(Vector3D(cube_origin.x + cube_width, cube_origin.y,cube_origin.z));
    //Back Bottom Left
    CubePoint p3 = CubePoint(Vector3D(cube_origin.x,cube_origin.y + cube_height,cube_origin.z));
    //Back Bottom Right
    CubePoint p4 = CubePoint(Vector3D(cube_origin.x + cube_width,cube_origin.y + cube_height,cube_origin.z));
    Plane * bottom = new Plane(p1.pos, getNormalOfPlane(p2.pos, p3.pos, p4.pos), 0.5);
    borders->push_back(bottom);

    //Plane 2:Top Side
    //Front Top Left
    CubePoint p5 = CubePoint(Vector3D(cube_origin.x,cube_origin.y,cube_origin.z + cube_width));
    //Front Top Right
    CubePoint p6 = CubePoint(Vector3D(cube_origin.x + cube_width,cube_origin.y,cube_origin.z + cube_width));
    //Back Top Left
    CubePoint p7 = CubePoint(Vector3D(cube_origin.x,cube_origin.y + cube_height,cube_origin.z + cube_width));
    //Back Top Right
    CubePoint p8 = CubePoint(Vector3D(cube_origin.x + cube_width,cube_origin.y + cube_height,cube_origin.z + cube_width));
    Plane * top = new Plane(p5.pos, getNormalOfPlane(p6.pos, p7.pos, p8.pos), 0.5);
    borders->push_back(top);

    //Plane 3: Front Side
    Plane * front = new Plane(p1.pos, getNormalOfPlane(p2.pos, p5.pos, p6.pos), 0.5);
    borders->push_back(front);

    //Plane 4: Left Side
    Plane * left = new Plane(p1.pos, getNormalOfPlane(p3.pos, p5.pos, p7.pos), 0.5);
    borders->push_back(left);

    //Plane 5: Back Side
    Plane * back = new Plane(p3.pos, getNormalOfPlane(p4.pos, p7.pos, p8.pos), 0.5);
    borders->push_back(back);

    wCube.p1 = p1;
    wCube.p2 = p2;
    wCube.p3 = p3;
    wCube.p4 = p4;
    wCube.p5 = p5;
    wCube.p6 = p6;
    wCube.p7 = p7;
    wCube.p8 = p8;

    wCube.p1.mesh.draw_sphere(shader, wCube.p1.pos, 0.01);
    wCube.p2.mesh.draw_sphere(shader, wCube.p2.pos, 0.01);
    wCube.p3.mesh.draw_sphere(shader, wCube.p3.pos, 0.01);
    wCube.p4.mesh.draw_sphere(shader, wCube.p4.pos, 0.01);
    wCube.p5.mesh.draw_sphere(shader, wCube.p5.pos, 0.01);
    wCube.p6.mesh.draw_sphere(shader, wCube.p6.pos, 0.01);
    wCube.p7.mesh.draw_sphere(shader, wCube.p7.pos, 0.01);
    wCube.p8.mesh.draw_sphere(shader, wCube.p8.pos, 0.01);

//    std::cout << "BUILT WATER CUBE MESH" << std::endl;


//    CubeMesh *clothMesh = new CubeMesh();
//    vector<Triangle *> triangles;
//
//    // Create vector of triangles
//    for (int y = 0; y < 12; y++) {
//            // Get neighboring point masses:
//            /*                      *
//             * pm_A -------- pm_B   *
//             *             /        *
//             *  |         /   |     *
//             *  |        /    |     *
//             *  |       /     |     *
//             *  |      /      |     *
//             *  |     /       |     *
//             *  |    /        |     *
//             *      /               *
//             * pm_C -------- pm_D   *
//             *                      *
//             */
//
//            float u_min = x;
//            u_min /= num_width_points - 1;
//            float u_max = x + 1;
//            u_max /= num_width_points - 1;
//            float v_min = y;
//            v_min /= num_height_points - 1;
//            float v_max = y + 1;
//            v_max /= num_height_points - 1;
//            Vector3D uv_A = Vector3D(u_min, v_min, 0);
//            Vector3D uv_B = Vector3D(u_max, v_min, 0);
//            Vector3D uv_C = Vector3D(u_min, v_max, 0);
//            Vector3D uv_D = Vector3D(u_max, v_max, 0);
//
//
//            // Both triangles defined by vertices in counter-clockwise orientation
//            triangles.push_back(new Triangle(uv_A, uv_C, uv_B));
//            triangles.push_back(new Triangle(uv_B, uv_C, uv_D));
//        }
//    }
//
//    // For each triangle in row-order, create 3 edges and 3 internal halfedges
//    for (int i = 0; i < triangles.size(); i++) {
//        Triangle *t = triangles[i];
//
//        // Allocate new halfedges on heap
//        Halfedge *h1 = new Halfedge();
//        Halfedge *h2 = new Halfedge();
//        Halfedge *h3 = new Halfedge();
//
//        // Allocate new edges on heap
//        Edge *e1 = new Edge();
//        Edge *e2 = new Edge();
//        Edge *e3 = new Edge();
//
//        // Assign a halfedge pointer to the triangle
//        t->halfedge = h1;
//
//        // Assign halfedge pointers to point masses
//        t->pm1->halfedge = h1;
//        t->pm2->halfedge = h2;
//        t->pm3->halfedge = h3;
//
//        // Update all halfedge pointers
//        h1->edge = e1;
//        h1->next = h2;
//        h1->pm = t->pm1;
//        h1->triangle = t;
//
//        h2->edge = e2;
//        h2->next = h3;
//        h2->pm = t->pm2;
//        h2->triangle = t;
//
//        h3->edge = e3;
//        h3->next = h1;
//        h3->pm = t->pm3;
//        h3->triangle = t;
//    }
//
//    // Go back through the cloth mesh and link triangles together using halfedge
//    // twin pointers
//
//    // Convenient variables for math
//    int num_height_tris = (num_height_points - 1) * 2;
//    int num_width_tris = (num_width_points - 1) * 2;
//
//    bool topLeft = true;
//    for (int i = 0; i < triangles.size(); i++) {
//        Triangle *t = triangles[i];
//
//        if (topLeft) {
//            // Get left triangle, if it exists
//            if (i % num_width_tris != 0) { // Not a left-most triangle
//                Triangle *temp = triangles[i - 1];
//                t->pm1->halfedge->twin = temp->pm3->halfedge;
//            } else {
//                t->pm1->halfedge->twin = nullptr;
//            }
//
//            // Get triangle above, if it exists
//            if (i >= num_width_tris) { // Not a top-most triangle
//                Triangle *temp = triangles[i - num_width_tris + 1];
//                t->pm3->halfedge->twin = temp->pm2->halfedge;
//            } else {
//                t->pm3->halfedge->twin = nullptr;
//            }
//
//            // Get triangle to bottom right; guaranteed to exist
//            Triangle *temp = triangles[i + 1];
//            t->pm2->halfedge->twin = temp->pm1->halfedge;
//        } else {
//            // Get right triangle, if it exists
//            if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
//                Triangle *temp = triangles[i + 1];
//                t->pm3->halfedge->twin = temp->pm1->halfedge;
//            } else {
//                t->pm3->halfedge->twin = nullptr;
//            }
//
//            // Get triangle below, if it exists
//            if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
//                Triangle *temp = triangles[i + num_width_tris - 1];
//                t->pm2->halfedge->twin = temp->pm3->halfedge;
//            } else {
//                t->pm2->halfedge->twin = nullptr;
//            }
//
//            // Get triangle to top left; guaranteed to exist
//            Triangle *temp = triangles[i - 1];
//            t->pm1->halfedge->twin = temp->pm2->halfedge;
//        }
//
//        topLeft = !topLeft;
//    }
//
//    clothMesh->triangles = triangles;
//    this->clothMesh = clothMesh;

}



