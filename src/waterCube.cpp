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

void WaterCube::respawn_particle(Particle *pm, Vector3D new_velocity) {
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

  int PARTICLE_LIMIT = 250; // maximum limit to # of particles to render in total in scene
  int NEW_GENERATION_PERIOD = 50; // period of time to generate new particles (in milliseconds)
  int NUM_PARTICLES_TO_GEN = 10; // number of particles to generate each period of time above
  int REBIRTH_PERIOD = 7500; // amount of time before rain particles disappear and regenerate from above (in milliseconds)


  // use sin() function and current time in milliseconds to generate smooth continuous wind gust velocities for rain particles
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();
  double t = (ms * 1.0) / 1000.0;
  double wind_velocity_x = (sin(t) + 0.3) / 2;
  double wind_velocity_y = 0;
  Vector3D new_velocity = Vector3D(wind_velocity_x, -0.0001, wind_velocity_y);

  if (ms % NEW_GENERATION_PERIOD == 0 and this->num_particles < PARTICLE_LIMIT) {
    addParticles(NUM_PARTICLES_TO_GEN, new_velocity);
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

    // restart raindrop at top if time expires
    long curr_time = std::chrono::duration_cast<std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();
    if (curr_time - pm->time_of_birth > REBIRTH_PERIOD) {
      respawn_particle(pm, new_velocity);
      pm->time_of_birth = curr_time;
//            cout << "respawning" << endl;
    }

    // restart raindrop at top if hit floor
    if (pm->position.y < cube_origin.y - 0.1) {
      respawn_particle(pm, new_velocity);
      pm->time_of_birth = curr_time;
      cout << "whoosh" << endl;
    }
  }

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
  CubePoint p2 = CubePoint(Vector3D(cube_origin.x + cube_width, cube_origin.y, cube_origin.z));
  //Back Bottom Left
  CubePoint p3 = CubePoint(Vector3D(cube_origin.x, cube_origin.y + cube_height, cube_origin.z));
  //Back Bottom Right
  CubePoint p4 = CubePoint(Vector3D(cube_origin.x + cube_width, cube_origin.y + cube_height, cube_origin.z));
  Plane *bottom = new Plane(p1.pos, getNormalOfPlane(p2.pos, p3.pos, p4.pos), 0.5);
  borders->push_back(bottom);

  //Plane 2:Top Side
  //Front Top Left
  CubePoint p5 = CubePoint(Vector3D(cube_origin.x, cube_origin.y, cube_origin.z + cube_width));
  //Front Top Right
  CubePoint p6 = CubePoint(Vector3D(cube_origin.x + cube_width, cube_origin.y, cube_origin.z + cube_width));
  //Back Top Left
  CubePoint p7 = CubePoint(Vector3D(cube_origin.x, cube_origin.y + cube_height, cube_origin.z + cube_width));
  //Back Top Right
  CubePoint p8 = CubePoint(
          Vector3D(cube_origin.x + cube_width, cube_origin.y + cube_height, cube_origin.z + cube_width));
  Plane *top = new Plane(p5.pos, getNormalOfPlane(p6.pos, p7.pos, p8.pos), 0.5);
  borders->push_back(top);

  //Plane 3: Front Side
  Plane *front = new Plane(p1.pos, getNormalOfPlane(p2.pos, p5.pos, p6.pos), 0.5);
  borders->push_back(front);

  //Plane 4: Left Side
  Plane *left = new Plane(p1.pos, getNormalOfPlane(p3.pos, p5.pos, p7.pos), 0.5);
  borders->push_back(left);

  //Plane 5: Back Side
  Plane *back = new Plane(p3.pos, getNormalOfPlane(p4.pos, p7.pos, p8.pos), 0.5);
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
}
