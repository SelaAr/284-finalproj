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

WaterCube::WaterCube(Vector3D cube_origin, double cube_width, double cube_height, int num_particles, Cube wCube, std::vector<Plane *> * borders,  int id_count) {
  //Boundaries
  this->cube_origin = cube_origin;
  this->cube_width = cube_width;
  this->cube_height = cube_height;
  this->num_particles = num_particles;
  this->wCube = wCube;
  this->borders = borders;
  this->id_count = id_count;


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
    double mass = 18.01;
    Particle particle = Particle(pos, vel, id_count, mass);
    id_count += 1;
    std::vector<Particle *> * v = new std::vector<Particle*>();
    particle.neighbors = *v;
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
    double mass = 18.01;
    Particle particle = Particle(pos, vel, id_count, mass);
    id_count += 1;
    std::vector<Particle *> * v = new std::vector<Particle*>();
    particle.neighbors = *v;
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


void WaterCube::build_spatial_map_2(WaterCubeParameters *cp) {
    for (const auto &entry : map) {
        delete(entry.second);
    }
    map.clear();


    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        int cell = hash_positionNoBounds(pm->position, cp->smoothing_length);
        if(map[cell] != nullptr){
            map[cell]->push_back(pm);
        }else{
            std::vector<Particle *> * vec = new std::vector<Particle*>();
            vec->push_back(pm);
            map[cell] = vec;
        }
    }
}

int WaterCube::hash_positionNoBounds(Vector3D pos, double h) {
    int x = std::floor(pos.x/(h * 1.0f));
    int y = std::floor(pos.y/(h * 1.0f));
    int z = std::floor(pos.z/(h * 1.0f));
    return std::abs((x * 92837111) ^ (y * 689287499) ^ (z * 283923481));
}



bool WaterCube::inBounds(Vector3D p, double h){
    if(p.x < cube_origin.x || p.x >= cube_origin.x + h ||
       p.y < cube_origin.y || p.y >= cube_origin.y + h ||
       p.z < cube_origin.z || p.z >= cube_origin.z + h){
        return false;
    }
    return true;
}


void WaterCube::getNeighborsSpatialMap(Particle &pm, WaterCubeParameters *cp){
    build_spatial_map_2(cp);
    double h = cp->smoothing_length;

    int pmCell = hash_positionNoBounds(pm.position, h);
    //Get current cell coordinates
    double numY = std::floor(pm.position.y/h) * h;
    double numX = std::floor(pm.position.x/h) * h;
    double numZ = std::floor(pm.position.z/h) * h;


    //Get Max 27 surrounding cells including current cell
    vector<Vector3D> gridCells;
    for(double x = numX - h; x <= numX + h; x += h){
        for(double y = numY - h; y <= numY + h; y += h){
            for(double z = numZ - h; z <= numZ +h; z += h){
                Vector3D pCell = Vector3D(x, y, z);
                //Check if cell is within bounds
                if(inBounds(pCell, h)){
                    gridCells.push_back(pCell);
                    // std::cout << pCell << std::endl;
                }
            }
        }
    }

    //Go through the gridCells and get the neighbors
    for(int i = 0; i < gridCells.size(); i++){
        Vector3D neighborCell = gridCells[i];
        int nCell = hash_positionNoBounds(pm.position, h);
        std::vector<Particle *> neighbors = *map[nCell];
        for(int j = 0; j < neighbors.size(); j++){
            Particle *n = neighbors.at(j);
            if (pm.id != n->id && (pm.position - n->position).norm() <= h) {
                std::cout << "PUSHED BACK NEIGHBOR" << std::endl;
                pm.neighbors.push_back(n);
            }
        }
    }
}



/*KERNELS */
double WaterCube::poly6Kernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 315.0/(64.0 * M_PI * std::pow(h, 9));
    if(r_norm >= 0 && r_norm <= h){
        return scaling_factor * std::pow(std::pow(h, 2) - std::pow(r_norm, 2), 3.0);
    }else{
        return 0.0f;
    }
}

Vector3D WaterCube::poly6GradKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = (945.0f/(32.0f * M_PI * std::pow(h, 9.0f)));
    if(std::pow(r_norm, 2.0f) > std::pow(h,2.0f)){
        return Vector3D(0.0f, 0.0f, 0.0f);
    }
    return -1.0f * r * scaling_factor * std::pow((std::pow(h, 2.0f) - std::pow(r_norm, 2.0f)), 2.0f);
}

double WaterCube::spikyKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 15.0/(M_PI * std::pow(h, 6));
    if(r_norm >= 0 && r_norm <= h){
        return scaling_factor * std::pow(h - r_norm, 3);
    }else{
        return 0.0f;
    }
}

Vector3D WaterCube::spikyGradKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 45.0f/(M_PI * std::pow(h, 6) * std::max(r_norm, 1e-24));
    if(r_norm >= 0 && r_norm <= h){
        return -r * scaling_factor * std::max(r_norm, 1e-24) * std::pow(h - r_norm, 2);
    }else{
        return Vector3D(0.0, 0.0, 0.0);
    }
}


/*Position Based Fluids Equations */

// Calculates the gradient of the constraint function
Vector3D WaterCube::calculateGradientConstraint(Particle pi, Particle pk, float h, double rho_0){
    if(pi.id == pk.id){
        Vector3D sum = Vector3D(0.0, 0.0, 0.0);
        for(int i = 0; i < pi.neighbors.size(); i++){
            Particle *pj = pi.neighbors.at(i);
            sum += spikyGradKernel(pi.position - pj->position, h);
        }
        return (1.0f/rho_0) * sum;
    }

    return -1.0f * (1.0f/rho_0) * spikyGradKernel(pi.position - pk.position, h);
}

//Calculates lambda_i = -1 * Ci(p1,...,pn) / sumK |gradient pk Ci| ^2
float WaterCube::calcuateLambdaI(Particle pi, double e, double rho_0, float h){

    //Step 1: Calculate Ci(p1,...,pn) = (rho_i/rho_0) - 1
    float rho_i = 0.0f;
    for(int i = 0; i < pi.neighbors.size(); i++){
        Particle *pj = pi.neighbors.at(i);
        rho_i += pj->mass * poly6Kernel(pi.position - pj->position, h);
    }

    float density_constraint = (rho_i/rho_0) - 1.0f;

    //Step 2: sumK |gradient pk Ci| ^2 + relaxation e
    float grad_sum = 0.0;
    for(int k = 0; k < pi.neighbors.size(); k++){
        Particle *pk = pi.neighbors.at(k);
        Vector3D grad = calculateGradientConstraint(pi, *pk, h, rho_0);
        //Tried to use the .norm() here
        grad_sum += std::pow(grad.norm(), 2);
    }
    grad_sum += e;

    //Step 3: Plug in above to lambda i
    float lambdaI = (-1.0f * density_constraint)/grad_sum;
    return lambdaI;
}

//constant = 0.1, n = 4, delta_q = 0.2h
float WaterCube::artificialPressure(Particle pi, Particle pj, float k, int n, float delta_q, float h){
    return -1 * k * std::pow((poly6Kernel(pi.position - pj.position, h)/(poly6Kernel(delta_q * h * Vector3D(1,0,0), h))), n);
}


void WaterCube::simulate(double frames_per_sec, double simulation_steps, WaterCubeParameters *cp,
                         vector<Vector3D> external_accelerations,
                         vector<CollisionObject *> *collision_objects) {
  //double volume = std::pow(cube_width, 3);
  //double mass = (cp->rest_density *  volume) / num_particles;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  Vector3D extern_forces = Vector3D(0.0,0.0,0.0);
  for (int i = 0; i < external_accelerations.size(); i++) {
    extern_forces += external_accelerations[i] * cp->mass;
  }

  for (int i = 0; i < water_particles.size(); i++) {
    Particle *pm = &water_particles[i];
    pm->forces = extern_forces;
  }

  int PARTICLE_LIMIT = 250; // maximum limit to # of particles to render in total in scene
  int NEW_GENERATION_PERIOD = 75; // period of time to generate new particles (in milliseconds)
  int NUM_PARTICLES_TO_GEN = 10; // number of particles to generate each period of time above
  int REBIRTH_PERIOD = 9000; // amount of time before rain particles disappear and regenerate from above (in milliseconds)


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
    //Vector3D acc = (pm->forces/mass);
    pm->velocity = pm->velocity + delta_t * pm->forces;

    // Update position
    //Vector3D curr_position = pm->position;
    pm->position = pm->last_position + delta_t * pm->velocity;
    //pm->last_position = curr_position;

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

   //build_spatial_map();
    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        getNeighborsSpatialMap(*pm, cp);
    }

    int iter = 0;
    while(iter < cp->iters) {
        //Calculate lambda i
        vector<double> lambdas;
        for (int i = 0; i < water_particles.size(); i++) {
            Particle *pi = &water_particles[i];
            double lambda_i = calcuateLambdaI(*pi, cp->relaxation_e, cp->rest_density, cp->smoothing_length);
            lambdas.push_back(lambda_i);
        }
        //Calculate delta_pi
        std::vector<Vector3D> *delta_pis = new std::vector<Vector3D>();
        for (int i = 0; i < water_particles.size(); i++) {
            Particle *pi = &water_particles[i];
            double *l = &lambdas[i];
            double lambdaI = *l;
            Vector3D sum = Vector3D(0.0, 0.0, 0.0);
            for (int j = 0; j < pi->neighbors.size(); j++) {
                Particle *pj = pi->neighbors.at(j);
                double *jl = &lambdas[j];
                double lambdaJ = *jl;
                float sCorr = artificialPressure(*pi, *pj, 0.1, 4, 0.3, cp->smoothing_length);
                sum += (lambdaI + lambdaJ + sCorr) * spikyGradKernel(pi->position - pj->position, cp->smoothing_length);
            }
            sum = sum * (1.0f / cp->rest_density);
            delta_pis->push_back(sum);
            // Collision and Detection
            for (Plane *p: *borders) {
                Vector3D normal = p->normal;
                Vector3D point = p->point;
                Vector3D tangent = pi->position - (dot(pi->position - point, normal) * normal);
                Vector3D direction = pi->last_position - tangent;
                double t = dot((point - pi->position), normal) / (dot(direction, normal));
                if (t >= 0) {
                    //Technique 2 implemented from SPH:Toward Flood Simulations
                    double additive_factor = 0.45 * (cp->smoothing_length / (delta_t * pi->velocity.norm()));
                    pi->velocity = pi->velocity - ((1 + additive_factor) * (dot(pi->velocity, normal) * normal));
                   //std::cout << pi->velocity << std::endl;
                    pi->position = pi->position + delta_t * pi->velocity;

                }
            }

            // Handle collisions with other primitives.
            for (CollisionObject *co: *collision_objects) {
                co->collide(*pi);
            }
        }
        //update particle position
        for (int i = 0; i < water_particles.size(); i++) {
            Particle *pm = &water_particles[i];
            Vector3D delta_pi = delta_pis->at(i);
            pm->position = pm->position + delta_pi;
        }
        iter += 1;
    }

    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        //Update velocity
        pm->velocity = (1.0/delta_t) * (pm->position - pm->last_position);

        //Apply vorticity confinement and XSPH Viscosity
        Vector3D wi = Vector3D(0.0, 0.0, 0.0);
        double sum_visc = 0.0;
        for(int j = 0; j < pm->neighbors.size(); j++) {
            Particle *pj = pm->neighbors.at(j);
            Vector3D vij = pj->velocity - pm->velocity;

            //Step 1: Calculate vorticity at a particle's location
            wi += cross(vij, spikyGradKernel(pm->position - pj->position, cp->smoothing_length));

            //Calc XSPH Viscosity
            sum_visc += dot(vij, poly6Kernel(pm->position - pj->position, cp->smoothing_length));
        }
        //Step 2: Calculate a corrective force
        Vector3D ni = poly6GradKernel(wi, cp->smoothing_length);
        Vector3D N = ni/(ni.norm());
        Vector3D vorticity = cp->relaxation_e * (cross(N, wi));
        pm->forces += vorticity;

        pm->velocity = pm->velocity + (0.01 * sum_visc);
        //Update position
        pm->last_position = pm->position;
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
