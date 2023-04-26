#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "waterCube.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "CubeMesh.h"

using namespace std;

WaterCube::WaterCube(double width, double height, int num_particles) {
  this->width = width;
  this->height = height;
  this->num_particles = num_particles;

  buildGrid();
  buildWaterCubeMesh();
}

WaterCube::~WaterCube() {
    water_particles.clear();
}

void WaterCube::buildGrid() {
    // Generates all particles
//    for(int i = 0; i < num_particles; i++){
//        //Generate a small random number between -1/1000 and 1/1000 for each point mass, and use z coordinate while varying positions over the xy plane
//        double ranX = (double) std::rand();
//        double ranY = (double) std::rand();
//        double ranZ = (double) std::rand();
//        double min = -1.0/1000;
//        double max = 1.0/1000;
//        double x = min + (ranX/RAND_MAX) * (max - min);
//        double y = min + (ranY/RAND_MAX) * (max - min);
//        double z = min + (ranZ/RAND_MAX) * (max - min);
//        Vector3D pos = Vector3D(x, y, z);
//        double density = 1000; //In kg/m^3
//        Particle particle = Particle(pos);
//       water_particles.push_back(particle);
//    }


    for (double y = 0; y < 9; y++) {
        for (double x = 0; x < 9; x++) {
            Vector3D pos;
            //Set the y coordinate for all point masses to 1 while varying positions over the xz plane
            double ranX = (double) std::rand();
            double ranY = (double) std::rand();
            double min = 0.0;
            double maxX = width;
            double maxY = height;

            double x_val = min + (ranX/RAND_MAX) * (maxX - min);
            double y_val = min + (ranY/RAND_MAX) * (maxY - min);

            //Make the y from the range 1.0 to 2.0
            double ranY2 = (double) std::rand();
            double maxY2 = 1.0 + 0.005;
            double minY2 = 1.0 - 0.005;
            double y2 = minY2 + (ranY2/RAND_MAX) * (maxY2 - min);

            pos = Vector3D(x_val,y2, y_val);
            double density = 1000; //In kg/m^3
            Particle particle = Particle(pos);
           water_particles.push_back(particle);
        }
    }

//    for (int i = 0; i < num_particles; i++) {
//
//        Vector3D bottom_right = Vector3D(0, 0, 0);
//        Vector3D top_left = Vector3D(0.005, 0.2, 0.005);
//        Vector3D size = top_left - bottom_right;
//        double x = ((float) rand() / RAND_MAX) * abs(size.x);
//        double y = ((float) rand() / RAND_MAX) * abs(size.y);
//        double z = ((float) rand() / RAND_MAX) * abs(size.z);
//        Vector3D position = Vector3D (bottom_right.x + x, 1.0,bottom_right.z + z);
//        Particle particle = Particle(position);
//        water_particles.push_back(particle);
//    }

}

void WaterCube::simulate(double frames_per_sec, double simulation_steps, WaterCubeParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects, double e, double rho_0, double h) {
  double mass = width * height * cp->density;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

    //For all particles i, calculate lambda i
    std::vector<double> * lambdas = new std::vector<double>();
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        double li = calcuateLambdaI(*pm, e,rho_0, h);
        lambdas->push_back(li);
    }
    std::cout << "LAMBDAS CALCULATED" << std::endl;

    //For all particles i, calculate delta_pi & perform collision detection and response
    std::vector<Vector3D> * new_positions = new std::vector<Vector3D>();
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pi = &water_particles[i];
        double lambdaI = lambdas->at(i);
        double sumJ = 0.0;
        for(int j = 0; j < pi->neighbors.size(); j++) {
            Particle pj = *pi->neighbors[j];
            double lambdaJ =lambdas->at(j);
            double sCorr = artificialPressure(*pi, pj, 0.1, 4, Vector3D(1,1,1), h);
            sumJ = (lambdaI + lambdaJ + sCorr) * spikyKernel(pi->position - pj.position, h);
        }
        Vector3D delta_pi = (1/rho_0) * sumJ;
        new_positions->push_back(delta_pi);
        std::cout << "DELTA PI CALCULATED" << std::endl;


        //perform collision detection and response
        for (CollisionObject *co: *collision_objects) {
            co->collide(*pi);
        }
        std::cout << "COLLISIONS HANDLED" << std::endl;
    }

    //for all particles i, update the position
    for(int i = 0; i < water_particles.size(); i++){
        Particle *pm = &water_particles[i];
        assert(water_particles.size() == new_positions->size());
        Vector3D delta_pi = new_positions->at(i);
        Vector3D last_pos = pm->position;
        pm->last_position = last_pos;
        pm->position = delta_pi;
    }
    std::cout << "DELTA PI POSITION UPDATES" << std::endl;

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

void isotropicKernel(){

}

//For density estimation
//r is the offset from the kernel center


//smoothing length as 1 double

//where h is the smoothing length
//r is the square magnitude of distance vector
double WaterCube::poly6Kernel(Vector3D r, double h){
    double r_norm = r.norm();
    if(r_norm >= 0 && r_norm <= h){
        return (315/(64 * M_PI * std::pow(h, 9))) * std::pow(std::pow(h, 2) - std::pow(r_norm, 2), 3);
    }else{
        return 0.0;
    }
}

//For gradient calculation
double WaterCube::spikyKernel(Vector3D r, double h){
    double r_norm = r.norm();
    if(r_norm >= 0 && r_norm <= h){
        return (15/M_PI * std::pow(h, 6)) * std::pow(h - r_norm, 3);
    }else{
        return 0.0;
    }
}

//Return rho_i - the particle pi density
double WaterCube::sphDensityEstimatorPoly(Particle pi, double h){
    double sum = 0.0;
    for(int i = 0; i < pi.neighbors.size(); i++){
        Particle pj = *pi.neighbors[i];
        Vector3D diff = pi.position - pj.position;
        sum += poly6Kernel(diff, h);
    }
    return sum;
}

double WaterCube::sphDensityEstimatorSpiky(Particle pi, double h){
    double sum = 0.0;
    for(int i = 0; i < pi.neighbors.size(); i++){
        Particle pj = *pi.neighbors[i];
        Vector3D diff = pi.position - pj.position;
        sum += spikyKernel(diff, h);
    }
    return sum;
}

//Caluclates the gradient of the constraint function
double WaterCube::gradientOfConstraint(Particle pi, double h, Particle pk, double rho_0){
    //See if k is a neighboring particle
    bool kIsNeighbor = false;
    for(int i = 0; i < pi.neighbors.size(); i++){
        Particle neighbor = *pi.neighbors[i];
        if(neighbor.position == pk.position){
            kIsNeighbor = true;
            break;
        }
    }

    if(kIsNeighbor){
        return (1.0/rho_0) * sphDensityEstimatorSpiky(pi, h);
    }else{
        return -1 * (1.0/rho_0) * spikyKernel(pi.position - pk.position, h);
    }
}

//Confused about i and j here and sumK
//e is relaxation parameter
double WaterCube::calcuateLambdaI(Particle pi, double e, double rho_0, double h){

    double sumK = 0.0;
    for(int k = 0; k < water_particles.size(); k++) {
        Particle *pk = &water_particles[k];
        sumK += std::pow(std::abs(gradientOfConstraint(pi,  h, *pk, rho_0)), 2);
    }
    double rho_i = sphDensityEstimatorPoly(pi,h);
    double Ci = (rho_i / rho_0) - 1;
    double lambdaI = -1 * (Ci/sumK + e);
    return lambdaI;
}

//constant = 0.1, n = 4, delta_q = 0.2h
double WaterCube::artificialPressure(Particle pi, Particle pj, double k, int n, Vector3D delta_q, double h){
    return -1 * k * std::pow((poly6Kernel(pi.position - pj.position, h)/(poly6Kernel(delta_q, h))), n);
}

Vector3D WaterCube::positionUpdate(Particle pi, double e, double rho_0, double h){
    double lambdaI = calcuateLambdaI(pi, e, rho_0, h);
    double sumJ = 0.0;
    for(int i = 0; i < pi.neighbors.size(); i++) {
        Particle pj = *pi.neighbors[i];
        double lambdaJ = calcuateLambdaI(pj, e, rho_0, h);
        double sCorr = artificialPressure(pi, pj, 0.1, 4, Vector3D(1,1,1), h);
        sumJ = (lambdaI + lambdaJ + sCorr) * spikyKernel(pi.position - pj.position, h);
    }
    return (1/rho_0) * sumJ;
}

void WaterCube::explicitEuler(Particle pi, double delta_t){
    Vector3D last_pos = pi.position;
    pi.position = pi.position + (delta_t * pi.velocity(delta_t));
    pi.last_position = last_pos;
    //velocity already calulated properly
}

Vector3D WaterCube::vorticity(Particle pi, double h){
//    Vector3D v_sum = 0.0;
//    for(int i = 0; i < pi.neighbors.size(); i++) {
//        Particle *pj = &pi.neighbors[i];
//        v_sum += (pj->vorticity - pi.vorticity);
//        Vector3D wi = cross(v_sum, spikyKernel(pi.position - pj->position, h));
//        Vector3D n = spikyKernel(std::abs(wi),h);
//    }
//
//
//    for(int i = 0; i < pi.neighbors.size(); i++) {
//        Particle *pj = &pi.neighbors[i];
//    }


}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void WaterCube::reset() {
  Particle *pm = &water_particles[0];
  for (int i = 0; i < water_particles.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void WaterCube::buildWaterCubeMesh() {
  if (water_particles.size() == 0) return;
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
