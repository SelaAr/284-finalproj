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

WaterCube::WaterCube(Vector3D cube_origin, double cube_width, double cube_height, int num_particles, Cube wCube, std::vector<Plane *> * borders, int id_count) {
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

Vector3D calculateVelocityFullStep(Particle &pm){
    return (pm.velocity_minus + pm.velocity_plus)/2;
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
        //(velocity_minus + velocity_plus)/2
        Vector3D velocity = Vector3D(0.0,0.0,0.0); // Vector3D vel = Vector3D(0,-0.0001,0);

        float density = 0.0f;
        Particle particle = Particle(pos, velocity, id_count, mass, density, radius);
        id_count += 1;
        std::vector<Particle *> * v = new std::vector<Particle*>();
        particle.neighbors = *v;
        water_particles.push_back(particle);
    }
    //generateFormedParticles();
}

void WaterCube::addParticles(int num_new_particles, Vector3D velocity) {
//
//    for(int i = 0; i < num_new_particles; i++){
//        //x needs to be in the range from origin.x to origin.x + cube_width
//        double x = generateRanBetween(cube_origin.x, cube_origin.x + cube_width);
//        //y needs to be in the range from origin.y to origin.y + cube_width
//        double y = cube_origin.y + cube_height - 0.0001;
//        //z needs to be in the range from origin.z to origin.z + cube_height
//        double z = generateRanBetween(cube_origin.z, cube_origin.z + cube_width);;
//        Vector3D pos = Vector3D(x, y, z);
//        Vector3D vel = Vector3D(0.0,0.0,0.0);
//        double volume = std::pow(cube_width, 3);
//        double mass = (998.20 * volume) / num_particles;
//        Particle particle = Particle(pos, vel, id_count, mass, radius);
//        id_count += 1;
//        std::vector<Particle *> * v = new std::vector<Particle*>();
//        particle.neighbors = *v;
//        water_particles.push_back(particle);
//    }
//
//    this->num_particles += num_new_particles;
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



void WaterCube::generateFormedParticles() {
//grids positions but plus 0.05
    float h = 0.0457;
    for(float i = cube_origin.x; i < cube_width; i += h){
        for(float j = cube_origin.y; j < cube_width; j += h){
            for(float k = cube_origin.z; k < cube_height; k += h){
                Vector3D pos = Vector3D(i + radius, j + radius, k + radius);
                Vector3D velocity = Vector3D(0.0,0.0,0.0); // Vector3D vel = Vector3D(0,-0.0001,0);
                float density = 0.0f;
                Particle particle = Particle(pos, velocity, id_count, mass, density, radius);
                id_count += 1;
                std::vector<Particle *> * v = new std::vector<Particle*>();
                particle.neighbors = *v;
                water_particles.push_back(particle);
            }
        }
    }
    num_particles =  water_particles.size();
}


Vector3D getNormalOfPlane(Vector3D p1, Vector3D p2, Vector3D p3){
    Vector3D a = p2 - p1;
    Vector3D b = p3 - p1;
    Vector3D normal = cross(a,b);
    return normal;
}

bool withinRadius(Particle p1, Particle p2, double radius){
    double f = (p1.position - p2.position).norm();
    if((p1.position - p2.position).norm() <= radius){
        return true;
    }
    return false;
}


void WaterCube::buildHashGrid(float h){
    //Create an array of size numX * numY * numZ + 1
    ////std::cout << "BUILD HASH GRID FUNCTION" << std::endl;
    int numY = cube_width/h;
    int numX = numY;
    int numZ = cube_height/h;
    int size = numX * numY * numZ + 1;

    //Keeps count of the particles in each square of the grid
    //vector<int> vec(size, 0);

    std::vector<int> * vec = new std::vector<int>(size, 0);
    cellCounts = vec;

//    //std::cout << "SET cellCOUNTS TO vec" << std::endl;
//
//    //std::cout << "cellCounts Size: ";
//    //std::cout << cellCounts->size() << std::endl;
//    for(int i = 0; i < cellCounts->size(); i++){
//        //std::cout << "i: ";
//        //std::cout << i;
//        //std::cout << " ";
//        //std::cout << cellCounts->at(i) << std::endl;
//    }

    //Fill the count array - compute hash value of the cell and increase the corresponding value in array
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        int cell = hash_position(pm->position, h);
//        //std::cout << "CELL: " << std::endl;
//        //std::cout << cell << std::endl;
        int curr_count = cellCounts->at(cell);
        // std::vector<int> cells = *cellCounts;
        cellCounts->at(cell) = curr_count + 1;
    }
//    //std::cout << "FILLED COUNT ARRAY" << std::endl;

    //Run through the cellCounts array and compute the partial sum
    //Partial sum: where each corresponding element is the sum of all the elements right to it (including it)
    int sum = 0;
    for(int i = 0; i < cellCounts->size(); i++){
        int curr_sum = cellCounts->at(i);
        sum += curr_sum;
        cellCounts->at(i) = sum;
        //std::vector<int> cells = *cellCounts;
        //cells[i] = sum;
    }
    // //std::cout << "COMPUTED PARTIAL SUM" << std::endl;

    //Create an array of size number of particles
    std::vector<Particle *> * pVec = new std::vector<Particle *>(num_particles, nullptr);
    particleMap = pVec;

    ////std::cout << "Set particleMap to pVec" << std::endl;
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        //1. Index into cellCount based on particle hash
        int cell = hash_position(pm->position, h);
        //2. Decrement cellCount at this index
        int newVal = cellCounts->at(cell) - 1;
        //std::vector<int> cells = *cellCounts;
        //cells[i] = newVal;
        cellCounts->at(cell) = newVal;

        //3. Use this new integer as an index in particleMap - store the pointer of the particle at this index in particleMap
//        std::vector<Particle *> pMap = *particleMap;
//        pMap[newVal] = pm;
        particleMap->at(newVal) = pm;
    }
    ////std::cout << "CREATED PARTICLE POINTER ARRAY" << std::endl;
}


bool WaterCube::inBounds(Vector3D p, double h){
    if(p.x < cube_origin.x || p.x >= cube_origin.x + h ||
       p.y < cube_origin.y || p.y >= cube_origin.y + h ||
       p.z < cube_origin.z || p.z >= cube_origin.z + h){
        return false;
    }
    return true;
}


void WaterCube::getNeighbors(Particle &pm, double h){
    buildHashGrid(h);
    //std::cout << "BUILT HASH GRID" << std::endl;

    ////std::cout << "GET NEIGHBORS"<< std::endl;
    //The particle actual cell
    int pmCell = hash_position(pm.position, h);
    // //std::cout << "PM CELL" << std::endl;

    //Get current cell coordinates
    double numY = std::floor(pm.position.y/h) * h;
    double numX = std::floor(pm.position.x/h) * h;
    double numZ = std::floor(pm.position.z/h) * h;
    //std::cout << "CURRENT CELL COORDINATES" << std::endl;

    //Get Max 27 surrounding cells including current cell
    vector<Vector3D> gridCells;
    for(double x = numX - h; x <= numX + h; x += h){
        for(double y = numY - h; y <= numY + h; y += h){
            for(double z = numZ - h; z <= numZ +h; z += h){
                Vector3D pCell = Vector3D(x, y, z);
                //Check if cell is within bounds
                if(inBounds(pCell, h)){
                    gridCells.push_back(pCell);
                    // //std::cout << pCell << std::endl;
                }
            }
        }
    }

    //Go through the gridCells and get the neighbors
    for(int i = 0; i < gridCells.size(); i++){
        Vector3D neighborCell = gridCells[i];
//        //std::cout << "NEIGHBOR Position: ";
//        //std::cout << neighborCell << std::endl;

        int cell = hash_position(neighborCell, h);
//        //std::cout << "NEIGHBOR CELL: " ;
//        //std::cout << cell << std::endl;

//        //std::cout << cellCounts->size() << std::endl;
//        //std::cout << "CELL + 1: ";
        int cell1 = cell + 1;
//        //std::cout <<  cellCounts->at(cell1) << std::endl;
//
//        //std::cout << "CELL: ";
//        //std::cout << cellCounts->at(cell) << std::endl;

        int n_particles = cellCounts->at(cell + 1) - cellCounts->at(cell);
//        //std::cout << "N_PARTICLES: " ;
//        //std::cout << n_particles << std::endl;

        if(n_particles > 0){
            //Index into particleMap at index of cellCount[i];
            std::vector<Particle *> pMap = *particleMap;
            int start = cell;
//           //std::cout << "START: ";
//           //std::cout << start << std::endl;
            int end = cellCounts->at(cell + 1); //Not inclusive
//            //std::cout << "END: ";
//            //std::cout << end << std::endl;
            while(start < end) {
                Particle *n = pMap[cellCounts->at(cell)];
                if (pm.id != n->id && (pm.position - n->position).norm() <= h) {
                    //std::cout << "PUSHED BACK NEIGHBOR" << std::endl;
                    pm.neighbors.push_back(n);
                }
                start += 1;
            }
        }
    }//std::cout << "WENT THROUGH GRID CELLS" << std::endl;
}


void WaterCube::getNeighborsNaive(WaterCubeParameters *cp, int H){
    double h = cp->smoothing_length;
    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        for(int j = 0; j < water_particles.size(); j++){
            Particle *pj = &water_particles[j];
            if(pm->id != pj->id && withinRadius(*pm, *pj, h)){
                pm->neighbors.push_back(pj);
                pj->neighbors.push_back(pm);
            }
        }
    }
}

void WaterCube::getNeighborsSpatialMap(Particle &pm, WaterCubeParameters *cp){
    build_spatial_map(cp);
    //std::cout << "BUILT SPATIAL MAP" << std::endl;
    double h = cp->smoothing_length;

    int pmCell = hash_position(pm.position, h);
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
                }
            }
        }
    }

    //std::cout << "HERE" << std::endl;
    //Go through the gridCells and get the neighbors
    for(int i = 0; i < gridCells.size(); i++){
        Vector3D neighborCell = gridCells[i];
        //std::cout << neighborCell << std::endl;
        //std::cout << "NOT HERE" << std::endl;
        int nCell = hash_position(pm.position, h);
        //std::cout << "NOT HERE 2" << std::endl;
        std::vector<Particle *> neighbors = *map[nCell];
        //std::cout << "GOT NEIGHBOR CELLS" << std::endl;
        for(int j = 0; j < neighbors.size(); j++){
            Particle *n = neighbors.at(j);
            if (pm.id != n->id && (pm.position - n->position).norm() <= h) {
                //std::cout << "PUSHED BACK NEIGHBOR" << std::endl;
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

double WaterCube::poly6W2Kernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = (945.0f/(32.0f * M_PI * std::pow(h, 9.0f)));
    if(r_norm >= 0 && r_norm <= h){
        return -1 * scaling_factor * (std::pow(h, 2) - std::pow(r_norm, 2)) * ((3 * std::pow(h, 2)) - (7 * std::pow(r_norm, 2)));
    }else{
        return 0.0f;
    }
}


double WaterCube::spikyKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 45.0/(M_PI * std::pow(h, 6));
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

//Reference: https://www10.cs.fau.de/publications/theses/2010/Staubach_BT_2010.pdf
double WaterCube::viscWKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 15.0f/(2 * M_PI * std::pow(h, 3));
    double a = -1 * (std::pow(r_norm, 3)/(2 * std::pow(h, 3)));
    double b = (std::pow(r_norm, 2)/std::pow(h, 2));
    double c = h/(2 * r_norm);
    if(r_norm >= 0 && r_norm <= h){
        return scaling_factor * (a + b + c - 1);
    }else{
        return 0.0;
    }
}

Vector3D WaterCube::viscGradWKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 15.0f/(2 * M_PI * std::pow(h, 3));
    double a = -1 * ((3 * r_norm)/(2 * std::pow(h, 3)));
    double b = (2/std::pow(h, 2));
    double c = -1 * (h/(2 * std::pow(r_norm, 3)));
    if(r_norm >= 0 && r_norm <= h){
        return (scaling_factor * r) * (a + b + c);
    }else{
        return 0.0;
    }
}

double WaterCube::viscGrad2WKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 45.0f/(M_PI * std::pow(h, 6));
    if(r_norm >= 0 && r_norm <= h){
        return scaling_factor * (h - r_norm);
    }else{
        return 0.0;
    }
}

double WaterCube::pressSpikyWKernel(Vector3D r, float h){
    double r_norm = r.norm();
    float scaling_factor = 15.0/(M_PI * std::pow(h, 6));
    if(r_norm >= 0 && r_norm <= h){
        return scaling_factor * std::pow(h - r_norm, 3);
    }else{
        return 0.0f;
    }
}



//SPF//

float WaterCube::estimate_density(Particle pi, double rho_0, float h){
    float rho_i = 0.0f;
    for(int i = 0; i < pi.neighbors.size(); i++){
        Particle *pj = pi.neighbors.at(i);
        rho_i += pj->mass * poly6Kernel(pi.position - pj->position, h);
    }
    return rho_i;
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
                         vector<CollisionObject *> *collision_objects, int currStep) {
//    double volume = std::pow(cube_width, 3);
    //mass_i / density_i = volume_i
//    double mass = (cp->rest_density/num_particles) *  volume);
    // double delta_t = 1.0f / frames_per_sec / simulation_steps; //They set delta_t to 0.01
    double delta_t = 0.01;
    float k = cp->gas_stiffness; //Gas constant
    float vC = cp->viscosity; //viscosity coefficient

    //build_spatial_map(cp);
    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        getNeighborsSpatialMap(*pm, cp);
    }
    //std::cout <<"GOT NEIGHBORS" << std::endl;

    Vector3D extern_forces = Vector3D(0.0,0.0,0.0);
    for (int i = 0; i < external_accelerations.size(); i++) {
        extern_forces += external_accelerations[i] * mass;
    }

    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        pm->forces = extern_forces;
    }

    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        pm->velocity = pm->velocity + delta_t * pm->forces;
        Vector3D curr_pos = pm->position;
        pm->position = pm->last_position + delta_t * pm->velocity;
        pm->last_position = curr_pos;
    }


    //std::cout <<"EXTERNAL ACCELERATIONS: ";
    //std::cout << extern_forces << std::endl;

//    if(currStep == 0){
//        //std::cout << "LEAP FROG STEP 0" << std::endl;
//        for (int i = 0; i < water_particles.size(); i++) {
//            Particle *pm = &water_particles[i];
//            pm->forces = extern_forces;
//            Vector3D at = pm->forces/pm->mass;
//            pm->velocity_minus = pm->velocity - (0.5 * delta_t * at);
//            pm->velocity_plus = pm->velocity_minus + (delta_t * at);
//            pm->velocity = calculateVelocityFullStep(*pm);
//            //std::cout <<"v_minus: ";
//            //std::cout << pm->velocity_minus << std::endl;
//            //std::cout <<"v_plus: ";
//            //std::cout << pm->velocity_plus << std::endl;
//            //std::cout <<"v: ";
//            //std::cout << pm->velocity << std::endl;
//        }
//    }
    //Estimate density
    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pi = &water_particles[i];
        pi->density = estimate_density(*pi, cp->rest_density, cp->smoothing_length);
    }
    //std::cout <<"Estimated Density for each particle" << std::endl;

    //Get pressure of each particle
    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pi = &water_particles[i];
        pi->pressure = k * (pi->density - cp->rest_density);
    }
    //std::cout <<"Get Pressure of each particle" << std::endl;


    //Get forces
    Vector3D f_pressure = Vector3D(0.0, 0.0, 0.0);
    Vector3D f_viscosity = Vector3D(0.0, 0.0, 0.0);
    Vector3D f_gravity = Vector3D(0, -9.82, 0);
    Vector3D f_surface = Vector3D(0.0, 0.0, 0.0);
    double ci = 0.0;
    Vector3D ni = Vector3D(0.0, 0.0, 0.0);
    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        for (int i = 0; i < pm->neighbors.size(); i++) {
            Particle *pn = pm->neighbors.at(i);
            float pi = pm->pressure;
            float pj = pn->pressure;
            float sFactor = (pi + pj)/2;
            f_pressure += (sFactor * (pn->mass/pj) * pressSpikyWKernel(pm->position - pn->position, cp->smoothing_length));//For all j, not equal i
//            Vector3D nVel = calculateVelocityFullStep(*pn);
//            Vector3D pVel = calculateVelocityFullStep(*pm);
//            //std::cout <<"nVel: ";
//            //std::cout << nVel << std::endl;
//
//            //std::cout <<"pVel: ";
//            //std::cout << pVel << std::endl;
            //Vector3D velDiff = nVel -pVel;

            Vector3D velDiff = pn->velocity - pm->velocity;
            f_viscosity += (velDiff * (pn->mass/pn->density) * viscGrad2WKernel(pm->position - pn->position, cp->smoothing_length)); //For all j, not equal i
            ci += ((pn->mass/pn->density) * poly6Kernel(pm->position - pn->position, cp->smoothing_length)); // For all j
            ni += ((pn->mass/pn->density) * poly6GradKernel(pm->position - pn->position, cp->smoothing_length)); // For all j
            // When ni.norm() > 0 -> then particle i is located near the liquid's surface
        }
        f_pressure = f_pressure * -1;
        f_viscosity = f_viscosity * vC;
        if(ni.norm() >= cp->threshold) {
            f_surface = -1 * cp->surface_tension * poly6W2Kernel(ci * (ni / ni.norm()), cp->smoothing_length);
        }
        //Accumulate internal force fields: f_pressure + f_viscosity
        pm->internal_forces += (f_pressure + f_viscosity);
        //Accumulate external force fields: f_gravity + f_surface
        pm->external_forces += (f_gravity + f_surface);
        //Accumulate both forces
        pm->forces += (pm->internal_forces + pm->external_forces);
        //std::cout <<"Accumulated all forces" << std::endl;
    }

    //Collision handling:
    //Contact point: point of impact - cp
    //Penetration depth: depth through obstacle d -> collisions is when d>0
    //Surface normal: vector at the contact point with direction away from object -> n
//    for (int i = 0; i < water_particles.size(); i++) {
//        Particle *pm = &water_particles[i];
//        for(int j = 0; j < pm->neighbors.size(); j++) {
//            Particle *pn = pm->neighbors.at(j);
//            //cpSphere = c + r * (x - c)/(x - c).norm();;
//            //default x is outside primitive
//            int x = 1;
//            if((pm->position - pn->position).norm() < (2 * pm->radius)){
//                //inside
//                x = -1;
//            }else if((pm->position - pn->position).norm() == (2 * pm->radius)){
//                //on surface
//                x = 0;
//            }
//            //Contact point
//            Vector3D cpoint = pm->position + (pm->radius * ((pn->position - pm->position)/(pn->position - pm->position).norm()));
//            float d = std::abs((pm->position - pn->position).norm() - pm->radius);
//
//            Vector3D n = x * ((pm->position - pn->position)/(pm->position - pn->position).norm());
//            if(d > 0 && (pm->velocity).norm() > 0){
//                pm->position = pm->position + (d * n);
//                float cr = cp->restitution;
//                float scaling_factor = d/(delta_t * (pm->velocity).norm());
//                pm->velocity = pm->velocity - ((1 + (cr * scaling_factor)) * (dot(pm->velocity, n) * n));
//            }
//        }
//    }

    for (int i = 0; i < water_particles.size(); i++) {
        Particle *pi = &water_particles[i];

//        Particle *pm = &water_particles[i];
//        for(int j = 0; j < pm->neighbors.size(); j++) {
//            Particle *pn = pm->neighbors.at(j);
//            //cpSphere = c + r * (x - c)/(x - c).norm();;
//            //default x is outside primitive
//            int x = 1;
//            if((pm->position - pn->position).norm() < (2 * pm->radius)){
//                //inside
//                x = -1;
//            }else if((pm->position - pn->position).norm() == (2 * pm->radius)){
//                //on surface
//                x = 0;
//            }
//            //Contact point
//            Vector3D cpoint = pm->position + (pm->radius * ((pn->position - pm->position)/(pn->position - pm->position).norm()));
//            float d = std::abs((pm->position - pn->position).norm() - pm->radius);
//
//            Vector3D n = x * ((pm->position - pn->position)/(pm->position - pn->position).norm());
//            if(d > 0 && (pm->velocity).norm() > 0){
//                pm->position = pm->position + (d * n);
//                float cr = cp->restitution;
//                float scaling_factor = d/(delta_t * (pm->velocity).norm());
//                Vector3D v_change =  pm->velocity - ((1 + (cr * scaling_factor)) * (dot(pm->velocity, n) * n));
//                pm->velocity = v_change;
//                //pn->velocity = -1 * pm->velocity;
//            }
//
//        }

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
                pi->position = pi->position + delta_t * pi->velocity;

            }
        }
    }




//    for (int i = 0; i < water_particles.size(); i++) {
//        Particle *pm = &water_particles[i];
//        pm->last_position = pm->position;
//    }

//    //Leap frog step
//    if(currStep != 0){
//        for (int i = 0; i < water_particles.size(); i++) {
//            //std::cout << "LEAP FROG STEP N" << std::endl;
//            Particle *pm = &water_particles[i];
//            Vector3D at = pm->forces / pm->mass;
//            Vector3D old_plus = pm->velocity_plus;
//            pm->velocity_plus = pm->velocity_minus + (delta_t * at);
//            pm->velocity_minus = old_plus;
//            pm->velocity = calculateVelocityFullStep(*pm);
//            //std::cout <<"v_minus: ";
//            //std::cout << pm->velocity_minus << std::endl;
//            //std::cout <<"v_plus: ";
//            //std::cout << pm->velocity_plus << std::endl;
//            //std::cout <<"v: ";
//            //std::cout << pm->velocity << std::endl;
//        }
//    }













}


void WaterCube::buildGrid(double h){
    for(double i = 0; i < cube_width; i += h){
        for(double j = 0; j < cube_width; j += h){
            for(double k = 0; k < cube_height; k += h){
                grid.push_back(Vector3D(i, j, k));
                Vector3D pos = Vector3D(i, j, k);
                double x = std::floor(pos.x/h);
                double y = std::floor(pos.y/h);
                double z = std::floor(pos.z/h);
                Vector3D ijk = Vector3D(x * h, y * h, z * h);
                ////std::cout << ijk << std::endl;
            }
        }
    }
    //std::cout << "BUILT GRID" << std::endl;
}


void WaterCube::build_spatial_map(WaterCubeParameters *cp) {
    for (const auto &entry : map) {
        delete(entry.second);
    }
    map.clear();


    for(int i = 0; i < water_particles.size(); i++) {
        Particle *pm = &water_particles[i];
        int cell = hash_position(pm->position, cp->smoothing_length);
        if(map[cell] != nullptr){
            map[cell]->push_back(pm);
        }else{
            std::vector<Particle *> * vec = new std::vector<Particle*>();
            vec->push_back(pm);
            map[cell] = vec;
        }
    }
}

int WaterCube::hash_position(Vector3D pos, double h) {
    int x = std::floor(pos.x/(h * 1.0f));
    int y = std::floor(pos.y/(h * 1.0f));
    int z = std::floor(pos.z/(h * 1.0f));
    int p1 = 73856093;
    int p2 = 19349663;
    int p3 = 83492791;


    return ((x * p1) ^ (y * p2) ^ (z * p3)) % (num_particles * 2 * next_prime);
}

int WaterCube::hash_positionNoBounds(Vector3D pos, double h) {
    int x = std::floor(pos.x/(h * 1.0f));
    int y = std::floor(pos.y/(h * 1.0f));
    int z = std::floor(pos.z/(h * 1.0f));
    return std::abs((x * 92837111) ^ (y * 689287499) ^ (z * 283923481));
}





void WaterCube::self_collide(Particle &pi, Particle &pj, float h) {
    Vector3D x1 = pi.position; // center of particles radius
    Vector3D x2 = pj.position;
    //Overlap of particles and normalized line of centers N
    double r1 = pi.radius;
    double r2 = pj.radius;

    Vector3D epislon = std::max(0.0, r1 + r2 - (x2 - x1).norm());
    Vector3D normal_dir = (x2 - x1)/((x2 - x1).norm());
    Vector3D relative_velocity = pi.velocity - pj.velocity;
    double relative_velocity_normal = dot(relative_velocity, normal_dir);
    Vector3D tangential_velocity = relative_velocity - (relative_velocity_normal * normal_dir);

    //Normal Force
    //Vector3D fn = -1 * (cp->damping *relative_velocity_normal) -



    //Equal but opposite force applied to particle pj
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

void WaterCube::createCube(Vector3D origin, double width, double height, GLShader &shader){
    //Plane 1:Bottom Side
    //Front Bottom Left
    CubePoint p1 = CubePoint(origin);
    //Front Bottom Right
    CubePoint p2 = CubePoint(Vector3D(origin.x + width, origin.y, origin.z));
    //Back Bottom Left
    CubePoint p3 = CubePoint(Vector3D(origin.x, origin.y + width, origin.z));
    //Back Bottom Right
    CubePoint p4 = CubePoint(Vector3D(origin.x + width, origin.y + width, origin.z));


    //Plane 2:Top Side
    //Front Top Left
    CubePoint p5 = CubePoint(Vector3D(origin.x, origin.y, origin.z + height));
    //Front Top Right
    CubePoint p6 = CubePoint(Vector3D(origin.x + width, origin.y, origin.z + height));
    //Back Top Left
    CubePoint p7 = CubePoint(Vector3D(origin.x, origin.y + width, origin.z + height));
    //Back Top Right
    CubePoint p8 = CubePoint(Vector3D(origin.x + width, origin.y + width,origin.z + height));
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

    //Added bottom last
    Plane * bottom = new Plane(p1.pos, getNormalOfPlane(p2.pos, p3.pos, p4.pos), 0.5);
    borders->push_back(bottom);

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

void WaterCube::buildWaterCube(GLShader &shader) {
    //if (water_particles.size() == 0) return;

    createCube(cube_origin, cube_width, cube_height, shader);
    //std::cout << "BUILT SMALLER CUBE" << std::endl;

    //createCube(larger_cube_origin, larger_cube_width, larger_cube_height, shader);
//    //std::cout << "BUILT LARGER CUBE" << std::endl;
}

void WaterCube::release(){
    //Remove lower plane
    //borders->pop_back();
}