#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.

  float dist_along_width = width / (1.0 * num_width_points - 1);
  float dist_along_height = height / (1.0 * num_height_points - 1);

  double y = 0;
  double x = 0;
  Vector3D pos;

    for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
        if (orientation == HORIZONTAL) {
          pos = Vector3D(x, 1, y);
        } else {
          double ran = (double) std::rand();
          double min = -1.0/1000;
          double max = 1.0/1000;
          double offset = min + (ran/RAND_MAX) * (max - min);
          pos = Vector3D(x, y, offset);
        }

        bool pin = false;
        vector<int> pin_test{int(i), int(j)};

        if (std::find(pinned.begin(), pinned.end(), pin_test) != pinned.end()) {
          pin = true;
        }

        PointMass point_mass = PointMass(pos, pin);
        point_masses.push_back(point_mass);
        x = x + dist_along_width;
      }
      y = y + dist_along_height;
      x = 0;
    }


  //Spring stuff
  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      int current_index = y * num_width_points + x;

      // Structural
      if (x != 0) {
        int left_point_index = y * num_width_points + x - 1;
        Spring structural_spring = Spring(&point_masses[current_index], &point_masses[left_point_index], STRUCTURAL);
        springs.push_back(structural_spring);
      }

      if (y != 0) {
        int top_point_index = (y - 1) * num_width_points + x;
        Spring structural_spring = Spring(&point_masses[current_index], &point_masses[top_point_index], STRUCTURAL);
        springs.push_back(structural_spring);
      }

      // Shearing
      if (x != 0 && y != 0) {
        int top_left_point_index = (y - 1) * num_width_points + x - 1;
        Spring shearing_spring = Spring(&point_masses[current_index], &point_masses[top_left_point_index], SHEARING);
        springs.push_back(shearing_spring);
      }

      if (x != num_width_points - 1 && y != 0) {
        int top_right_point_index = (y - 1) * num_width_points + x + 1;
        Spring shearing_spring = Spring(&point_masses[current_index], &point_masses[top_right_point_index], SHEARING);
        springs.push_back(shearing_spring);
      }

      // Bending
      if (x > 1) {
        int two_left_point_index = y * num_width_points + x - 2;
        Spring bending_spring = Spring(&point_masses[current_index], &point_masses[two_left_point_index], BENDING);
        springs.push_back(bending_spring);
      }

      if (y > 1) {
        int two_top_point_index = (y - 2) * num_width_points + x;
        Spring bending_spring = Spring(&point_masses[current_index], &point_masses[two_top_point_index], BENDING);
        springs.push_back(bending_spring);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  // clear forces vector
  Vector3D external_force = Vector3D{0.0};
  for (int idx = 0; idx < external_accelerations.size(); idx++) {
    external_force += mass * external_accelerations[idx];
  }

  for (int idx = 0; idx < point_masses.size(); idx++) {
    PointMass *pm = &point_masses[idx];
    pm->forces = external_force;
  }

  for (int idx = 0; idx < springs.size(); idx++) {
    e_spring_type spring_type = springs[idx].spring_type;
    if (spring_type == STRUCTURAL && !cp->enable_structural_constraints) {
      continue;
    }
    else if (spring_type == SHEARING && !cp->enable_shearing_constraints) {
      continue;
    }
    else if (spring_type == BENDING && !cp->enable_bending_constraints) {
      continue;
    }
    double k_s = cp->ks;
    if (spring_type == BENDING) {
      k_s = k_s * 0.2;
    }
    Vector3D dist = (springs[idx].pm_a->position - springs[idx].pm_b->position);
    double spring_force = k_s * (dist.norm() - springs[idx].rest_length);

    springs[idx].pm_a->forces -= (spring_force * dist.unit());
    springs[idx].pm_b->forces += (spring_force * dist.unit());
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (int idx = 0; idx < point_masses.size(); idx++) {
    PointMass *pm = &point_masses[idx];
    if (pm->pinned) {
      continue;
    }

    Vector3D new_pos = pm->position + (1.00 - cp->damping / 100.00) * (pm->position - pm->last_position) + (pm->forces/mass) * delta_t * delta_t;
    pm->last_position = pm->position;
    pm->position = new_pos;
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for(int i = 0; i < point_masses.size(); i++) {
    PointMass *pm = &point_masses[i];
    self_collide(*pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  for(int i = 0; i < point_masses.size(); i++) {
    PointMass *pm = &point_masses[i];
    for (CollisionObject *co: *collision_objects) {
      co->collide(*pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int idx = 0; idx < springs.size(); idx++) {
    Spring spring = springs[idx];
    if (spring.pm_a->pinned && spring.pm_b->pinned) {
      continue;
    }
    double rest_len = spring.rest_length;
    double changed_len = (spring.pm_a->position - spring.pm_b->position).norm();
    double max_new_len = rest_len * 1.10;

    if (changed_len > max_new_len) {
      Vector3D correction = (changed_len - max_new_len) * (spring.pm_a->position - spring.pm_b->position).unit();
      if (spring.pm_a->pinned) {
        spring.pm_b->position += correction;
      } else if (spring.pm_b->pinned) {
        spring.pm_a->position -= correction;
      } else {
        spring.pm_b->position += ((correction/2.0));
        spring.pm_a->position -= ((correction/2.0));
      }
    }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass &pm: point_masses) {
    float hash_pos = hash_position(pm.position);
    if (map[hash_pos] != nullptr) {
      map[hash_pos]->push_back(&pm);
    } else {
      vector<PointMass *> *new_vector = new vector<PointMass *>();
      new_vector->push_back(&pm);
      map[hash_pos] = new_vector;
    }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float hash_pos = hash_position(pm.position);
  vector<PointMass *> *hash_pms = map[hash_pos];

  int count = 0;
  Vector3D correction_sum = Vector3D(0.0, 0.0, 0.0);
  for (PointMass* candidate_pm: *hash_pms) {
    if (candidate_pm == &pm){
      continue;
    }
    if ((candidate_pm->position - pm.position).norm() < (2 * thickness)) {
      double correction_distance = (2 * thickness) - (candidate_pm->position - pm.position).norm();
      Vector3D correction = correction_distance * (pm.position - candidate_pm->position).unit();
      correction_sum += correction;
      count += 1;
    }
  }
  if (count > 0) {
    Vector3D final_correction = (correction_sum / count) * (1.0 / simulation_steps);
    pm.position = pm.position + final_correction;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float box_w =  3.0 * width / (num_width_points * 1.0);
  float box_h = 3.0 * height/ (num_height_points * 1.0);
  float box_t = max(box_w,box_h);

  int box_num_x = floor(pos.x / box_w);
  int box_num_y = floor(pos.y / box_h);
  int box_num_t = floor(pos.z / box_t);

  return (box_num_x * (width + 1)) + (box_num_y * (height + 1)) + (box_num_t);
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}