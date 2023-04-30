#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include "misc/getopt.h" // getopt for windows
#else
#include <getopt.h>
#include <unistd.h>
#endif
#include <unordered_set>
#include <stdlib.h> // atoi for getopt inputs

#include "CGL/CGL.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "cloth.h"
#include "clothSimulator.h"
#include "json.hpp"
#include "misc/file_utils.h"
#include <iostream>
#include <vector>
#include <cstdio>
#include "misc/duck_drawing.h"
#include "collision/rubber_duck.h"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ClothSim] " << s << endl;

const string SPHERE = "sphere";
const string PLANE = "plane";
const string CLOTH = "cloth";

const unordered_set<string> VALID_KEYS = {SPHERE, PLANE, CLOTH};

ClothSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Cloth Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene\n");
  printf("  -r     <STRING>    Project root.\n");
  printf("                     Should contain \"shaders/Default.vert\".\n");
  printf("                     Automatically searched for by default.\n");
  printf("  -a     <INT>       Sphere vertices latitude direction.\n");
  printf("  -o     <INT>       Sphere vertices longitude direction.\n");
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

bool loadObjectsFromDuck(string filename, vector<CollisionObject *>* objects, Misc::DuckMesh* duckmesh) {
  // , Misc::DuckMesh* duckmesh
  // Read JSON from file
  ifstream i(filename);
  if (!i.good()) {
    return false;
  }
  json j;
  i >> j;

  duckmesh->verts = j["verts"].get<vector<double>>();
  duckmesh->indices = j["indices"].get<vector<int>>();
  Duck* d = new Duck(Vector3D{0}, 0.2, duckmesh);
//  vector<double> verts = j["verts"].get<vector<double>>();
//  vector<int> indices = j["indices"].get<vector<int>>();
  objects->push_back(d);
  i.close();
  return true;
}

bool loadObjectsFromFile(string filename, Cloth *cloth, ClothParameters *cp, vector<CollisionObject *>* objects, int sphere_num_lat, int sphere_num_lon) {
  // Read JSON from file
  ifstream i(filename);
  if (!i.good()) {
    return false;
  }
  json j;
  i >> j;

  // Loop over objects in scene
  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();

    // Check that object is valid
    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
    if (query == VALID_KEYS.end()) {
      cout << "Invalid scene object found: " << key << endl;
      exit(-1);
    }

    // Retrieve object
    json object = it.value();

    // Parse object depending on type (cloth, sphere, or plane)
    if (key == CLOTH) {
      // Cloth
      double width, height;
      int num_width_points, num_height_points;
      float thickness;
      e_orientation orientation;
      vector<vector<int>> pinned;

      auto it_width = object.find("width");
      if (it_width != object.end()) {
        width = *it_width;
      } else {
        incompleteObjectError("cloth", "width");
      }

      auto it_height = object.find("height");
      if (it_height != object.end()) {
        height = *it_height;
      } else {
        incompleteObjectError("cloth", "height");
      }

      auto it_num_width_points = object.find("num_width_points");
      if (it_num_width_points != object.end()) {
        num_width_points = *it_num_width_points;
      } else {
        incompleteObjectError("cloth", "num_width_points");
      }

      auto it_num_height_points = object.find("num_height_points");
      if (it_num_height_points != object.end()) {
        num_height_points = *it_num_height_points;
      } else {
        incompleteObjectError("cloth", "num_height_points");
      }

      auto it_thickness = object.find("thickness");
      if (it_thickness != object.end()) {
        thickness = *it_thickness;
      } else {
        incompleteObjectError("cloth", "thickness");
      }

      auto it_orientation = object.find("orientation");
      if (it_orientation != object.end()) {
        orientation = *it_orientation;
      } else {
        incompleteObjectError("cloth", "orientation");
      }

      auto it_pinned = object.find("pinned");
      if (it_pinned != object.end()) {
        vector<json> points = *it_pinned;
        for (auto pt : points) {
          vector<int> point = pt;
          pinned.push_back(point);
        }
      }

      cloth->width = width;
      cloth->height = height;
      cloth->num_width_points = num_width_points;
      cloth->num_height_points = num_height_points;
      cloth->thickness = thickness;
      cloth->orientation = orientation;
      cloth->pinned = pinned;

      // Cloth parameters
      bool enable_structural_constraints, enable_shearing_constraints, enable_bending_constraints;
      double damping, density, ks;

      auto it_enable_structural = object.find("enable_structural");
      if (it_enable_structural != object.end()) {
        enable_structural_constraints = *it_enable_structural;
      } else {
        incompleteObjectError("cloth", "enable_structural");
      }

      auto it_enable_shearing = object.find("enable_shearing");
      if (it_enable_shearing != object.end()) {
        enable_shearing_constraints = *it_enable_shearing;
      } else {
        incompleteObjectError("cloth", "it_enable_shearing");
      }

      auto it_enable_bending = object.find("enable_bending");
      if (it_enable_bending != object.end()) {
        enable_bending_constraints = *it_enable_bending;
      } else {
        incompleteObjectError("cloth", "it_enable_bending");
      }

      auto it_damping = object.find("damping");
      if (it_damping != object.end()) {
        damping = *it_damping;
      } else {
        incompleteObjectError("cloth", "damping");
      }

      auto it_density = object.find("density");
      if (it_density != object.end()) {
        density = *it_density;
      } else {
        incompleteObjectError("cloth", "density");
      }

      auto it_ks = object.find("ks");
      if (it_ks != object.end()) {
        ks = *it_ks;
      } else {
        incompleteObjectError("cloth", "ks");
      }

      cp->enable_structural_constraints = enable_structural_constraints;
      cp->enable_shearing_constraints = enable_shearing_constraints;
      cp->enable_bending_constraints = enable_bending_constraints;
      cp->density = density;
      cp->damping = damping;
      cp->ks = ks;
    } else if (key == SPHERE) {
      Vector3D origin;
      double radius, friction;

      auto it_origin = object.find("origin");
      if (it_origin != object.end()) {
        vector<double> vec_origin = *it_origin;
        origin = Vector3D(vec_origin[0], vec_origin[1], vec_origin[2]);
      } else {
        incompleteObjectError("sphere", "origin");
      }

      auto it_radius = object.find("radius");
      if (it_radius != object.end()) {
        radius = *it_radius;
      } else {
        incompleteObjectError("sphere", "radius");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("sphere", "friction");
      }

      Sphere *s = new Sphere(origin, radius, friction, sphere_num_lat, sphere_num_lon);
      objects->push_back(s);
    } else { // PLANE
      Vector3D point, normal;
      double friction;

      auto it_point = object.find("point");
      if (it_point != object.end()) {
        vector<double> vec_point = *it_point;
        point = Vector3D(vec_point[0], vec_point[1], vec_point[2]);
      } else {
        incompleteObjectError("plane", "point");
      }

      auto it_normal = object.find("normal");
      if (it_normal != object.end()) {
        vector<double> vec_normal = *it_normal;
        normal = Vector3D(vec_normal[0], vec_normal[1], vec_normal[2]);
      } else {
        incompleteObjectError("plane", "normal");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("plane", "friction");
      }

      Plane *p = new Plane(point, normal, friction);
      objects->push_back(p);
    }
  }

  i.close();
  
  return true;
}

bool is_valid_project_root(const std::string& search_path) {
    std::stringstream ss;
    ss << search_path;
    ss << "/";
    ss << "shaders/Default.vert";
    
    return FileUtils::file_exists(ss.str());
}

// Attempt to locate the project root automatically
bool find_project_root(const std::vector<std::string>& search_paths, std::string& retval) {
  
  for (std::string search_path : search_paths) {
    if (is_valid_project_root(search_path)) {
      retval = search_path;
      return true;
    }
  }
  return false;
}

// ********************************************************************************************
//
// Created by Riddhi Bagadiaa on 27/04/23.
//

struct vec2
{
public:
    vec2(float _u, float _v) : u(_u), v(_v) { }

    float u;
    float v;
};

struct idx3
{
public:
    idx3(int _a, int _b, int _c) : a(_a), b(_b), c(_c) { }

    bool operator==(const idx3& other) const {
      if( this->a == other.a && this->b == other.b && this->c == other.c) {
        return true;
      }

      return false;
    }

    int a;
    int b;
    int c;
};

struct vec4
{
public:
    vec4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) { }

    float x;
    float y;
    float z;
    float w;
};

struct tri
{
public:
    tri(int _v1, int _v2, int _v3) : v1(_v1), v2(_v2), v3(_v3), vn1(0), vn2(0), vn3(0), vt1(0), vt2(0), vt3(0) { }

    int v1;
    int vn1;
    int vt1;

    int v2;
    int vn2;
    int vt2;

    int v3;
    int vn3;
    int vt3;
};

struct polygroup
{
public:
    std::vector<vec4>    verts;
    std::vector<vec4>    normals;
    std::vector<vec2>    texcoords;
    std::vector<tri>    tris;
};

struct polygroup_denormalized
{
public:
    std::vector<vec4>    verts;
    std::vector<vec4>    normals;
    std::vector<vec2>    texcoords;
    std::vector<int>    indexbuf;
};

void echo(const char* line)
{
  std::cout << line << std::endl;
}

vec4 parseVertex(const char* line)
{
  char prefix[4];
  float x, y, z;

  sscanf(line, "%s %f %f %f", prefix, &x, &y, &z);

  return vec4(x,y,z,1);
}

vec2 parseTexCoord(const char* line)
{
  char prefix[4];
  float u, v;

  sscanf(line, "%s %f %f", prefix, &u, &v);

  return vec2(u,v);
}


std::vector<int> readFace(const char* fstr)
{
  std::vector<int> ret;

  char buf[64];
  int bufidx = 0;
  for(int i=0; i<strlen(fstr); i++) {

    if(fstr[i] != '/') {
      buf[bufidx++] = fstr[i];
    } else {
      ret.push_back( atoi(buf) );
      bufidx = 0;
      memset(buf, 0, 64); // clear buffer
    }
  }

  if(strlen(buf) > 0) {
    ret.push_back( atoi(buf) );
  }

  return ret;
}

tri parseTriFace(const char* line)
{
  char prefix[4];
  char p1[64];
  char p2[64];
  char p3[64];

  int v1=0, v2=0, v3=0;
  int vn1=0, vn2=0, vn3=0;
  int vt1=0, vt2=0, vt3=0;

  sscanf(line, "%s %s %s %s", prefix, p1, p2, p3);

  std::vector<int> f1 = readFace(p1);
  if(f1.size() >= 1) { v1 = f1[0] - 1; }
  if(f1.size() >= 2) { vt1 = f1[1] - 1; }
  if(f1.size() >= 3) { vn1 = f1[2] - 1; }

  std::vector<int> f2 = readFace(p2);
  if(f2.size() >= 1) { v2 = f2[0] - 1; }
  if(f2.size() >= 2) { vt2 = f2[1] - 1; }
  if(f2.size() >= 3) { vn2 = f2[2] - 1; }

  std::vector<int> f3 = readFace(p3);
  if(f3.size() >= 1) { v3 = f3[0] - 1; }
  if(f3.size() >= 2) { vt3 = f3[1] - 1; }
  if(f3.size() >= 3) { vn3 = f3[2] - 1; }


  tri ret(v1, v2, v3);
  ret.vt1 = vt1;
  ret.vt2 = vt2;
  ret.vt3 = vt3;
  ret.vn1 = vn1;
  ret.vn2 = vn2;
  ret.vn3 = vn3;

  return ret;
}

std::vector<polygroup*> polygroups_from_obj(const char* filename)
{
  bool inPolyGroup = false;
  polygroup* curPolyGroup = NULL;
  std::vector<polygroup*>    polygroups;

  FILE* fp = fopen(filename, "r");
  if(fp == NULL) {
    echo("ERROR: Input file not found");
    return polygroups;
  }

  // make poly group
  if(curPolyGroup == NULL) {
    curPolyGroup = new polygroup();
    polygroups.push_back(curPolyGroup);
  }

  // parse
  echo("reading OBJ geometry data...");
  while(true) {

    char buf[2056];
    if(fgets(buf, 2056, fp) != NULL) {

      if(strlen(buf) >= 1) {

        // texture coordinate line
        if(strlen(buf) >= 2 && buf[0] == 'v' && buf[1] == 't') {
          vec2 tc = parseTexCoord(buf);
          curPolyGroup->texcoords.push_back(tc);
        }
          // vertex normal line
        else if(strlen(buf) >= 2 && buf[0] == 'v' && buf[1] == 'n') {
          vec4 vn = parseVertex(buf);
          curPolyGroup->normals.push_back(vn);
        }
          // vertex line
        else if(buf[0] == 'v') {
          vec4 vtx = parseVertex(buf);
          curPolyGroup->verts.push_back(vtx);
        }
          // face line (ONLY TRIANGLES SUPPORTED)
        else if(buf[0] == 'f') {
          tri face = parseTriFace(buf);
          curPolyGroup->tris.push_back(face);
        }
        else
        { }

      }

    } else {
      break;
    }
  }

  fclose(fp);

  return polygroups;
}


std::string int_array_to_json_array(const std::vector<int>& arr)
{
  std::string json = "[";
  for(int i=0; i<arr.size(); i++) {

    char buf[256];
    sprintf(buf, "%i", arr[i]);

    if(i > 0) {
      json.append(",");
    }

    json.append(buf);
  }

  json.append("]");

  return json;
}

std::string vec4_array_to_json_array(const std::vector<vec4>& arr)
{
  std::string json = "[";
  for(int i=0; i<arr.size(); i++) {

    char buf[64];
    sprintf(buf, "%f", arr[i].x);

    if(i > 0) {
      json.append(",");
    }

    json.append(buf);

    sprintf(buf, "%f", arr[i].y);
    json.append(",");
    json.append(buf);

    sprintf(buf, "%f", arr[i].z);
    json.append(",");
    json.append(buf);
  }

  json.append("]");

  return json;
}

std::string vec2_array_to_json_array(const std::vector<vec2>& arr)
{
  std::string json = "[";
  for(int i=0; i<arr.size(); i++) {

    char buf[64];
    sprintf(buf, "%f", arr[i].u);

    if(i > 0) {
      json.append(",");
    }

    json.append(buf);

    sprintf(buf, "%f", arr[i].v);
    json.append(",");
    json.append(buf);
  }

  json.append("]");

  return json;
}


polygroup_denormalized* denormalize_polygroup(polygroup& pg)
{
  polygroup_denormalized* ret = new polygroup_denormalized();

  std::vector<idx3> processedVerts;

  for(int i=0; i<pg.tris.size(); i++) {

    for(int v=0; v<3; v++) {

      idx3 vidx(0,0,0);
      if(v == 0) {
        vidx = idx3(pg.tris[i].v1, pg.tris[i].vn1, pg.tris[i].vt1);
      } else if(v == 1) {
        vidx = idx3(pg.tris[i].v2, pg.tris[i].vn2, pg.tris[i].vt2);
      } else if (v == 2) {
        vidx = idx3(pg.tris[i].v3, pg.tris[i].vn3, pg.tris[i].vt3);
      } else  { }


      // check if we already processed the vert
      int indexBufferIndex = -1;
      for(int pv=0; pv<processedVerts.size(); pv++) {
        if(vidx == processedVerts[pv]) {
          indexBufferIndex = pv;
          break;
        }
      }

      // add to buffers
      if(indexBufferIndex == -1) {

        processedVerts.push_back(vidx);

        ret->verts.push_back(pg.verts[vidx.a]);

        if(pg.normals.size() > 0) {
          ret->normals.push_back(pg.normals[vidx.b]);
        }

        if(pg.texcoords.size() > 0) {
          ret->texcoords.push_back(pg.texcoords[vidx.c]);
        }

        int idx = (int)ret->verts.size() - 1;
        ret->indexbuf.push_back(idx);

      } else {
        ret->indexbuf.push_back(indexBufferIndex);
      }

    }

  }

  return ret;
}

void polygroup_to_json(polygroup& pg, const char* jsonFilename)
{
  echo("denormalizing polygroup...");
  polygroup_denormalized* dpg = denormalize_polygroup(pg);

  echo("making verts array...");
  std::string vertsStr = "";
  vertsStr.append("\"verts\":");
  vertsStr.append(vec4_array_to_json_array(dpg->verts));
  vertsStr.append(",");

  echo("making indices array...");
  std::string indicesStr = "";
  indicesStr.append("\"indices\":");
  indicesStr.append(int_array_to_json_array(dpg->indexbuf));
  indicesStr.append(",");

  echo("making texcoords array...");
  std::string texcoordsStr = "";
  texcoordsStr.append("\"texcoords\":");
  if(dpg->texcoords.size() > 0) {
    texcoordsStr.append(vec2_array_to_json_array(dpg->texcoords));
  } else {
    texcoordsStr.append("[]");
  }
  texcoordsStr.append(",");

  echo("making normals array...");
  std::string normalsStr = "";
  normalsStr.append("\"normals\":");
  if(dpg->normals.size() > 0) {
    normalsStr.append(vec4_array_to_json_array(dpg->normals));
  } else {
    normalsStr.append("[]");
  }


  echo("writing output file...");
  FILE *fp = fopen(jsonFilename, "w");
  fputs("{", fp);
  fputs(vertsStr.c_str(), fp);
  fputs("\n", fp);
  fputs(indicesStr.c_str(), fp);
  fputs("\n", fp);
  fputs(texcoordsStr.c_str(), fp);
  fputs("\n", fp);
  fputs(normalsStr.c_str(), fp);
  fputs("}", fp);
  fclose(fp);

  delete dpg;
  dpg = NULL;
}

// ********************************************************************************************



int main(int argc, char **argv) {
  // Attempt to find project root
  std::vector<std::string> search_paths = {
    ".",
    "..",
    "../..",
    "../../.."
  };
  std::string project_root;
  bool found_project_root = find_project_root(search_paths, project_root);
  
  Cloth cloth;
  ClothParameters cp;
  vector<CollisionObject *> objects;
  Misc::DuckMesh duckmesh;
  
  int c;
  
  int sphere_num_lat = 40;
  int sphere_num_lon = 40;
  
  std::string file_to_load_from;
  bool file_specified = false;
  
  while ((c = getopt (argc, argv, "f:r:a:o:")) != -1) {
    switch (c) {
      case 'f': {
        file_to_load_from = optarg;
        file_specified = true;
        break;
      }
      case 'r': {
        project_root = optarg;
        if (!is_valid_project_root(project_root)) {
          std::cout << "Warn: Could not find required file \"shaders/Default.vert\" in specified project root: " << project_root << std::endl;
        }
        found_project_root = true;
        break;
      }
      case 'a': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lat = arg_int;
        break;
      }
      case 'o': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lon = arg_int;
        break;
      }
      default: {
        usageError(argv[0]);
        break;
      }
    }
  }
  
  if (!found_project_root) {
    std::cout << "Error: Could not find required file \"shaders/Default.vert\" anywhere!" << std::endl;
    return -1;
  } else {
    std::cout << "Loading files starting from: " << project_root << std::endl;
  }

  if (!file_specified) { // No arguments, default initialization
    std::stringstream def_fname;
    def_fname << project_root;
    def_fname << "/scene/pinned2.json";
    file_to_load_from = def_fname.str();
  } else {
    std::stringstream def_fname;
    def_fname << project_root;
    def_fname << file_to_load_from;
    file_to_load_from = def_fname.str();
  }

  bool success;
  if (file_to_load_from == "../scene/rubber_duck.json") {
    success = loadObjectsFromDuck(file_to_load_from, &objects, &duckmesh);
  } else {
    success = loadObjectsFromFile(file_to_load_from, &cloth, &cp, &objects, sphere_num_lat, sphere_num_lon);
  }
  
  if (!success) {
    std::cout << "Warn: Unable to load from file: " << file_to_load_from << std::endl;
  }

  // *****************************************************
  // Converting .obj file to .json file
//  std::cout << "OBJ to JSON converter" << std::endl;
//
//  string inFileName = project_root + "/scene/rubber_duck.obj";
//  string outFileName = project_root + "/scene/rubber_duck.json";
//  char* inputFilename = const_cast<char*>(inFileName.c_str());
//  char* outputFilename = const_cast<char*>(outFileName.c_str());
//
//  std::cout << "reading OBJ data into polygroup..." << std::endl;
//  std::vector<polygroup*> pg = polygroups_from_obj(inputFilename);
//
//  if(pg.size() > 0) {
//    std::cout << "converting polygroup to JSON arrays..." << std::endl;
//    polygroup_to_json(*pg[0], outputFilename);
//  }
//
//  // cleanup
//  for(int i=0; i<pg.size(); i++) {
//    delete pg[i];
//    pg[i] = NULL;
//  }
//  pg.clear();
//
//  std::cout << "done." << std::endl;

  // *****************************************************

  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the Cloth object
  cloth.buildGrid();
  cloth.buildClothMesh();

  // Initialize the ClothSimulator object
  app = new ClothSimulator(project_root, screen);
  app->loadCloth(&cloth);
  app->loadClothParameters(&cp);
  app->loadCollisionObjects(&objects);
  app->init();

  // Call this after all the widgets have been defined

  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window

  setGLFWCallbacks();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    app->drawContents();

    // Draw nanogui
    screen->drawContents();
    screen->drawWidgets();

    glfwSwapBuffers(window);

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }

  return 0;
}
