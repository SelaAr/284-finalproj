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
#include "waterCube.h"
#include "waterSimulator.h"
#include "json.hpp"
#include "misc/file_utils.h"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[WaterCubeSim] " << s << endl;

const string SPHERE = "sphere";
const string PLANE = "plane";
const string WCUBE = "wcube";

const unordered_set<string> VALID_KEYS = {SPHERE, PLANE, WCUBE};

WaterSimulator *app = nullptr;
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
    window = glfwCreateWindow(800, 800, "WaterCube Simulator", nullptr, nullptr);
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

bool loadObjectsFromFile(string filename, WaterCube *waterCube, WaterCubeParameters *cp, vector<CollisionObject *>* objects, int sphere_num_lat, int sphere_num_lon) {
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

        // Parse object depending on type (waterCube, sphere, or plane)
        if (key == WCUBE) {
            double width, height, cube_width, cube_height;
            int num_particles;
            Vector3D cube_origin;

            auto it_width = object.find("width");
            if (it_width != object.end()) {
                width = *it_width;
            } else {
                incompleteObjectError("wcube", "width");
            }

            auto it_height = object.find("height");
            if (it_height != object.end()) {
                height = *it_height;
            } else {
                incompleteObjectError("wcube", "height");
            }

            auto it_num_particles = object.find("num_particles");
            if (it_num_particles != object.end()) {
                num_particles = *it_num_particles;
            } else {
                incompleteObjectError("wcube", "num_particles");
            }

            auto it_cube_origin = object.find("cube_origin");
            if (it_cube_origin != object.end()) {
                vector<double> vec_origin = *it_cube_origin;
                cube_origin = Vector3D(vec_origin[0], vec_origin[1], vec_origin[2]);
            } else {
                incompleteObjectError("wcube", "cube_origin");
            }

            auto it_cube_width = object.find("cube_width");
            if (it_cube_width != object.end()) {
                cube_width = *it_cube_width;
            } else {
                incompleteObjectError("wcube", "cube_width");
            }

            auto it_cube_height = object.find("cube_height");
            if (it_cube_height != object.end()) {
                cube_height = *it_cube_height;
            } else {
                incompleteObjectError("wcube", "cube_height");
            }


            waterCube->width = width;
            waterCube->height = height;
            waterCube->num_particles = num_particles;
            waterCube->cube_origin = cube_origin;
            waterCube->cube_width = cube_width;
            waterCube->cube_height = cube_height;
            waterCube->wCube = Cube();
            std::vector<Plane *> * vec = new std::vector<Plane*>();
            waterCube->borders = vec;

            // Cloth parameters
            double damping, density, ks, relaxation_e, rest_density, smoothing_length;

            auto it_damping = object.find("damping");
            if (it_damping != object.end()) {
                damping = *it_damping;
            } else {
                incompleteObjectError("wcube", "damping");
            }

            auto it_density = object.find("density");
            if (it_density != object.end()) {
                density = *it_density;
            } else {
                incompleteObjectError("wcube", "density");
            }

            auto it_ks = object.find("ks");
            if (it_ks != object.end()) {
                ks = *it_ks;
            } else {
                incompleteObjectError("wcube", "ks");
            }


            auto it_relaxation_e = object.find("relaxation_e");
            if (it_relaxation_e != object.end()) {
                relaxation_e = *it_relaxation_e;
            } else {
                incompleteObjectError("wcube", "relaxation_e");
            }


            auto it_rest_density = object.find("rest_density");
            if (it_rest_density != object.end()) {
                rest_density = *it_rest_density;
            } else {
                incompleteObjectError("wcube", "rest_density");
            }


            auto it_smoothing_length = object.find("smoothing_length");
            if (it_smoothing_length != object.end()) {
                smoothing_length = *it_smoothing_length;
            } else {
                incompleteObjectError("wcube", "smoothing_length");
            }
            cp->density = density;
            cp->damping = damping;
            cp->ks = ks;
            cp->relaxation_e = relaxation_e;
            cp->rest_density = rest_density;
            cp->smoothing_length = smoothing_length;

        }else if (key == SPHERE) {
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

    WaterCube waterCube;
    WaterCubeParameters cp;
    vector<CollisionObject *> objects;

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
    }

    bool success = loadObjectsFromFile(file_to_load_from, &waterCube, &cp, &objects, sphere_num_lat, sphere_num_lon);
    if (!success) {
        std::cout << "Warn: Unable to load from file: " << file_to_load_from << std::endl;
    }

    glfwSetErrorCallback(error_callback);

    createGLContexts();

//  //Add plane borders into collision objects
//  for(int i = 0; i < waterCube.borders->size(); i++){
//      Plane *p = waterCube.borders->at(i);
//      objects.push_back(p);
//  }

    // Initialize the WaterCube object
    waterCube.generateParticles();


    std::cout << "POINTS GENERATED" << std::endl;


    // Initialize the WaterSimulator object
    app = new WaterSimulator(project_root, screen);
    app->loadWaterCube(&waterCube);
    app->loadWaterCubeParameters(&cp);
    app->loadCollisionObjects(&objects);
//  app->loadCube(&cubeW);
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