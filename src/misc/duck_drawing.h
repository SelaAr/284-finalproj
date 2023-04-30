//
// Created by Riddhi Bagadiaa on 26/04/23.
//

#ifndef CLOTHSIM_DUCK_DRAWING_H
#define CLOTHSIM_DUCK_DRAWING_H

#include <vector>

#include <nanogui/nanogui.h>

#include "CGL/CGL.h"

using namespace nanogui;

namespace CGL {
    namespace Misc {

        class DuckMesh {
        public:
            // Supply the desired number of vertices
            // TODO: Need to figure num_lat and num_lon and change it
            DuckMesh();
            std::vector<double> verts;
            std::vector<int> indices;

            /**
             * Draws a duck with the given position and radius in opengl, using the
             * current modelview/projection matrices and color/material settings.
             */
            void draw_duck(GLShader &shader, const Vector3D &p);
        private:

            int s_index(int x, int y);

            void build_data();

            int duck_num_lat;
            int duck_num_lon;

            int duck_num_vertices;
            int duck_num_indices;

            MatrixXf positions;
            MatrixXf normals;
            MatrixXf uvs;
            MatrixXf tangents;
        };


    } // namespace Misc
} // namespace CGL

#endif //CLOTHSIM_DUCK_DRAWING_H
