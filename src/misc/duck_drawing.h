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
            int duck_num_indices;

            /**
             * Draws a duck with the given position and radius in opengl, using the
             * current modelview/projection matrices and color/material settings.
             */
            void draw_duck(GLShader &shader, const Vector3D &p);
            void build_data();
        private:

            MatrixXf positions;
            MatrixXf normals;
            MatrixXf uvs;
            MatrixXf tangents;
        };


    } // namespace Misc
} // namespace CGL

#endif //CLOTHSIM_DUCK_DRAWING_H
