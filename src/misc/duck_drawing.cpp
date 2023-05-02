//
// Created by Riddhi Bagadiaa on 26/04/23.
//

#include "duck_drawing.h"
#include <cmath>
#include <nanogui/nanogui.h>
//#include "../triangle_face.h"
#include "TriFace.h"

#include "CGL/color.h"
#include "CGL/vector3D.h"

#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5
#define TANGEN_OFFSET 8
#define VERTEX_SIZE 11

using namespace nanogui;

namespace CGL {
    namespace Misc {

        DuckMesh::DuckMesh()
        {
        }

        void DuckMesh::build_data() {
          duck_num_indices = indices.size();
          std::cout << "build_data 1" << std::endl;
          positions = MatrixXf(4, 3 * duck_num_indices);
          normals = MatrixXf(4, 3 * duck_num_indices);
          uvs = MatrixXf(2, 3 * duck_num_indices);
          tangents = MatrixXf(4, 3 * duck_num_indices);

          std::cout << "build_data 2" << std::endl;
          for (int i = 0; i < duck_num_indices; i += 3) {
            std::cout << "build_data 2.1" << std::endl;
            int vertex_idx1 = indices[i] - 1;
            std::cout << "build_data 2.2" << std::endl;
            int vertex_idx2 = indices[i+1] - 1;
            std::cout << "build_data 2.3" << std::endl;
            int vertex_idx3 = indices[i+2] - 1;

            std::cout << "build_data 3" << std::endl;
            Vector3D vector1{verts[vertex_idx1 * 3], verts[vertex_idx1 * 3 + 1], verts[vertex_idx1 * 3 + 2]};
            Vector3D vector2{verts[vertex_idx2 * 3], verts[vertex_idx2 * 3 + 1], verts[vertex_idx2 * 3 + 2]};
            Vector3D vector3{verts[vertex_idx3 * 3], verts[vertex_idx3 * 3 + 1], verts[vertex_idx3 * 3 + 2]};

            faces.push_back(TriFace(vector1, vector2, vector3));

            std::cout << "build_data 4" << std::endl;
            // Iterate through all the points and add it to the matrices
            positions.col(i    ) << vector1.x, vector1.y, vector1.z, 1.0;
            positions.col(i + 1) << vector2.x, vector2.y, vector2.z, 1.0;
            positions.col(i + 2) << vector3.x, vector3.y, vector3.z, 1.0;

            std::cout << "build_data 2.1" << std::endl;
            int norm_idx1 = normindices[i] - 1;
            std::cout << "build_data 2.2" << std::endl;
            int norm_idx2 = normindices[i+1] - 1;
            std::cout << "build_data 2.3" << std::endl;
            int norm_idx3 = normindices[i+2] - 1;

            std::cout << "build_data 3" << std::endl;
            Vector3D norm1{norms[norm_idx1 * 3], norms[norm_idx1 * 3 + 1], norms[norm_idx1 * 3 + 2]};
            Vector3D norm2{norms[norm_idx2 * 3], norms[norm_idx2 * 3 + 1], norms[norm_idx2 * 3 + 2]};
            Vector3D norm3{norms[norm_idx3 * 3], norms[norm_idx3 * 3 + 1], norms[norm_idx3 * 3 + 2]};


            std::cout << "build_data 5" << std::endl;
            normals.col(i    ) << norm1.x, norm1.y, norm1.z, 0.0;
            normals.col(i + 1) << norm2.x, norm2.y, norm2.z, 0.0;
            normals.col(i + 2) << norm3.x, norm3.y, norm3.z, 0.0;

            std::cout << "build_data 6" << std::endl;
            uvs.col(i    ) << 0.0, 0.0;
            uvs.col(i + 1) << 0.0, 0.0;
            uvs.col(i + 2) << 0.0, 0.0;

            std::cout << "build_data 6" << std::endl;
            tangents.col(i    ) << 0.0, 0.0, 0.0, 0.0;
            tangents.col(i + 1) << 0.0, 0.0, 0.0, 0.0;
            tangents.col(i + 2) << 0.0, 0.0, 0.0, 0.0;
          }
        }

        void DuckMesh::draw_duck(GLShader &shader, const Vector3D &p) {

          Matrix4f model;
          model << 1, 0, 0, p.x, 0, 1, 0, p.y, 0, 0, 1, p.z, 0, 0, 0, 1;

          shader.setUniform("u_model", model);


          shader.uploadAttrib("in_position", positions);
          if (shader.attrib("in_normal", false) != -1) {
            shader.uploadAttrib("in_normal", normals);
          }
          if (shader.attrib("in_uv", false) != -1) {
            shader.uploadAttrib("in_uv", uvs);
          }
          if (shader.attrib("in_tangent", false) != -1) {
            shader.uploadAttrib("in_tangent", tangents, false);
          }

          shader.drawArray(GL_TRIANGLES, 0, duck_num_indices);
        }

    } // namespace Misc
} // namespace CGL

