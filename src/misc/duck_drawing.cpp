//
// Created by Riddhi Bagadiaa on 26/04/23.
//

#include "duck_drawing.h"
#include <cmath>
#include <nanogui/nanogui.h>

#include "CGL/color.h"
#include "CGL/vector3D.h"

#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5
#define TANGEN_OFFSET 8
// TODO: What is Vertex size
#define VERTEX_SIZE 11

using namespace nanogui;

namespace CGL {
    namespace Misc {

        DuckMesh::DuckMesh()
//                : duck_num_vertices(6646)
//                , duck_num_indices(13280)
        : duck_num_vertices(verts.size())
        , duck_num_indices(indices.size())
                {
          build_data();
        }

//        int DuckMesh::s_index(int x, int y) {
//          return ((x) * (sphere_num_lon + 1) + (y));
//        }

        void DuckMesh::build_data() {

          positions = MatrixXf(4, 3 * duck_num_indices);
          normals = MatrixXf(4, 3 * duck_num_indices);
          uvs = MatrixXf(2, 3 * duck_num_indices);
          tangents = MatrixXf(4, 3 * duck_num_indices);

          for (int i = 0; i < duck_num_indices; i += 3) {
            int vertex_idx1 = indices[i];
            int vertex_idx2 = indices[i+1];
            int vertex_idx3 = indices[i+2];

            Vector3D vector1{verts[vertex_idx1 * 3], verts[vertex_idx1 * 3 + 1], verts[vertex_idx1 * 3 + 2]};
            Vector3D vector2{verts[vertex_idx2 * 3], verts[vertex_idx2 * 3 + 1], verts[vertex_idx2 * 3 + 2]};
            Vector3D vector3{verts[vertex_idx3 * 3], verts[vertex_idx3 * 3 + 1], verts[vertex_idx3 * 3 + 2]};

            // Iterate through all the points and add it to the matrices
            positions.col(i    ) << vector1.x, vector1.y, vector1.z, 1.0;
            positions.col(i + 1) << vector2.x, vector2.y, vector2.z, 1.0;
            positions.col(i + 2) << vector3.x, vector3.y, vector3.z, 1.0;

            normals.col(i    ) << 0.0, 0.0, 0.0, 0.0;
            normals.col(i + 1) << 0.0, 0.0, 0.0, 0.0;
            normals.col(i + 2) << 0.0, 0.0, 0.0, 0.0;

            uvs.col(i    ) << 0.0, 0.0;
            uvs.col(i + 1) << 0.0, 0.0;
            uvs.col(i + 2) << 0.0, 0.0;

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

