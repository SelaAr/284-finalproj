#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  // (Placeholder code. You will want to replace it.)
    float r2 = pow(distance(v_position, vec4(u_light_pos, 1)), 2);
    vec4 falloff = vec4(u_light_intensity/r2, 1);

    vec4 light_diff = vec4(u_light_pos, 1) - v_position;
    vec4 l = normalize(light_diff);

    vec4 Ia = vec4(1,1,1,1);
    vec3 kd = u_color.xyz;
    float ka =  0.1;
    float ks = 0.9;

  //viewer direction
    vec4 v = normalize(vec4(u_cam_pos, 1) - v_position);

    vec4 h = (v + l)/length(v + l);

    out_color = ((ka * Ia) + (vec4(kd, 1) * falloff * max(0, dot(normalize(v_normal), l))) + (ks * (falloff) * pow(max(0, dot(normalize(v_normal),h)), 25)));
    out_color.a = 1;
}

