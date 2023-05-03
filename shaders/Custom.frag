#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform samplerCube u_texture_cubemap;


in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

vec4 mirror() {
  // YOUR CODE HERE

  vec3 outgoing = u_cam_pos - v_position.xyz;

  vec3 reflected = 2 * (dot(outgoing, v_normal.xyz)) * v_normal.xyz - outgoing;
  

  vec4 ret = texture(u_texture_cubemap, reflected);
  out_color.a = 1;
  return ret;
}

vec4 phong() {
  // YOUR CODE HERE
  // ka = 0.1, kd = u_color, ks = 0.5, Ia = vector of ones, and p = 100


  vec4 ka = vec4(0.1, 0.1, 0.1, 0.1);
  vec4 kd = vec4(0.0, 0.4, 1.0, 0.5);
  vec4 ks = vec4(0.5, 0.5, 0.5, 0.5);
  vec4 Ia = vec4(1.0,1.0,1.0,1.0);
  float p = 100.0;


  vec4 first_term = ka * Ia;

  vec4 l = vec4(u_light_pos, 1.0) - v_position;
  float r_squared = length(l) * length(l);


  vec4 second_term = kd * (vec4(u_light_intensity, 0.0) /r_squared)  * max(0.0, dot(normalize(v_normal),normalize(l)));

  vec4 v = vec4(u_cam_pos, 1.0) - v_position;

  vec4 h = normalize(normalize(l) + normalize(v));

  float power_term = pow(max(0.0, dot(normalize(h), normalize(v_normal))), p);
  vec4 third_term = ks * (vec4(u_light_intensity, 0.0) /r_squared) * power_term;

  vec4 ret = first_term + second_term + third_term;


  ret.a = 1;
  return ret;
}


void main() {
  vec4 mirror = mirror();
  vec4 phong = phong();

  out_color = (0.01 * mirror) + (0.99 * phong);
  out_color.a = 0.4;
}
