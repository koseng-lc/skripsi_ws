#version 330

in vec3 vert;
in vec3 vert_normal;

uniform vec3 light_pos;

out vec4 out_color;

void main(void){
   vec3 dir = normalize(vert - light_pos);
   float gain = max(dot(normalize(vert_normal),dir), .0f);
   vec3 color = vec3(0.3f, 0.3f, 0.3f);
   vec3 combined_color = clamp(color * 0.2 + color * 0.8 * gain, .0f, 1.0f);
   out_color = vec4(combined_color, 1.0f);
}
