#version 330

in vec3 vertex;//unique ID 0
in vec3 normal;//unique ID 1
in float instance;//unique ID 2

uniform mat4 m_proj;
uniform mat4 m_view;
uniform mat4 m_model[21];
uniform mat3 m_normal[21];

out vec3 vert;
out vec3 vert_normal;

void main(void){
    int inst = int(instance);
    vert = vertex;
    vert_normal = m_normal[inst] * normal;
    gl_Position = m_proj * m_view * m_model[inst] * vec4(vertex, 1.0f);
}
