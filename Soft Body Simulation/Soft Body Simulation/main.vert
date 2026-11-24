#version 330 core

uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform mat4 depthBiasMVP;
uniform vec3 lightPos;

layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;

out vec3 n;
out vec3 l;
out vec4 shadowCoord;

void main() {
    gl_Position = MVP * vec4(pos, 1.0);
    gl_PointSize = 20.0 / length(gl_Position);

    shadowCoord = depthBiasMVP * vec4(pos, 1.0);

    n = normalize((M * vec4(normal, 0.0)).xyz);
    l = normalize((M * vec4(lightPos, 0.0)).xyz);
}
