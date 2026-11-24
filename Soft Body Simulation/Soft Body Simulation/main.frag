#version 330 core

uniform sampler2DShadow shadowMap;
uniform mat4 depthBiasMVP;
uniform vec3 objectColor;
uniform vec3 ambientLight;
uniform vec3 diffuseLight0;

layout(location = 0) out vec4 color;

in vec3 n;
in vec3 l;
in vec4 shadowCoord;

const float BLUR_WORLD = 0.07;
const float MIN_BLUR_TEX = 3.0;
const float BIAS_WORLD = 0.01;
const float NORMAL_TEX = 1.5;
const float STRENGTH = 0.8;
const int   SAMPLES = 48;

void main(void) {
    vec3 N = normalize(n);
    float cosTheta = clamp(dot(N, l), 0.0, 1.0);

    vec3 gu = vec3(depthBiasMVP[0][0], depthBiasMVP[1][0], depthBiasMVP[2][0]);
    vec3 gv = vec3(depthBiasMVP[0][1], depthBiasMVP[1][1], depthBiasMVP[2][1]);
    vec3 gz = vec3(depthBiasMVP[0][2], depthBiasMVP[1][2], depthBiasMVP[2][2]);
    float texel = 1.0 / float(textureSize(shadowMap, 0).x);
    float cosL = clamp(-dot(N, gz) / length(gz), 0.0, 1.0);

    vec3 sc = shadowCoord.xyz / shadowCoord.w;
    sc += vec3(dot(N, gu), dot(N, gv), dot(N, gz)) * NORMAL_TEX * texel / length(gu) * sqrt(1.0 - cosL * cosL);
    vec2 slope = vec2(dot(N, gu) / dot(gu, gu), dot(N, gv) / dot(gv, gv)) * length(gz) / max(cosL, 0.1);
    float maxDz = 0.3 * length(gz);
    float bias = BIAS_WORLD * length(gz);
    float R = max(BLUR_WORLD * length(gu), MIN_BLUR_TEX * texel);

    vec2 d = vec2(1.0, 0.0);
    float lit = 0.0, wsum = 0.0;
    for (int i = 0; i < SAMPLES; i++) {
        float t = (float(i) + 0.5) / float(SAMPLES);
        vec2 o = d * sqrt(t) * R;
        float dz = clamp(dot(o, slope), -maxDz, maxDz);
        lit += (1.0 - t) * texture(shadowMap, vec3(sc.xy + o, sc.z + dz - bias));
        wsum += 1.0 - t;
        d = vec2(d.x * -0.7373689 - d.y * 0.6754903, d.x * 0.6754903 - d.y * 0.7373689);
    }
    lit /= wsum;

    float visibility = 1.0 - (1.0 - lit) * STRENGTH;
    color = vec4(objectColor * (ambientLight + visibility * diffuseLight0 * cosTheta), 1.0);
}
