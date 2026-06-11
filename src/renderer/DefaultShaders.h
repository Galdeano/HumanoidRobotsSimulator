#ifndef DEFAULT_SHADERS_H
#define DEFAULT_SHADERS_H

// Vertex Shader
static const char* DEFAULT_VERTEX_SHADER = R"(
#version 330 core
layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Normal;
layout(location = 2) in vec2 in_TexCoord;

out vec3 frag_Position;
out vec3 frag_Normal;
out vec2 frag_TexCoord;

uniform mat4 MVP;
uniform mat4 Model;
uniform mat3 NormalMatrix;

void main() {
    frag_Position = vec3(Model * vec4(in_Position, 1.0));
    frag_Normal = normalize(NormalMatrix * in_Normal);
    frag_TexCoord = in_TexCoord;
    gl_Position = MVP * vec4(in_Position, 1.0);
}
)";

// Fragment Shader
static const char* DEFAULT_FRAGMENT_SHADER = R"(
#version 330 core
in vec3 frag_Position;
in vec3 frag_Normal;
in vec2 frag_TexCoord;

out vec4 out_Color;

uniform vec3 view_Pos;
uniform vec3 light_Pos;
uniform vec3 light_Color;
uniform vec3 ambient_Color;

// Material Uniforms
uniform vec3 material_Ambient;
uniform vec3 material_Diffuse;
uniform vec3 material_Specular;
uniform float material_Shininess;

uniform vec3 base_Color;
uniform bool use_Base_Color;
uniform bool use_Flat_Color;

void main() {
    if (use_Flat_Color) {
        out_Color = vec4(base_Color, 1.0);
        return;
    }

    // Ambient
    vec3 ambient = ambient_Color * (use_Base_Color ? base_Color : material_Ambient);

    // Diffuse
    vec3 norm = normalize(frag_Normal);
    vec3 lightDir = normalize(light_Pos - frag_Position);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * light_Color * (use_Base_Color ? base_Color : material_Diffuse);

    // Specular
    vec3 viewDir = normalize(view_Pos - frag_Position);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfwayDir), 0.0), use_Base_Color ? 32.0 : material_Shininess);
    vec3 specular = spec * light_Color * (use_Base_Color ? vec3(0.5) : material_Specular);

    vec3 result = ambient + diffuse + specular;
    out_Color = vec4(result, 1.0);
}
)";

#endif // DEFAULT_SHADERS_H
