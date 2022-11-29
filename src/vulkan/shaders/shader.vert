#version 450

layout(binding = 0) uniform UniformBufferObject {
    mat4 mvp;
    vec4 color;
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec2 inTexCoord;

layout(location = 0) out vec4 fragTexColor;
layout(location = 1) out vec2 fragTexCoord;

void main() {
    gl_Position = ubo.mvp * vec4(inPosition, 1.0);
    fragTexCoord = inTexCoord;
    fragTexColor = ubo.color;
}