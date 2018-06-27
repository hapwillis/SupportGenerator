#version 330 core

in vec3 Normal; 
in vec3 FragPos;
in float Wireframe;

out vec4 FragColor;

uniform vec3 lightOneDir;
uniform float lightOneInten;

uniform vec3 camLightDir;
uniform float camLightInten;

void main()
{
    if (Wireframe > 0.0) {
        FragColor = vec4(0.0f, 1.0f, 0.0f, 1.0f);
    } else {
        vec4 color = vec4(0.565f, 0.565f, 0.565f, 1.0f);

        float ambient = 0.0;

        vec3 normal = normalize(Normal);
        vec3 lightOneD = normalize(lightOneDir);
        vec3 camLightD = normalize(camLightDir);
        float diffuseOne = lightOneInten * max(dot(normal, lightOneD), 0.0);
        float diffuseTwo = camLightInten * max(dot(normal, camLightD), 0.0);

        FragColor = (diffuseOne + diffuseTwo + ambient) * color;
    }
} 