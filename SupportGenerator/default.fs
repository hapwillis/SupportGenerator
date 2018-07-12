#version 330 core

in vec3 Normal; 
in vec3 FragPos;
in float Wireframe;

out vec4 FragColor;

uniform vec3 lightOneDir;
uniform float lightOneInten;

uniform vec3 camLightDir;
uniform float camLightInten;
uniform vec4 color;

void main()
{
    if (Wireframe > 0.0) {
        FragColor = color;
    } else {
        float ambient = 0.0;

        vec3 normal = normalize(Normal);
        vec3 lightOneD = normalize(lightOneDir);
        vec3 camLightD = normalize(camLightDir);
        float diffuseOne = lightOneInten * max(dot(normal, lightOneD), 0.0);
        float diffuseTwo = camLightInten * max(dot(normal, camLightD), 0.0);

        vec4 tcolor = color;
        float angle = acos(dot(normal, vec3(0.0, -1.0, 0.0)));
        if (angle < 0.785398 && angle > 0.001) {
            float redness = color.x + ((1.0 - color.x) * (0.785398 - angle) / 0.785398);
            tcolor = vec4(redness, color.y * angle, color.z * angle, 1.0);
        }

        FragColor = (diffuseOne + diffuseTwo + ambient) * tcolor;
    }
} 