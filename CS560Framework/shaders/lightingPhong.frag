/////////////////////////////////////////////////////////////////////////
// Pixel shader for lighting
////////////////////////////////////////////////////////////////////////
#version 330

float pi = 3.14159;
float pi2 = 2*pi;

out vec4 FragColor;

in vec2 texCoord;

uniform int width, height;
uniform vec3 lightPos, eyePos, lightCol;

uniform sampler2D gBufferWorldPos;
uniform sampler2D gBufferNormal;
uniform sampler2D gBufferDiffuse;
uniform sampler2D gBufferSpecular;

void main()
{
    vec2 uv = gl_FragCoord.xy / vec2(width, height);
    vec3 normalVec = texture2D(gBufferNormal, uv).xyz;
    vec3 Kd = texture2D(gBufferDiffuse, uv).xyz; 
    vec3 Ks = vec3(0.04);
    vec3 specular = texture2D(gBufferSpecular, uv).xyz; 
    float shininess = texture2D(gBufferSpecular, uv).w;
    shininess = 10000.0;
    vec4 worldPos = texture2D(gBufferWorldPos, uv);

    vec3 lightVec = lightPos - worldPos.xyz;
    vec3 eyeVec = eyePos - worldPos.xyz;
    vec3 ONE = vec3(1.0, 1.0, 1.0);
    vec3 N = normalize(normalVec);
    vec3 L = normalize(lightVec);
    vec3 V = normalize(eyeVec);
    vec3 H = normalize(L+V);
    vec3 R = 2*dot(N,V)*N -V;
    float NL = max(dot(N,L),0.0);
    float NV = max(dot(N,V),0.0);
    float HN = max(dot(H,N),0.0);

    vec3 diffuseColor = Kd/pi;

    vec3 I = lightCol;
    vec3 Ia = 0.1*lightCol; //material ambient 0.1


    FragColor.xyz = Ia * Kd + I*Kd*NL + I*specular*pow(HN, shininess);

}
