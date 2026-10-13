#version 330 core

in vec3 normalVec;
in vec2 texCoord;
in vec4 worldPos;

uniform vec3 diffuse;
uniform vec3 specular;
uniform float shininess;
uniform sampler2D diffuseMap;
uniform bool hasDiffuseMap;

void main()
{   	
	gl_FragData[0] = worldPos;
	gl_FragData[1] = vec4(normalVec, 1.0);
	vec3 baseColor = diffuse;
	if (hasDiffuseMap)
    {
        baseColor *= texture(diffuseMap, texCoord).rgb;
    }
    gl_FragData[2] = vec4(baseColor, 1.0);
	gl_FragData[3] = vec4(specular,shininess);

}
