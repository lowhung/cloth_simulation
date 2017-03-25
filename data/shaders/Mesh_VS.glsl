#version 330

uniform mat4 gProjMatrix;
uniform mat4 gModelViewMatrix;

in vec3 inPosition;
in vec3 inNormal;
in vec2 inTexCoord;

out vec3 ViewPos;
out vec3 Normal;
out vec2 TexCoord;

void main()
{
	vec4 view_pos = gModelViewMatrix * vec4(inPosition.xyz, 1.f);
	vec4 WVP_Pos = gProjMatrix * view_pos;

	gl_Position = WVP_Pos;
	ViewPos = view_pos.xyz;
	Normal = (gModelViewMatrix * vec4(inNormal.xyz, 0.f)).xyz;
	TexCoord = inTexCoord.xy;
}