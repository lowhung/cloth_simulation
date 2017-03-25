#version 330

uniform		vec4		gColor;
uniform		vec3		gLightDir;
uniform		vec3		gLightColour;
uniform		vec3		gAmbientColour;

in		vec3	ViewPos;
in		vec3	Normal;
in		vec2	TexCoord;

out		vec4	out_color;

float CalcDiffuse(vec3 world_space_normal, vec3 light_direction)
{
	vec3 n = world_space_normal;
	vec3 l = light_direction;
	float l_dot_n = dot(l, n);
	float diffuse = clamp(l_dot_n, 0, 1);
	return diffuse;
}

vec3 CalculateBRDF(vec3 normal, vec3 light_dir,
	vec3 light_colour, vec3 view_dir, vec3 albedo)
{
	float diffuse_coef = CalcDiffuse(normal, light_dir);
	vec3 result = diffuse_coef * light_colour * albedo.rgb;

	return result;
}

vec3 CalcAmbient(vec3 normal, vec3 albedo)
{
	vec3 ambient = gAmbientColour;
	ambient *= albedo;
	return ambient;
}

void main()
{
	vec3 view_dir = -normalize(ViewPos);
	vec3 norm = normalize(Normal);

	vec3 albedo = gColor.rgb;

	vec3 light_colour = gLightColour;
	vec3 light_dir = gLightDir;

	vec3 light_result = CalculateBRDF(norm, light_dir,
						light_colour, view_dir, albedo);

	vec3 ambient = CalcAmbient(norm, albedo);
	light_result.rgb += ambient;

	out_color = vec4(light_result, gColor.a);
}