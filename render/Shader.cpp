#include "Shader.h"

cShader::cShader()
{
}

cShader::~cShader()
{
}

void cShader::Bind()
{
	cDrawUtil::BindShader(this);
}