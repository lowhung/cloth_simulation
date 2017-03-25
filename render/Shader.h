#pragma once

#include <nanogui/glutil.h>
#include "render/DrawUtil.h"

class PLUGIN_EXPORT cShader : public nanogui::GLShader
{
public:
	
	cShader();
	virtual ~cShader();

	virtual void Bind();

protected:
};
