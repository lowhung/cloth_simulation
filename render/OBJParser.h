#pragma once

#include <memory>
#include "render/DrawMesh.h"
#include "util/MathUtil.h"

class PLUGIN_EXPORT cOBJParser
{
public:
	static bool LoadMesh(const std::string& filename, cDrawMesh& out_mesh);

protected:

};
