#include "render/OBJParser.h"
#include "render/OBJLoader.h"
#include "render/MeshUtil.h"

bool cOBJParser::LoadMesh(const std::string& filename, cDrawMesh& out_mesh)
{
	bool succ = true;
	cObjLoader obj_parser(filename);

	// gather data from the parser to construct mesh
	GLubyte *data = NULL;
	int *indices;
	int data_size, num_indices, num_attr;
	obj_attrib_info *parsed_attr_info;

	// export parser data
	// NOTE: data is owned by the obj_parser
	obj_parser.objExportGLSeparate(data_size, data, num_indices, indices, num_attr, parsed_attr_info);

	if (parsed_attr_info == NULL)
	{
		printf("Mesh Not Found: Failed to load\n");
		return false;
	}

	std::vector<float> vert_data;
	std::vector<int> face_data;

	// populate indices
	for (int i = 0; i < num_indices; i++)
	{
		face_data.push_back(indices[i]);
	}

	// populate vertices from byte array (so lots of pointer pushing)
	int attr_end = data_size;
	int v_stride = parsed_attr_info[0].data_stride;
	int v_offset = parsed_attr_info[0].data_offset;

	if (v_stride == 0)
	{
		for (int i = 0; i < num_attr; i++)
		{
			int off = parsed_attr_info[i].data_offset;
			if (off < attr_end && off > v_offset)
			{
				attr_end = off;
			}
		}
	}

	int      attrib_size = parsed_attr_info[0].attrib_size;
	int      elem_size = attrib_size / parsed_attr_info[0].num_comp;

	GLubyte *pAttrib = data;
	GLubyte *pEnd = data + attr_end;

	// TODO: safety check on number of elements per attribute
	// (there should never be less than 3 but who knows)
	// if (sizeof(float) != elem_size)
	//    unhandled as of right now

	// not particularily safe...
	for (; pAttrib < pEnd; pAttrib += elem_size)
	{
		double tmp = (double)*(float*)pAttrib;
		vert_data.push_back(tmp);
	}

	cMeshUtil::BuildDrawMesh(vert_data.data(), static_cast<int>(vert_data.size()),
							face_data.data(), static_cast<int>(face_data.size()),
							&out_mesh);
	return succ;
}
