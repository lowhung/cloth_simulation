#include "DrawUtil.h"
#include <nanogui/opengl.h>
#include "render/Shader.h"

tVector cDrawUtil::gColor = tVector::Ones();
cDrawUtil::eMatrixMode cDrawUtil::mMatrixMode = cDrawUtil::eMatrixModeModelView;

const int gNumSlice = 16;
const int gNumStacks = 8;
const int gDiskSlices = 32;

std::unique_ptr<cDrawMesh> cDrawUtil::gPointMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gLineMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gQuadMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gBoxMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gSphereMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gDiskMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gTriangleMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gCylinderMesh = nullptr;

std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>> cDrawUtil::mMatrixStackProj = std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>>();
std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>> cDrawUtil::mMatrixStackModelView = std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>>();
cShader* cDrawUtil::gShader = nullptr;

void cDrawUtil::InitDrawUtil()
{
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glFrontFace(GL_CCW);

	BuildMeshes();
	mMatrixMode = eMatrixModeModelView;
	mMatrixStackProj.clear();
	mMatrixStackModelView.clear();
	mMatrixStackProj.push_back(tMatrix::Identity());
	mMatrixStackModelView.push_back(tMatrix::Identity());

	gColor.setIdentity();

	gShader = nullptr;
}

void cDrawUtil::DrawRect(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	tVector a = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector b = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector c = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	tVector d = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	DrawQuad(a, b, c, d, draw_mode);
}

void cDrawUtil::DrawBox(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINE_LOOP;
	
	const int num_faces = 6;
	const int pos_len = num_faces * 6 * cMeshUtil::gPosDim;

	tVector sw0 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector se0 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector ne0 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector nw0 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);

	tVector sw1 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector se1 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector ne1 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector nw1 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);

	const float pos_data[pos_len] = {
		ne0[0], ne0[1], ne0[2], // top
		nw0[0], nw0[1], nw0[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		ne1[0], ne1[1], ne1[2],
		ne0[0], ne0[1], ne0[2],

		se1[0], se1[1], se1[2],  // bottom
		sw1[0], sw1[1], sw1[2],
		sw0[0], sw0[1], sw0[2],
		sw0[0], sw0[1], sw0[2],
		se0[0], se0[1], se0[2],
		se1[0], se1[1], se1[2],

		se1[0], se1[1], se1[2], // front
		se0[0], se0[1], se0[2],
		ne0[0], ne0[1], ne0[2],
		ne0[0], ne0[1], ne0[2],
		ne1[0], ne1[1], ne1[2],
		se1[0], se1[1], se1[2],

		sw0[0], sw0[1], sw0[2], // back
		sw1[0], sw1[1], sw1[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		nw0[0], nw0[1], nw0[2],
		sw0[0], sw0[1], sw0[2],

		sw0[0], sw0[1], sw0[2], // left
		nw0[0], nw0[1], nw0[2],
		ne0[0], ne0[1], ne0[2],
		ne0[0], ne0[1], ne0[2],
		se0[0], se0[1], se0[2],
		sw0[0], sw0[1], sw0[2],

		se1[0], se1[1], se1[2], // right
		ne1[0], ne1[1], ne1[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		sw1[0], sw1[1], sw1[2],
		se1[0], se1[1], se1[2]
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gBoxMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gBoxMesh->Draw(gl_mode);
}

void cDrawUtil::DrawTriangle(const tVector& pos, double side_len, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawWire) ? GL_LINE_LOOP : GL_TRIANGLES;
	PushMatrix();
	Translate(pos);
	Scale(tVector(side_len, side_len, side_len, 1));
	gTriangleMesh->Draw(gl_mode);
	PopMatrix();
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, eDrawMode draw_mode)
{
	DrawQuad(a, b, c, d, tVector(0, 0, 0, 0), tVector(1, 0, 0, 0),
		tVector(1, 1, 0, 0), tVector(0, 1, 0, 0), draw_mode);
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d,
	const tVector& coord_a, const tVector& coord_b, const tVector& coord_c, const tVector& coord_d,
	eDrawMode draw_mode)
{
	const int num_verts = 4;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const int norm_len = num_verts * cMeshUtil::gNormDim;
	const int coord_len = num_verts * cMeshUtil::gCoordDim;

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;

	tVector normal = (b - a).cross3(d - a);
	double normal_len = normal.norm();
	if (normal_len == 0)
	{
		normal = tVector(0, 0, 1, 0);
	}
	else
	{
		normal = (normal / normal_len);
	}

	const float pos_data[pos_len] =
	{
		a[0], a[1], a[2],
		b[0], b[1], b[2],
		c[0], c[1], c[2],
		d[0], d[1], d[2]
	};

	const float norm_data[norm_len] =
	{
		normal[0], normal[1], normal[2],
		normal[0], normal[1], normal[2],
		normal[0], normal[1], normal[2],
		normal[0], normal[1], normal[2]
	};

	const float coord_data[coord_len] =
	{
		coord_a[0], coord_a[1],
		coord_b[0], coord_b[1],
		coord_c[0], coord_c[1],
		coord_d[0], coord_d[1]
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = cMeshUtil::eAttributeNormal;
	attr_info.mAttribSize = sizeof(norm_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gNormDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * norm_len, (GLubyte*)norm_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = cMeshUtil::eAttributeCoord;
	attr_info.mAttribSize = sizeof(norm_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gCoordDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * coord_len, (GLubyte*)coord_data, 0, 1, &attr_info);

	gQuadMesh->Draw(gl_mode);
}

void cDrawUtil::DrawDisk(const tVector& pos, double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(pos);
	DrawDisk(r, draw_mode);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawDisk(double r, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_STRIP;
	cDrawUtil::PushMatrix();
	cDrawUtil::Scale(tVector(r, r, r, 1));
	gDiskMesh->Draw(gl_mode);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawPoint(const tVector& pt)
{
	const int num_verts = 1;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const float pos_data[pos_len] =
	{
		pt[0], pt[1], pt[2]
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gPointMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gPointMesh->Draw(GL_POINTS);
}

void cDrawUtil::DrawLine(const tVector& a, const tVector& b)
{
	const int num_verts = 2;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const float pos_data[pos_len] =
	{
		a[0], a[1], a[2],
		b[0], b[1], b[2],
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gLineMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gLineMesh->Draw(GL_LINES);
}

void cDrawUtil::DrawSphere(double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrix();
	cDrawUtil::Scale(tVector(r, r, r, 1));
	GLenum gl_mode = (draw_mode == eDrawWire) ? GL_LINES : GL_TRIANGLES;
	gSphereMesh->Draw(gl_mode);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawCylinder(double h, double r, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawWire) ? GL_LINES : GL_TRIANGLES;
	const int slices = gNumSlice;
	const int num_verts = 12 * slices;
	const int pos_size = cMeshUtil::gPosDim;
	const int norm_size = cMeshUtil::gNormDim;
	const int coord_size = cMeshUtil::gCoordDim;
	const int pos_len = num_verts * pos_size;

	float pos_data[pos_len];
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);

		double x1 = r * std::cos(theta1);
		double z1 = r * std::sin(-theta1);

		tVector n0 = tVector(x0, 0, z0, 0).normalized();
		tVector n1 = tVector(x1, 0, z1, 0).normalized();

		int pos_offset = i * 12 * pos_size;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5 * h;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = -0.5 * h;
		pos_data[pos_offset + 5] = z1;
		pos_data[pos_offset + 6] = x1;
		pos_data[pos_offset + 7] = 0.5 * h;
		pos_data[pos_offset + 8] = z1;
		pos_data[pos_offset + 9] = x1;
		pos_data[pos_offset + 10] = 0.5 * h;
		pos_data[pos_offset + 11] = z1;
		pos_data[pos_offset + 12] = x0;
		pos_data[pos_offset + 13] = 0.5 * h;
		pos_data[pos_offset + 14] = z0;
		pos_data[pos_offset + 15] = x0;
		pos_data[pos_offset + 16] = -0.5 * h;
		pos_data[pos_offset + 17] = z0;

		pos_data[pos_offset + 18] = x0;
		pos_data[pos_offset + 19] = 0.5 * h;
		pos_data[pos_offset + 20] = z0;
		pos_data[pos_offset + 21] = x1;
		pos_data[pos_offset + 22] = 0.5 * h;
		pos_data[pos_offset + 23] = z1;
		pos_data[pos_offset + 24] = 0;
		pos_data[pos_offset + 25] = 0.5 * h;
		pos_data[pos_offset + 26] = 0;
		pos_data[pos_offset + 27] = 0;
		pos_data[pos_offset + 28] = -0.5 * h;
		pos_data[pos_offset + 29] = 0;
		pos_data[pos_offset + 30] = x1;
		pos_data[pos_offset + 31] = -0.5 * h;
		pos_data[pos_offset + 32] = z1;
		pos_data[pos_offset + 33] = x0;
		pos_data[pos_offset + 34] = -0.5 * h;
		pos_data[pos_offset + 35] = z0;
	}

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gCylinderMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gCylinderMesh->Draw(gl_mode);
}

void cDrawUtil::DrawPlane(const tVector& coeffs, double size, eDrawMode draw_mode)
{
	const Eigen::Vector3d ref = Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d n = Eigen::Vector3d(coeffs[0], coeffs[1], coeffs[2]);
	double c = coeffs[3];

	Eigen::Vector3d axis = ref.cross(n);
	double axis_len = axis.norm();
	double theta = 0;
	if (axis_len != 0)
	{
		axis /= axis_len;
		theta = std::acos(ref.dot(n));
	}

	Eigen::Vector3d offset = c * n;

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(tVector(offset[0], offset[1], offset[2], 0));
	if (theta != 0)
	{
		cDrawUtil::Rotate(theta, tVector(axis[0], axis[1], axis[2], 0));
	}
	DrawRect(tVector::Zero(), tVector(size, size, 0, 0), draw_mode);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawCapsule(double h, double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrix();
	DrawCylinder(h, r, draw_mode);

	cDrawUtil::Translate(tVector(0, h * 0.5, 0, 0));
	DrawSphere(r, draw_mode);
	cDrawUtil::Translate(tVector(0, -h, 0, 0));
	DrawSphere(r, draw_mode);

	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawArrow2D(const tVector& start, const tVector& end, double head_size)
{
	GLboolean prev_enable;
	glGetBooleanv(GL_CULL_FACE, &prev_enable);
	glDisable(GL_CULL_FACE);

	tVector dir = tVector(0, 1, 0, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}

	dir[3] = 0;
	tVector axis = tVector(0, 0, 1, 0);
	tVector tangent = axis.cross3(dir);
	tangent.normalize();

	const double width = head_size * 0.1854;
	tVector body_end = end - dir * head_size;

	tVector a = start - width * tangent;
	tVector b = body_end - width * tangent;
	tVector c = body_end + width * tangent;
	tVector d = start + width * tangent;
	DrawQuad(a, b, c, d);

	tVector e0 = body_end - tangent * head_size * 0.5f;
	tVector e1 = body_end + tangent * head_size * 0.5f;
	DrawQuad(end, e1, e0, end);

	if (prev_enable)
	{
		glEnable(GL_CULL_FACE);
	}
}

void cDrawUtil::ClearColor(const tVector& col)
{
	glClearColor(static_cast<float>(col[0]), static_cast<float>(col[1]),
		static_cast<float>(col[2]), static_cast<float>(col[3]));
	glClear(GL_COLOR_BUFFER_BIT);
}

void cDrawUtil::ClearDepth(double depth)
{
	glClearDepth(depth);
	glClear(GL_DEPTH_BUFFER_BIT);
}

void cDrawUtil::MatrixMode(eMatrixMode mode)
{
	mMatrixMode = mode;
}

void cDrawUtil::LoadIdentityMatrix()
{
	SetMatrix(tMatrix::Identity());
}

void cDrawUtil::SetMatrix(const tMatrix& mat)
{
	auto& stack = GetCurrMatrixStack();
	tMatrix& top = stack.back();
	top = mat;
}

void cDrawUtil::MultMatrix(const tMatrix& mat)
{
	auto& stack = GetCurrMatrixStack();
	tMatrix& top = stack.back();
	top *= mat;
}

void cDrawUtil::PushMatrix()
{
	auto& stack = GetCurrMatrixStack();
	stack.push_back(stack.back());
}

void cDrawUtil::PopMatrix()
{
	auto& stack = GetCurrMatrixStack();
	stack.pop_back();
}

const tMatrix& cDrawUtil::GetProjMatrix()
{
	return mMatrixStackProj.back();
}

const tMatrix& cDrawUtil::GetModelViewMatrix()
{
	return mMatrixStackModelView.back();
}

void cDrawUtil::Finish()
{
	glFinish();
}

void cDrawUtil::BuildMeshes()
{
	cMeshUtil::BuildPointMesh(gPointMesh);
	cMeshUtil::BuildLineMesh(gLineMesh);
	cMeshUtil::BuildQuadMesh(gQuadMesh);
	cMeshUtil::BuildBoxMesh(gBoxMesh);
	cMeshUtil::BuildSphereMesh(gNumStacks, gNumSlice, gSphereMesh);
	cMeshUtil::BuildDiskMesh(gDiskSlices, gDiskMesh);
	cMeshUtil::BuildTriangleMesh(gTriangleMesh);
	cMeshUtil::BuildCylinder(gNumSlice, gCylinderMesh);
}

void cDrawUtil::BindShader(cShader* shader)
{
	gShader = shader;
	gShader->bind();
}

void cDrawUtil::Translate(const tVector& trans)
{
	MultMatrix(cMathUtil::TranslateMat(trans));
}

void cDrawUtil::Scale(const tVector& scale)
{
	MultMatrix(cMathUtil::ScaleMat(scale));
}

void cDrawUtil::Rotate(const tVector& euler)
{
	MultMatrix(cMathUtil::RotateMat(euler));
}

void cDrawUtil::Rotate(double theta, const tVector& axis)
{
	MultMatrix(cMathUtil::RotateMat(axis, theta));
}

void cDrawUtil::Rotate(const tQuaternion& q)
{
	MultMatrix(cMathUtil::RotateMat(q));
}

void cDrawUtil::SetColor(const tVector& col)
{
	gColor = col;
	if (gShader != nullptr)
	{
		gShader->setUniform("gColor", gColor);
	}
}

void cDrawUtil::SetLineWidth(double w)
{
	glLineWidth(static_cast<float>(w));
}

void cDrawUtil::SetPointSize(double pt_size)
{
	glPointSize(static_cast<float>(pt_size));
}

std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>>& cDrawUtil::GetCurrMatrixStack()
{
	if (mMatrixMode == eMatrixModeProj)
	{
		return mMatrixStackProj;
	}
	else
	{
		return mMatrixStackModelView;
	}
}

void cDrawUtil::SyncMatrices()
{
	if (gShader != nullptr)
	{
		Eigen::Matrix4f proj = GetProjMatrix().cast<float>();
		Eigen::Matrix4f model_view = GetModelViewMatrix().cast<float>();
		gShader->setUniform("gProjMatrix", proj);
		gShader->setUniform("gModelViewMatrix", model_view);
	}
}