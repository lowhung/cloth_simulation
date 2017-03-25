#pragma once

#include <memory>
#include "util/MathUtil.h"
#include "util/PluginAPI.h"
#include "render/DrawMesh.h"
#include "render/MeshUtil.h"

class cShader;

class PLUGIN_EXPORT cDrawUtil
{
public:
	enum eDrawMode
	{
		eDrawSolid,
		eDrawWire,
		eDrawMax
	};

	enum eMatrixMode
	{
		eMatrixModeProj,
		eMatrixModeModelView,
		eMatrixModeMax
	};

	static void InitDrawUtil();
	static void DrawRect(const tVector& pos, const tVector& size, eDrawMode draw_mode = eDrawSolid);
	static void DrawBox(const tVector& pos, const tVector& size, eDrawMode draw_mode = eDrawSolid);
	static void DrawTriangle(const tVector& pos, double side_len, eDrawMode draw_mode = eDrawSolid);
	static void DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, eDrawMode draw_mode = eDrawSolid);
	static void DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, 
						const tVector& coord_a, const tVector& coord_b, const tVector& coord_c, const tVector& coord_d,
						eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(const tVector& pos, double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawPoint(const tVector& pt);
	static void DrawLine(const tVector& a, const tVector& b);
	static void DrawLineStrip(const tVectorArr& pts);
	static void DrawStrip(const tVector& a, const tVector& b, double width, eDrawMode draw_mode = eDrawSolid);
	static void DrawPlane(const tVector& coeffs, double size, eDrawMode draw_mode = eDrawSolid);
	static void DrawSphere(double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawCylinder(double h, double r, eDrawMode draw_mode = eDrawSolid);
	static void DrawCapsule(double h, double r, eDrawMode draw_mode = eDrawSolid);

	static void DrawArrow2D(const tVector& start, const tVector& end, double head_size);

	static void ClearColor(const tVector& col);
	static void ClearDepth(double depth);

	static void Translate(const tVector& trans);
	static void Scale(const tVector& scale);
	static void Rotate(const tVector& euler);
	static void Rotate(double theta, const tVector& axis);
	static void Rotate(const tQuaternion& q);
	static void SetColor(const tVector& col);
	static void SetLineWidth(double w);
	static void SetPointSize(double pt_size);

	static void MatrixMode(eMatrixMode mode);
	static void LoadIdentityMatrix();
	static void SetMatrix(const tMatrix& mat);
	static void MultMatrix(const tMatrix& mat);
	static void PushMatrix();
	static void PopMatrix();
	static const tMatrix& GetProjMatrix();
	static const tMatrix& GetModelViewMatrix();

	static void Finish();
	
	static void BuildMeshes();
	static void BindShader(cShader* shader);

	static void SyncMatrices();

protected:
	static tVector gColor;
	static cShader* gShader;

	static std::unique_ptr<cDrawMesh> gPointMesh;
	static std::unique_ptr<cDrawMesh> gLineMesh;
	static std::unique_ptr<cDrawMesh> gQuadMesh;
	static std::unique_ptr<cDrawMesh> gBoxMesh;
	static std::unique_ptr<cDrawMesh> gSphereMesh;
	static std::unique_ptr<cDrawMesh> gDiskMesh;
	static std::unique_ptr<cDrawMesh> gTriangleMesh;
	static std::unique_ptr<cDrawMesh> gCylinderMesh;

	static eMatrixMode mMatrixMode;
	static std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>> mMatrixStackProj;
	static std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>> mMatrixStackModelView;
	static std::vector<tMatrix, Eigen::aligned_allocator<tMatrix>>& GetCurrMatrixStack();
};
