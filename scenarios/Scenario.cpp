#include "scenarios/Scenario.h"
#include "render/DrawUtil.h"

cScenario::cScenario()
{
	mTime = 0;
	mWinSize.setZero();
}

cScenario::~cScenario()
{
}

void cScenario::Init()
{
	mTime = 0;
	InitCamera();
}

void cScenario::Reset()
{
}

void cScenario::Clear()
{
}

void cScenario::LoadParams(const std::string& param_file)
{
}

void cScenario::Update(double time_elapsed)
{
	mTime += time_elapsed;
}

void cScenario::Draw()
{
	SetupDraw();
	DrawScene();
}

void cScenario::Resize(const Eigen::Vector2i& win_size)
{
	mWinSize = win_size;
	double h = mCamera.GetHeight();
	double w = (h * win_size[0]) / win_size[1];
	mCamera.Resize(w, h);
}


bool cScenario::MouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers)
{
	double x = static_cast<double>(p.x()) / mWinSize.x();
	double y = static_cast<double>(p.y()) / mWinSize.y();
	x = 2 * (x - 0.5);
	y = -2 * (y - 0.5);
	mCamera.MouseClick(button, down, modifiers, x, y);
	return false;
}

bool cScenario::MouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers)
{
	double x = static_cast<double>(p.x()) / mWinSize.x();
	double y = static_cast<double>(p.y()) / mWinSize.y();
	x = 2 * (x - 0.5);
	y = -2 * (y - 0.5);
	mCamera.MouseMove(x, y);
	return false;
}

bool cScenario::ScrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel)
{
	mCamera.MouseScroll(rel[0], rel[1]);
	return false;
}

double cScenario::GetTime() const
{
	return mTime;
}

void cScenario::SetTime(double time)
{
	mTime = time;
}

const std::vector<std::string>& cScenario::GetParamFiles() const
{
	return mParamFiles;
}

double cScenario::GetPlaybackProgress() const
{
	return 0;
}

void cScenario::SetPlaybackProgress(double val)
{
}


void cScenario::InitCamera()
{
	double h = 2;
	double w = (h * mWinSize.x()) / mWinSize.y();
	double near_z = 0.1;
	double far_z = 20;

	mCamera = cCamera(tVector(0, 0, 5, 0), tVector(0, 0, 0, 0), tVector(0, 1, 0, 0),
						w, h, near_z, far_z);
	mCamera.SetProj(cCamera::eProjPerspective);
}

void cScenario::SetupCamera()
{
	tMatrix proj = mCamera.BuildProjMatrix();
	cDrawUtil::MatrixMode(cDrawUtil::eMatrixModeProj);
	cDrawUtil::SetMatrix(proj);

	tMatrix world_view = mCamera.BuildWorldViewMatrix();
	cDrawUtil::MatrixMode(cDrawUtil::eMatrixModeModelView);
	cDrawUtil::SetMatrix(world_view);
}

void cScenario::SetupDraw()
{
	SetupCamera();
}

void cScenario::DrawScene()
{
	// set background color
	cDrawUtil::ClearColor(GetClearColor());
}

tVector cScenario::GetClearColor() const
{
	return tVector(0.25, 0.25, 0.25, 0);
}