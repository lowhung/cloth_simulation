#include "scenarios/ClothScenario.h"
#include <iostream>
#include <fstream>

const tVector gPoleColor = tVector(0.5, 0.5, 0.5, 1);

cClothScenario::tPole::tPole()
{
	mStart = tVector(-0.5, 1.5, 0, 0);
	mEnd = tVector(0.5, 1.5, 0, 0);
	mRadiusHorizontal = 0.025;
	mRadiusVertical = 0.05;
}

cClothScenario::cClothScenario()
{
	mWindDir = tVector(0, 0, 1, 0);
	mWindStrength = 5;
	mSimStepsize = 0.001;
	mFPS = 0;

	mClothParams.mTopPos0 = mPole.mStart;
	mClothParams.mTopPos1 = mPole.mEnd;
	mClothParams.mLength = 1;
}

cClothScenario::~cClothScenario()
{
	mShader.free();
}

void cClothScenario::Reset()
{
	cScenario::Reset();
	mSimCounter = 0;
	mFPS = 0;

	BuildCloth();
}

void cClothScenario::Init()
{
	cScenario::Init();
	mSimCounter = 0;

	LoadShaders();
	BuildCloth();
}

void cClothScenario::Update(double time_elapsed)
{
	cScenario::Update(time_elapsed);
	Simulate(time_elapsed);
}

void cClothScenario::SetWindStrength(double wind)
{
	mWindStrength = wind;
}

double cClothScenario::GetWindStrength() const
{
	return mWindStrength;
}

cCloth::eIntegrator cClothScenario::GetIntegrator() const
{
	return mCloth.GetIntegrator();
}

void cClothScenario::SetIntegrator(cCloth::eIntegrator integrator)
{
	mCloth.SetIntegrator(integrator);
}

double cClothScenario::GetStepsize() const
{
	return mSimStepsize;
}

void cClothScenario::SetStepsize(double stepsize)
{
	mSimStepsize = stepsize;
}

double cClothScenario::GetStiffness() const
{
	return mCloth.GetStiffness();
}

void cClothScenario::SetStiffness(double stiffness)
{
	mCloth.SetStiffness(stiffness);
	mClothParams.mStiffness = stiffness;
}

double cClothScenario::GetFPS() const
{
	return mFPS;
}

void cClothScenario::InitCamera()
{
	double h = 4;
	double w = (h * mWinSize.x()) / mWinSize.y();
	double near_z = 0.1;
	double far_z = 20;

	mCamera = cCamera(tVector(0, 2, 10, 0), tVector(0, 0.8, 0, 0), tVector(0, 1, 0, 0),
						w, h, near_z, far_z);
	mCamera.SetProj(cCamera::eProjPerspective);
}

void cClothScenario::LoadShaders()
{
	mShader.initFromFiles("a_simple_shader", "data/shaders/Mesh_VS.glsl", "data/shaders/Mesh_PS.glsl");
}

void cClothScenario::BuildCloth()
{
	mCloth.Init(mClothParams);

	// add pin constraints to hold the top of the cloth in place
	tVector res = mCloth.GetRes();
	for (int u = 0; u < res[0]; ++u)
	{
		int v = 0;
		int id = mCloth.GetVertID(u, v);
		cCloth::tPin pin;
		pin.mVert = id;
		pin.mPos = mCloth.GetVertPos(mCloth.GetVertID(u, v));
		mCloth.AddPin(pin);
	}
}

void cClothScenario::Simulate(double timestep)
{
	mSimCounter += timestep;
	int num_sim_steps = static_cast<int>(mSimCounter / mSimStepsize);
	double desired_fps = 1 / timestep;

	if (num_sim_steps > 0)
	{
		// log the time used to update simulation
		const double fps_lerp = 0.1;
		double start_time = glfwGetTime();

		for (int i = 0; i < num_sim_steps; ++i)
		{
			ApplyWind();
			mCloth.Update(mSimStepsize); // step simulation
		}
		mSimCounter -= num_sim_steps * mSimStepsize;

		// update FPS log
		double end_time = glfwGetTime();
		double fps = desired_fps * (num_sim_steps * mSimStepsize) / (end_time - start_time);
		mFPS = (1 - fps_lerp) * mFPS + fps_lerp * fps;
	}
}

void cClothScenario::ApplyWind()
{
	// apply wind force to each vertex
	tVector wind_force = mWindStrength * mWindDir;
	tVector res = mCloth.GetRes();

	for (int v = 0; v < res[1] - 1; ++v)
	{
		for (int u = 0; u < res[0] - 1; ++u)
		{
			int v0 = mCloth.GetVertID(u, v);
			int v1 = mCloth.GetVertID(u + 1, v);
			int v2 = mCloth.GetVertID(u + 1, v + 1);
			int v3 = mCloth.GetVertID(u, v + 1);

			tVector p0 = mCloth.GetVertPos(v0);
			tVector p1 = mCloth.GetVertPos(v1);
			tVector p2 = mCloth.GetVertPos(v2);
			tVector p3 = mCloth.GetVertPos(v3);

			tVector norm0 = (p3 - p0).cross3(p1 - p0);
			tVector norm1 = (p1 - p2).cross3(p3 - p2);
			double area0 = norm0.norm();
			double area1 = norm1.norm();
			norm0 /= area0;
			norm1 /= area1;

			tVector force0 = area0 * norm0 * (norm0.dot(wind_force));
			tVector force1 = area1 * norm1 * (norm1.dot(wind_force));

			mCloth.ApplyForce(v0, force0);
			mCloth.ApplyForce(v1, force0);
			mCloth.ApplyForce(v3, force0);

			mCloth.ApplyForce(v2, force1);
			mCloth.ApplyForce(v1, force1);
			mCloth.ApplyForce(v3, force1);
		}
	}
}

void cClothScenario::SetColor(const tVector& col)
{
	cDrawUtil::SetColor(col);
}

void cClothScenario::SetupDraw()
{
	cScenario::SetupDraw();
	SetupShader();
}


void cClothScenario::SetupShader()
{
	tVector light_dir = tVector(0.1, 1, 0.5, 0).normalized();
	const tVector light_col = tVector(0.5, 0.5, 0.5, 0);
	const tVector ambient_col = tVector(0.5, 0.5, 0.5, 0);

	mShader.Bind();

	tMatrix view_mat = mCamera.BuildWorldViewMatrix();
	light_dir = view_mat * light_dir;

	mShader.setUniform("gLightDir", Eigen::Vector3f(light_dir[0], light_dir[1], light_dir[2]));
	mShader.setUniform("gLightColour", Eigen::Vector3f(light_col[0], light_col[1], light_col[2]));
	mShader.setUniform("gAmbientColour", Eigen::Vector3f(ambient_col[0], ambient_col[1], ambient_col[2]));
}

void cClothScenario::DrawScene()
{
	cScenario::DrawScene();
	DrawGround();
	DrawPole();
	DrawCloth();
}

void cClothScenario::DrawGround()
{
	const double size = 100;
	SetColor(tVector(0.7, 0.7, 0.7, 1));
	cDrawUtil::DrawPlane(tVector(0, 1, 0, 0), size);
}

void cClothScenario::DrawPole()
{
	tVector mid = 0.5 * (mPole.mEnd + mPole.mStart);
	tVector pole_dir = (mPole.mEnd - mPole.mStart).normalized();
	double w = (mPole.mEnd - mPole.mStart).norm();
	double rh = mPole.mRadiusHorizontal;

	tQuaternion rot = cMathUtil::VecDiffQuat(tVector(1, 0, 0, 0), pole_dir);

	cDrawUtil::SetColor(gPoleColor);

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(mid);
	cDrawUtil::Rotate(rot);
	cDrawUtil::DrawBox(tVector::Zero(), tVector(w, 2 * rh, 2 * rh, 0));
	cDrawUtil::PopMatrix();
}

void cClothScenario::DrawCloth() const
{
	cDrawUtil::SetColor(tVector(0, 1, 0, 1));
	cDrawUtil::SetLineWidth(1);
	for (int e = 0; e < mCloth.GetNumEdges(); ++e)
	{
		tVector pos0;
		tVector pos1;
		mCloth.GetEdgePos(e, pos0, pos1);
		cDrawUtil::DrawLine(pos0, pos1);
	}

	cDrawUtil::SetColor(tVector(0, 0, 1, 1));
	cDrawUtil::SetPointSize(3);
	for (int v = 0; v < mCloth.GetNumVerts(); ++v)
	{
		tVector pos = mCloth.GetVertPos(v);
		cDrawUtil::DrawPoint(pos);
	}
}

tVector cClothScenario::GetClearColor() const
{
	return tVector(0.8, 0.8, 0.8, 0);
}