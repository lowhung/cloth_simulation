#pragma once

#include <nanogui/glutil.h>
#include "scenarios/Scenario.h"
#include "Cloth.h"
#include "render/Shader.h"

// animates a biped using keyframes

class PLUGIN_EXPORT cClothScenario : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cClothScenario();
	virtual ~cClothScenario();

	virtual void Reset();
	virtual void Init();
	virtual void Update(double time_elapsed);

	virtual void SetWindStrength(double wind);
	virtual double GetWindStrength() const;

	virtual cCloth::eIntegrator GetIntegrator() const;
	virtual void SetIntegrator(cCloth::eIntegrator integrator);
	virtual double GetStepsize() const;
	virtual void SetStepsize(double stepsize);
	virtual double GetStiffness() const;
	virtual void SetStiffness(double stiffness);

	virtual double GetFPS() const;

protected:
	
	struct tPole
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tVector mStart;
		tVector mEnd;
		double mRadiusHorizontal;
		double mRadiusVertical;

		tPole();
	};

	cShader mShader;
	tPole mPole;
	cCloth::tParams mClothParams;
	cCloth mCloth;
	double mSimCounter;
	double mSimStepsize; // stepsize used for simulation, not the same as the framerate

	tVector mWindDir;
	double mWindStrength;
	double mFPS;

	virtual void InitCamera();
	virtual void LoadShaders();
	virtual void BuildCloth();

	virtual void Simulate(double timestep);
	virtual void ApplyWind();

	virtual void SetColor(const tVector& col);

	virtual void SetupDraw();
	virtual void SetupShader();

	virtual void DrawScene();
	virtual void DrawGround();
	virtual void DrawPole();
	virtual void DrawCloth() const;

	virtual tVector GetClearColor() const;
};
