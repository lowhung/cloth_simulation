#pragma once

#include "Eigen/Dense"

#include "render/Camera.h"

class PLUGIN_EXPORT cScenario
{
public:
	
	virtual ~cScenario();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void LoadParams(const std::string& param_file);

	virtual void Update(double time_elapsed);
	virtual void Draw();
	
	virtual void Resize(const Eigen::Vector2i& win_size);
	virtual bool MouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers);
	virtual bool MouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers);
	virtual bool ScrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel);

	virtual double GetTime() const;
	virtual void SetTime(double time);

	virtual const std::vector<std::string>& GetParamFiles() const;
	virtual double GetPlaybackProgress() const;
	virtual void SetPlaybackProgress(double val);

protected:
	double mTime;
	Eigen::Vector2i mWinSize;
	std::vector<std::string> mParamFiles;

	cCamera mCamera;

	cScenario();

	virtual void InitCamera();
	virtual void SetupCamera();

	virtual void SetupDraw();
	virtual void DrawScene();

	virtual tVector GetClearColor() const;
};
