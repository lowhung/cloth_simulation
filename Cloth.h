#pragma once
#include "util/MathUtil.h"
#include <Eigen/Sparse>

class cCloth
{
public:
	
	enum eIntegrator
	{
		eIntegratorExplicit,
		eIntegratorMidpoint,
		eIntegratorTrapezoid,
		eIntegratorImplicit,
		eIntegratorMax
	};

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		tVector mTopPos0;
		tVector mTopPos1;
		double mLength;
		tVector mRes;

		double mMass;
		tVector mGravity;

		double mStiffness;
		double mDamping;
		double mAirFriction;

		tParams();
	};

	struct tPin
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		int mVert;
		tVector mPos;

		tPin();
	};

	cCloth();
	virtual ~cCloth();

	virtual void Init(const tParams& params);
	virtual void Update(double timestep);

	virtual int GetNumVerts() const;
	virtual const tVector& GetRes() const;
	virtual tVector GetVertPos(int id) const;
	virtual void SetVertPos(int id, const tVector& pos);
	virtual tVector GetVertVel(int id) const;
	virtual void SetVertVel(int id, const tVector& vel);

	virtual int AddPin(const tPin& pin);
	virtual void ClearPins();
	virtual int GetNumPins() const;

	virtual int GetVertID(int u, int v) const;
	virtual int GetNumEdges() const;
	virtual void GetEdgePos(int e, tVector& out_pos0, tVector& out_pos1) const;
	virtual eIntegrator GetIntegrator() const;
	virtual void SetIntegrator(eIntegrator integrator);
	virtual double GetStiffness() const;
	virtual void SetStiffness(double stiffness);

	virtual void ApplyForce(int id, const tVector& force);

protected:
	
	struct tEdge
	{
		int mVert0;
		int mVert1;
		double mRestLen;
		
		tEdge();
	};

	static const int gPosDim;
	static const int gVelDim;
	static const int gVertDim;

	tParams mParams;
	Eigen::VectorXd mX;
	Eigen::VectorXd mV;
	Eigen::VectorXd mForceBuffer;

	// temporary memory buffers used for intermediate computation
	Eigen::VectorXd mTempX;
	Eigen::VectorXd mTempV;
	Eigen::VectorXd mdX;
	Eigen::VectorXd mdV;
	Eigen::VectorXd mdX1;
	Eigen::VectorXd mdV1;
	Eigen::VectorXd mExternalForces;
	Eigen::VectorXd mY;

	Eigen::SparseMatrix<double> mJ;
	Eigen::SparseLU<Eigen::SparseMatrix<double>> mSolver;
	Eigen::VectorXi mIsPinned;

	std::vector<tEdge, Eigen::aligned_allocator<tEdge>> mEdges;
	std::vector<tPin, Eigen::aligned_allocator<tPin>> mPins;

	eIntegrator mIntegrator;

	virtual void InitBuffers();
	virtual void InitJacobian();
	virtual void BuildVerts();
	virtual void BuildEdges();
	virtual void AddEdge(const tEdge& edge);

	virtual double GetVertInvMass(int id) const;
	virtual bool IsVertPinned(int id) const;
	virtual void ClearForces();

	virtual void IntegrateForward(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
										Eigen::VectorXd& out_X, Eigen::VectorXd& out_V);
	virtual void IntegrateMidpoint(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
										Eigen::VectorXd& out_X, Eigen::VectorXd& out_V);
	virtual void IntegrateTrapezoid(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
										Eigen::VectorXd& out_X, Eigen::VectorXd& out_V);
	virtual void IntegrateImplicit(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
										Eigen::VectorXd& out_X, Eigen::VectorXd& out_V);
	virtual void EvalDerivative(const Eigen::VectorXd& X, const Eigen::VectorXd& V, Eigen::VectorXd& out_dX, Eigen::VectorXd& out_dV);
	virtual void BuildJacobian(const Eigen::VectorXd& X, const Eigen::VectorXd& V, Eigen::SparseMatrix<double>& out_J) const;
	virtual void ClearSparseMat(double val, Eigen::SparseMatrix<double>& out_J) const;

	virtual void ApplyGravity(Eigen::VectorXd& out_F) const;
	virtual void ApplyAirFriction(const Eigen::VectorXd& V, Eigen::VectorXd& out_F) const;
	virtual void ApplyInternalForces(const Eigen::VectorXd& X, const Eigen::VectorXd& V, Eigen::VectorXd& out_F) const;
	virtual void ApplyConstraints(Eigen::VectorXd& out_X, Eigen::VectorXd& out_V) const;
	
	virtual tVector GetVertData(int idx, const Eigen::VectorXd& data) const;
	virtual void SetVertData(int idx, const tVector& x, Eigen::VectorXd& out_data) const;
	virtual void AddVertData(int idx, const tVector& x, Eigen::VectorXd& out_data) const;
};
