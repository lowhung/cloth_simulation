#include "Cloth.h"
#include <iostream>
#include <Eigen/SparseLU>

const int cCloth::gVertDim = 3;

cCloth::tParams::tParams()
{
	mTopPos0 = tVector(0, 1, 0, 0);
	mTopPos1 = tVector(1, 1, 0, 0);
	mLength = 1;
	mRes = tVector(10, 10, 0, 0);

	mMass = 0.1; // mass of each vertex (kg)
	mGravity = tVector(0, -9.8, 0, 0);

	mStiffness = 10;
	mDamping = 1;
	mAirFriction = 0.05;
}

cCloth::tPin::tPin()
{
	mVert = gInvalidIdx;
	mPos.setZero();
}

cCloth::tEdge::tEdge()
{
	mVert0 = gInvalidIdx;
	mVert1 = gInvalidIdx;
	mRestLen = 0;
}

cCloth::cCloth()
{
	mIntegrator = eIntegratorExplicit;
}

cCloth::~cCloth()
{
}

void cCloth::Init(const tParams& params)
{
	ClearPins();
	mParams = params;
	
	InitBuffers();
	BuildVerts();
	BuildEdges();
	InitJacobian();
}

void cCloth::Update(double timestep)
{
	// step simulation with chosen integrator
	switch (mIntegrator)
	{
	case eIntegratorExplicit:
		IntegrateForward(timestep, mX, mV, mX, mV);
		break;
	case eIntegratorMidpoint:
		IntegrateMidpoint(timestep, mX, mV, mX, mV);
		break;
	case eIntegratorTrapezoid:
		IntegrateTrapezoid(timestep, mX, mV, mX, mV);
		break;
	case eIntegratorImplicit:
		IntegrateImplicit(timestep, mX, mV, mX, mV);
		break;
	default:
		printf("Unsupported integrator\n");
		assert(false);
		break;
	}
	
	ApplyConstraints(mX, mV);
	ClearForces();
}

int cCloth::GetNumVerts() const
{
	return mParams.mRes[0] * mParams.mRes[1];
}

const tVector& cCloth::GetRes() const
{
	return mParams.mRes;
}

tVector cCloth::GetVertPos(int id) const
{
	return GetVertData(id, mX);
}

void cCloth::SetVertPos(int id, const tVector& pos)
{
	SetVertData(id, pos, mX);
}

tVector cCloth::GetVertVel(int id) const
{
	return GetVertData(id, mV);
}

void cCloth::SetVertVel(int id, const tVector& vel)
{
	SetVertData(id, vel, mV);
}

int cCloth::AddPin(const tPin& pin)
{
	int id = static_cast<int>(mPins.size());
	mPins.push_back(pin);
	mIsPinned[pin.mVert] = 1;
	return id;
}

void cCloth::ClearPins()
{
	mPins.clear();
}

int cCloth::GetNumPins() const
{
	return static_cast<int>(mPins.size());
}

int cCloth::GetVertID(int u, int v) const
{
	assert(u >= 0 && u < mParams.mRes[0]);
	assert(v >= 0 && v < mParams.mRes[1]);
	return v * mParams.mRes[0] + u;
}

int cCloth::GetNumEdges() const
{
	return static_cast<int>(mEdges.size());
}

void cCloth::GetEdgePos(int e, tVector& out_pos0, tVector& out_pos1) const
{
	// start and end points of an edge
	const tEdge& edge = mEdges[e];
	out_pos0 = GetVertPos(edge.mVert0);
	out_pos1 = GetVertPos(edge.mVert1);
}

cCloth::eIntegrator cCloth::GetIntegrator() const
{
	return mIntegrator;
}

void cCloth::SetIntegrator(eIntegrator integrator)
{
	mIntegrator = integrator;
}

double cCloth::GetStiffness() const
{
	return mParams.mStiffness;
}

void cCloth::SetStiffness(double stiffness)
{
	mParams.mStiffness = stiffness;
}

void cCloth::ApplyForce(int id, const tVector& force)
{
	AddVertData(id, force, mExternalForces);
}

void cCloth::InitBuffers()
{
	int num_verts = GetNumVerts();

	// vectors used to stores the state of the vertices (position + velocity)
	mX = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mV = Eigen::VectorXd::Zero(num_verts * gVertDim);

	// allocate a bunch of temporary memory buffer used for calculations later
	mTempX = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mTempV = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mdX = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mdV = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mdX1 = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mdV1 = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mExternalForces = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mForceBuffer = Eigen::VectorXd::Zero(num_verts * gVertDim);
	mY = Eigen::VectorXd(2 * num_verts * gVertDim);
}

void cCloth::InitJacobian()
{
	// Initialize the sparse Jacobian matrix
	std::vector<Eigen::Triplet<double>> J_elem;

	int num_verts = GetNumVerts();
	int X_dim = num_verts * gVertDim;
	int V_dim = X_dim;
	const double placeholder = 1;

	for (int i = 0; i < V_dim; ++i)
	{
		J_elem.push_back(Eigen::Triplet<double>(i, X_dim + i, placeholder));
	}

	for (int v = 0; v < 2 * num_verts; ++v)
	{
		for (int j = 0; j < gVertDim; ++j)
		{
			for (int i = 0; i < gVertDim; ++i)
			{
				J_elem.push_back(Eigen::Triplet<double>(
					v * gVertDim + i,
					v * gVertDim + j,
					placeholder));
			}
		}
	}
	
	for (int e = 0; e < GetNumEdges(); ++e)
	{
		const tEdge& edge = mEdges[e];

		for (int j = 0; j < gVertDim; ++j)
		{
			for (int i = 0; i < gVertDim; ++i)
			{
				J_elem.push_back(Eigen::Triplet<double>(
					X_dim + edge.mVert0 * gVertDim + i,
					edge.mVert1 * gVertDim + j,
					placeholder));
				J_elem.push_back(Eigen::Triplet<double>(
					X_dim + edge.mVert1 * gVertDim + i,
					edge.mVert0 * gVertDim + j,
					placeholder));
			}
		}

		for (int j = 0; j < gVertDim; ++j)
		{
			for (int i = 0; i < gVertDim; ++i)
			{
				J_elem.push_back(Eigen::Triplet<double>(
					X_dim + edge.mVert0 * gVertDim + i,
					X_dim + edge.mVert1 * gVertDim + j,
					placeholder));
				J_elem.push_back(Eigen::Triplet<double>(
					X_dim + edge.mVert1 * gVertDim + i,
					X_dim + edge.mVert0 * gVertDim + j,
					placeholder));
			}
		}
	}

	mJ.resize(X_dim + V_dim, X_dim + V_dim);
	mJ.setFromTriplets(J_elem.begin(), J_elem.end());
	
	// initialize solver
	mSolver.analyzePattern(mJ);   // for this step the numerical values of A are not used
}

void cCloth::BuildVerts()
{
	// intialize vertex positions and velocities
	int res_u = mParams.mRes[0];
	int res_v = mParams.mRes[1];
	for (int v = 0; v < res_v; ++v)
	{
		for (int u = 0; u < res_u; ++u)
		{
			tVector pos = tVector::Zero();

			double u_lerp = u / (res_u - 1.0);
			double v_lerp = v / (res_v - 1.0);
			pos = (1 - u_lerp) * mParams.mTopPos0 + u_lerp * mParams.mTopPos1;
			pos[1] = pos[1] - v_lerp * mParams.mLength;

			SetVertData(GetVertID(u, v), pos, mX);
		}
	}

	mIsPinned = Eigen::VectorXi::Zero(GetNumVerts());
}

void cCloth::BuildEdges()
{
	// build edges that connect the vertices
	int res_u = mParams.mRes[0];
	int res_v = mParams.mRes[1];
	int num_edges = res_u * (res_v - 1) + (res_u - 1) * res_u + 2 * (res_u - 1) * (res_v - 1);
	mEdges.clear();
	mEdges.reserve(num_edges);

	// structural springs
	for (int v = 0; v < res_v; ++v)
	{
		for (int u = 0; u < res_u - 1; ++u)
		{
			tVector p0 = GetVertPos(GetVertID(u, v));
			tVector p1 = GetVertPos(GetVertID(u + 1, v));
			tEdge e;
			e.mVert0 = GetVertID(u, v);
			e.mVert1 = GetVertID(u + 1, v);
			e.mRestLen = (p1 - p0).norm();
			AddEdge(e);
		}
	}

	for (int v = 0; v < res_v - 1; ++v)
	{
		for (int u = 0; u < res_u; ++u)
		{
			tVector p0 = GetVertPos(GetVertID(u, v));
			tVector p1 = GetVertPos(GetVertID(u, v + 1));
			tEdge e;
			e.mVert0 = GetVertID(u, v);
			e.mVert1 = GetVertID(u, v + 1);
			e.mRestLen = (p1 - p0).norm();
			AddEdge(e);
		}
	}

	// shear springs
	for (int v = 0; v < res_v - 1; ++v)
	{
		for (int u = 0; u < res_u - 1; ++u)
		{
			tVector p0 = GetVertPos(GetVertID(u, v));
			tVector p1 = GetVertPos(GetVertID(u + 1, v + 1));
			tEdge e0;
			e0.mVert0 = GetVertID(u, v);
			e0.mVert1 = GetVertID(u + 1, v + 1);
			e0.mRestLen = (p1 - p0).norm();
			AddEdge(e0);

			tVector p2 = GetVertPos(GetVertID(u + 1, v));
			tVector p3 = GetVertPos(GetVertID(u, v + 1));
			tEdge e1;
			e1.mVert0 = GetVertID(u + 1, v);
			e1.mVert1 = GetVertID(u, v + 1);
			e1.mRestLen = (p2 - p3).norm();
			AddEdge(e1);
		}
	}
}

void cCloth::AddEdge(const tEdge& edge)
{
	mEdges.push_back(edge);
}

double cCloth::GetVertInvMass(int id) const
{
	bool is_pined = IsVertPinned(id);
	double m = mParams.mMass;
	double inv_mass = (is_pined) ? 0 : (1 / m);
	return inv_mass;
}

bool cCloth::IsVertPinned(int id) const
{
	return mIsPinned[id] != 0;
}

void cCloth::ClearForces()
{
	mExternalForces.setZero();
}

void cCloth::ApplyGravity(Eigen::VectorXd& out_F) const
{
	for (int i = 0; i < GetNumVerts(); ++i)
	{
		double inv_m = GetVertInvMass(i);
		tVector f = tVector::Zero();
		if (inv_m != 0)
		{
			f = mParams.mGravity / inv_m;
		}
		AddVertData(i, f, out_F);
	}
}

void cCloth::IntegrateForward(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
								Eigen::VectorXd& out_X, Eigen::VectorXd& out_V)
{
	// TODO (CPSC426): Implement forward euler to update the particle positions X and Velocites V
	// updated state should be stored in out_X and out_V
	Eigen::VectorXd out_dX, out_dV;
		EvalDerivative(X, V, out_dX, out_dV);
		out_X = X + timestep*out_dX;
		out_V = V + timestep*out_dV;
}

void cCloth::IntegrateMidpoint(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
							Eigen::VectorXd& out_X, Eigen::VectorXd& out_V)
{
	// TODO (CPSC426): Implement midpoint method
}

void cCloth::IntegrateTrapezoid(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
								Eigen::VectorXd& out_X, Eigen::VectorXd& out_V)
{
	// TODO (CPSC426): Implement trapezoid method
}

void cCloth::IntegrateImplicit(double timestep, const Eigen::VectorXd& X, const Eigen::VectorXd& V,
								Eigen::VectorXd& out_X, Eigen::VectorXd& out_V)
{
	// TODO (CPSC426): Implement implicit euler
	BuildJacobian(X, V, mJ); // build Jacobian and store it in mJ
}

void cCloth::EvalDerivative(const Eigen::VectorXd& X, const Eigen::VectorXd& V, Eigen::VectorXd& out_dX, Eigen::VectorXd& out_dV)
{
	// Evaluates the derivatives f(X, V) for the given particle positions X and velocities V
	mForceBuffer = mExternalForces;

	ApplyGravity(mForceBuffer);
	ApplyAirFriction(V, mForceBuffer);
	ApplyInternalForces(X, V, mForceBuffer);

	out_dX = V;
	out_dV = mForceBuffer;

	for (int i = 0; i < GetNumVerts(); ++i)
	{
		double inv_mass = GetVertInvMass(i);
		out_dV.segment(i * gVertDim, gVertDim) *= inv_mass;
	}
}

void cCloth::BuildJacobian(const Eigen::VectorXd& X, const Eigen::VectorXd& V, 
							Eigen::SparseMatrix<double>& out_J) const
{
	// Builds the Jacobian J(X, V) of the derivative function f(X, V)
	int X_dim = X.size();
	int V_dim = V.size();
	double k = mParams.mStiffness;
	double kd = mParams.mDamping;

	// clear all entries to 0
	ClearSparseMat(0, out_J);

	for (int i = 0; i < V_dim; ++i)
	{
		out_J.coeffRef(i, X_dim + i) = 1;
	}

	for (int i = 0; i < GetNumVerts(); ++i)
	{
		double inv_mass = GetVertInvMass(i);
		double J_friction = -mParams.mAirFriction * inv_mass;

		for (int j = 0; j < gVertDim; ++j)
		{
			int idx = X_dim + i * gVertDim + j;
			out_J.coeffRef(idx, idx) = J_friction;
		}
	}

	for (int e = 0; e < GetNumEdges(); ++e)
	{
		const tEdge& edge = mEdges[e];
		Eigen::Vector3d p0 = GetVertData(edge.mVert0, X).segment(0, 3);
		Eigen::Vector3d p1 = GetVertData(edge.mVert1, X).segment(0, 3);
		Eigen::Vector3d delta = p1 - p0;
		double len2 = delta.squaredNorm();
		double len = std::sqrt(len2);

		double inv_m0 = GetVertInvMass(edge.mVert0);
		double inv_m1 = GetVertInvMass(edge.mVert1);
		Eigen::Matrix3d curr_Jvx = Eigen::Matrix3d::Identity() * (1 / edge.mRestLen - 1 / len);
		curr_Jvx += ((1 / (len2 * len)) * delta) * delta.transpose();

		for (int j = 0; j < gVertDim; ++j)
		{
			for (int i = 0; i < gVertDim; ++i)
			{
				out_J.coeffRef(X_dim + edge.mVert0 * gVertDim + i, edge.mVert0 * gVertDim + j) += (-k * inv_m0) * curr_Jvx(i, j);
				out_J.coeffRef(X_dim + edge.mVert0 * gVertDim + i, edge.mVert1 * gVertDim + j) += (k * inv_m0) * curr_Jvx(i, j);
				out_J.coeffRef(X_dim + edge.mVert1 * gVertDim + i, edge.mVert1 * gVertDim + j) += (-k * inv_m1) * curr_Jvx(i, j);
				out_J.coeffRef(X_dim + edge.mVert1 * gVertDim + i, edge.mVert0 * gVertDim + j) += (k * inv_m1) * curr_Jvx(i, j);
			}
		}
		
		Eigen::Matrix3d curr_Jvv = delta * delta.transpose();
		curr_Jvv *= kd / (edge.mRestLen * len2);

		for (int j = 0; j < gVertDim; ++j)
		{
			for (int i = 0; i < gVertDim; ++i)
			{
				out_J.coeffRef(X_dim + edge.mVert0 * gVertDim + i, X_dim + edge.mVert0 * gVertDim + j) += -inv_m0 * curr_Jvv(i, j);
				out_J.coeffRef(X_dim + edge.mVert0 * gVertDim + i, X_dim + edge.mVert1 * gVertDim + j) += inv_m0 * curr_Jvv(i, j);
				out_J.coeffRef(X_dim + edge.mVert1 * gVertDim + i, X_dim + edge.mVert1 * gVertDim + j) += -inv_m1 * curr_Jvv(i, j);
				out_J.coeffRef(X_dim + edge.mVert1 * gVertDim + i, X_dim + edge.mVert0 * gVertDim + j) += inv_m1 * curr_Jvv(i, j);
			}
		}
	}
}

void cCloth::ClearSparseMat(double val, Eigen::SparseMatrix<double>& out_J) const
{
	for (int k = 0; k < out_J.outerSize(); ++k)
	{
		for (Eigen::SparseMatrix<double>::InnerIterator it(out_J, k); it; ++it)
		{
			it.valueRef() = 0;
		}
	}
}

void cCloth::ApplyAirFriction(const Eigen::VectorXd& V, Eigen::VectorXd& out_F) const
{
	for (int i = 0; i < GetNumVerts(); ++i)
	{
		tVector vel = GetVertData(i, V);
		tVector f = -mParams.mAirFriction * vel;
		AddVertData(i, f, out_F);
	}
}

void cCloth::ApplyInternalForces(const Eigen::VectorXd& X, const Eigen::VectorXd& V, Eigen::VectorXd& out_F) const
{
	int num_edges = GetNumEdges();
	for (int e = 0; e < num_edges; ++e)
	{
		const tEdge& edge = mEdges[e];
		tVector p0 = GetVertData(edge.mVert0, X);
		tVector p1 = GetVertData(edge.mVert1, X);
		tVector v0 = GetVertData(edge.mVert0, V);
		tVector v1 = GetVertData(edge.mVert1, V);

		tVector delta = p1 - p0;
		double len = delta.norm();
		tVector dir = delta / len;
		double strain = len / edge.mRestLen - 1;

		double f = mParams.mStiffness * strain;
		tVector force = f * dir;

		double v = (v1 - v0).dot(dir);
		tVector damping = mParams.mDamping / edge.mRestLen * v * dir;
		force += damping;

		AddVertData(edge.mVert0, force, out_F);
		AddVertData(edge.mVert1, -force, out_F);
	}
}

void cCloth::ApplyConstraints(Eigen::VectorXd& out_X, Eigen::VectorXd& out_V) const
{
	int num_pins = GetNumPins();
	for (int p = 0; p < num_pins; ++p)
	{
		const tPin& pin = mPins[p];
		SetVertData(pin.mVert, pin.mPos, out_X);
		SetVertData(pin.mVert, tVector::Zero() , out_V);
	}
}

tVector cCloth::GetVertData(int idx, const Eigen::VectorXd& data) const
{
	tVector x = tVector::Zero();
	x.segment(0, gVertDim) = data.segment(idx * gVertDim, gVertDim);
	return x;
}

void cCloth::SetVertData(int idx, const tVector& x, Eigen::VectorXd& out_data) const
{
	out_data.segment(idx * gVertDim, gVertDim) = x.segment(0, gVertDim);
}

void cCloth::AddVertData(int idx, const tVector& x, Eigen::VectorXd& out_data) const
{
	out_data.segment(idx * gVertDim, gVertDim) += x.segment(0, gVertDim);
}