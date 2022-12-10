#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

DiffusionSimulator::DiffusionSimulator() 
	: T(_gridSizeX, _gridSizeY)
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
}

DiffusionSimulator::~DiffusionSimulator() { }

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	this->InitializeTemperatureGrid();
}


void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented

	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &_alpha, "min=0.0,max=1.0");

	TwAddVarRW(DUC->g_pTweakBar, "Size x", TW_TYPE_UINT32, &_gridSizeX, "");
	TwAddVarRW(DUC->g_pTweakBar, "Size y", TW_TYPE_UINT32, &_gridSizeY, "");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	reset();

	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::ProcessInput() {
	if (_gridSizeX != T.GetSizeX() || _gridSizeY != T.GetSizeY()) {
		T = Grid<double>(_gridSizeX, _gridSizeY);
		this->InitializeTemperatureGrid();
	}
}

void DiffusionSimulator::InitializeTemperatureGrid() {
	uint32_t sizeX = T.GetSizeX();
	uint32_t sizeY = T.GetSizeY();

	for (uint32_t y = 0; y < sizeY; y++) 
	{
		for (uint32_t x = 0; x < sizeX; x++) 
		{
			auto isEdge = (x == 0 || x + 1 == sizeX) || (y == 0 || y + 1 == sizeY);
			auto value = isEdge ? 0. : -1.;

			T.SetValue(x, y, value);
		}
	}

	T.SetValue(sizeX / 2, sizeY / 2, 200.);
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	ProcessInput();

	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

Grid<double> DiffusionSimulator::diffuseTemperatureExplicit(float timestep) {//add your own parameters
	auto newT = Grid<double>(T.GetSizeX(), T.GetSizeY());
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	
	for (uint32_t y = 1; y < newT.GetSizeY() - 1; y++)
	{
		for (uint32_t x = 1; x < newT.GetSizeX() - 1; x++)
		{
			auto tVal = T.GetValue(x, y);
			
			auto tValNextX = T.GetValue(x+1, y);
			auto tValPrevX = T.GetValue(x-1, y);
							  
			auto tValNextY = T.GetValue(x, y+1);
			auto tValPrevY = T.GetValue(x, y-1);

			auto alphaMul = _alpha * (
				(-2. * tVal + tValNextX + tValPrevX) +
				(-2. * tVal + tValNextY + tValPrevY)
			);

			auto val = tVal + alphaMul * timestep;
			newT.SetValue(x, y, val);
		}
	}

	//delete(T);
	return newT;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}



void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
		A.set_element(i, i, 1); // set diagonal
	}
}


void fillT(std::vector<Real> x) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x);//copy x to T
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::drawObjects()
{
	const float sphereSize = .05f;

	auto sizeX = T.GetSizeX();
	auto sizeY = T.GetSizeY();
	for (uint32_t y = 0; y < sizeY; y++)
	{
		for (uint32_t x = 0; x < sizeX; x++)
		{
			Vec3 pos(
				((float)x - sizeX / 2) * sphereSize,
				((float)y - sizeX / 2) * sphereSize,
				0
			);

			auto tVal = T.GetValue(x, y);
			DUC->setUpLighting(Vec3(), Vec3(tVal, 0, -tVal), 50., Vec3(tVal, 0, -tVal));
			DUC->drawSphere(pos, Vec3(sphereSize * .5f, sphereSize * .5f, sphereSize * .5f));
		}
	}
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
