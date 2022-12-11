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
		T = diffuseTemperatureImplicit(timeStep);
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


void setupA(SparseMatrix<Real>& a, double factor, Grid<double>& grid) 
{
	auto calc_2d_array_index = [](uint32_t x, uint32_t y, uint32_t gridSizeX) -> uint32_t {
		return y * gridSizeX + x;
	};
	
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	const int n = grid.GetSizeX() * grid.GetSizeY();//N = sizeX*sizeY*sizeZ
	for (int i = 0; i < n; i++) {
		a.set_element(i, i, 1); // set diagonal
	}


	auto sizeX = grid.GetSizeX();
	auto sizeY = grid.GetSizeY();
	auto row = 0;
	for (uint32_t y = 0; y < sizeY; y++)
	{
		for (uint32_t x = 0; x < sizeX; x++)
		{
			if (x != 0 && y != 0 && x != sizeX - 1 && y != sizeY - 1) {
				a.set_element(row, calc_2d_array_index(x, y - 1, sizeX), -factor);
				a.set_element(row, calc_2d_array_index(x - 1, y, sizeX), -factor);
				a.set_element(row, calc_2d_array_index(x, y, sizeX), 4. * factor + 1.);
				a.set_element(row, calc_2d_array_index(x + 1, y, sizeX), -factor);
				a.set_element(row, calc_2d_array_index(x, y + 1, sizeX), -factor);
			}
			
			row++;
		}
	}
}



Grid<double> DiffusionSimulator::diffuseTemperatureImplicit(float timestep) {//add your own parameters
	// solve A T = b
	// to be implemented
	const int n = T.GetSizeX() * T.GetSizeY();//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> a(n);	//lower-case letter a, conventions are important to respect!! We are not mathematicians, we are informaticians
	const std::vector<Real>& b = T.GetValues();

	setupA(a, _alpha * timestep, T);
	//setupB(b, T);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	auto newT = Grid<double>(T.GetSizeX(), T.GetSizeX());
	std::vector<Real>& x = newT.GetValues();
	//for (int j = 0; j < n; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(a, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values

	return newT;
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
