#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
template <class T>
class Grid {
public:
	// Construtors
	Grid(uint32_t x, uint32_t y)
	{
		this->_sizeX = x;
		this->_sizeY = y;

		_values.resize(x * y);
	}

	uint32_t GetSizeX() { return _sizeX; };
	uint32_t GetSizeY() { return _sizeY; };

	T GetValue(uint32_t x, uint32_t y) { return _values[_sizeX * y + x]; };
	void SetValue(uint32_t x, uint32_t y, T val) { _values[_sizeX * y + x] = val; };

private:
	uint32_t _sizeX;
	uint32_t _sizeY;

	std::vector<T> _values;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();
	~DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid<double> diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit();

private:
	// Functions
	void ProcessInput();
	void InitializeTemperatureGrid();

	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	uint32_t _gridSizeX = 16;
	uint32_t _gridSizeY = 16;
	Grid<double> T; //save results of every time step
	double _alpha = 1;
};

#endif