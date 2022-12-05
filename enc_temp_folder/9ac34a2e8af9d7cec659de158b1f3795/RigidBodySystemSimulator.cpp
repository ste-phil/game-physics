#include "RigidBodySystemSimulator.h"

using RBSS = RigidBodySystemSimulator;


void RBSS::onClick(int x, int y) {
	if (m_ignoreMouseInput) return;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	m_mousePressed = true;
}


void RBSS::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	m_mousePressed = false;
}



// Functions
const char* RBSS::getTestCasesStr(){
	return "Demo2,Demo3,Demo4";
}

void RBSS::initUI(DrawingUtilitiesClass* DUC){
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Ground Collider", TW_TYPE_BOOLCPP, &m_useGroundCollider, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_useGravity, "");

	TwAddVarRW(DUC->g_pTweakBar, "Velocity Damping", TW_TYPE_FLOAT, &m_velocityDamping, "min=0,max=1");
	TwAddVarRW(DUC->g_pTweakBar, "Angular Velocity Damping", TW_TYPE_FLOAT, &m_angularVelocityDamping, "min=0,max=1");

	TwAddVarRW(DUC->g_pTweakBar, "Ignore Mouse Input", TW_TYPE_BOOL32, &m_ignoreMouseInput, "");

	//TwType TW_TYPE_INTEGRATORS = TwDefineEnumFromString("Integrators", "Euler,Leapfrog,Midpoint");
	//TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATORS, &m_iIntegrator, "");
}

double deg2rad(double x) {
	return x / 360.0 * 2.0 * 3.141592653589793238462643383279502884197169399;
}

void RBSS::notifyCaseChanged(int testCase) {
	reset();

	switch (testCase) {
	case 0:
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(0, 0, deg2rad(90)));

		break;
	case 1:
		addRigidBody(Vec3(-1, 0, 0), Vec3(1, 2, 1), 2);
		setOrientationOf(0, Quat(0, 0, deg2rad(90)));

		addRigidBody(Vec3(1, 0, 0), Vec3(1, 1, 1), 2);

		break;
	case 2:
		const float boxSize = .2f;

		addRigidBody(Vec3(1, 0, 0), Vec3(boxSize, boxSize, boxSize), 2);
		addRigidBody(Vec3(0, 1, 0), Vec3(boxSize, boxSize * 2.0f, boxSize), 2);
		addRigidBody(Vec3(0, 0, 0), Vec3(boxSize, boxSize, boxSize), 2);
		addRigidBody(Vec3(1, 1, 0), Vec3(boxSize, boxSize, boxSize), 2);
		break;
	}
}

void RBSS::reset(){
	m_rigidbodies.clear();
}

void RBSS::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));

	const float sphereSize = .01f;

	for (size_t i = 0; i < m_rigidbodies.size(); i++)
	{
		auto rb = m_rigidbodies[i];
		auto& rbPos = rb.position;
		auto h = m_rigidbodies[i].size / 2.0f;

		auto rotationMat = m_rigidbodies[i].rotation.getRotMat();

		matrix4x4<double> m;
		m.initTRS(rb.position, ToEulerAngles(rb.rotation), rb.size);
		DUC->drawRigidBody(m.toDirectXMatrix());

		//auto bottomFrontLeft = m_rigidbodies[i].position	+ (rotationMat * Vec3(-h.x, -h.y,  h.z));
		//auto bottomFrontRight = m_rigidbodies[i].position	+ (rotationMat * Vec3( h.x, -h.y,  h.z));
		//auto bottomBackLeft = m_rigidbodies[i].position		+ (rotationMat * Vec3(-h.x, -h.y, -h.z));
		//auto bottomBackRight = m_rigidbodies[i].position	+ (rotationMat * Vec3( h.x, -h.y, -h.z));
		//auto topBackLeft = m_rigidbodies[i].position		+ (rotationMat * Vec3(-h.x,  h.y, -h.z));
		//auto topBackRight = m_rigidbodies[i].position		+ (rotationMat * Vec3( h.x,  h.y, -h.z));
		//auto topFrontLeft = m_rigidbodies[i].position		+ (rotationMat * Vec3(-h.x,  h.y,  h.z));
		//auto topFrontRight = m_rigidbodies[i].position		+ (rotationMat * Vec3( h.x,  h.y,  h.z));

		//DUC->drawSphere(bottomFrontLeft, Vec3(sphereSize));
		//DUC->drawSphere(bottomBackLeft, Vec3(sphereSize));
		//DUC->drawSphere(bottomFrontRight, Vec3(sphereSize));
		//DUC->drawSphere(bottomBackRight, Vec3(sphereSize));
		//DUC->drawSphere(topBackLeft, Vec3(sphereSize));
		//DUC->drawSphere(topBackRight, Vec3(sphereSize));
		//DUC->drawSphere(topFrontRight, Vec3(sphereSize));
		//DUC->drawSphere(topFrontLeft, Vec3(sphereSize));

		////top part of cube
		//DUC->beginLine();
		//DUC->drawLine(topBackLeft, Vec3(1), topBackRight, Vec3(1));
		//DUC->endLine();
		//DUC->beginLine();
		//DUC->drawLine(topBackRight, Vec3(1), topFrontRight, Vec3(1));
		//DUC->endLine();
		//DUC->beginLine();
		//DUC->drawLine(topFrontRight, Vec3(1), topFrontLeft, Vec3(1));
		//DUC->endLine();
		//DUC->beginLine();
		//DUC->drawLine(topFrontLeft, Vec3(1), topBackLeft, Vec3(1));
		//DUC->endLine();


		////bottom part of cube
		//DUC->beginLine();
		//DUC->drawLine(bottomBackLeft, Vec3(1), bottomBackRight, Vec3(1));
		//DUC->endLine();
		//DUC->beginLine();
		//DUC->drawLine(bottomBackRight, Vec3(1), bottomFrontRight, Vec3(1));
		//DUC->endLine();
		//DUC->beginLine();
		//DUC->drawLine(bottomFrontRight, Vec3(1), bottomFrontLeft, Vec3(1));
		//DUC->endLine();
		//DUC->beginLine();
		//DUC->drawLine(bottomFrontLeft, Vec3(1), bottomBackLeft, Vec3(1));
		//DUC->endLine();


		////vertical connections
		//DUC->beginLine();
		//DUC->drawLine(bottomFrontLeft, Vec3(1), topFrontLeft, Vec3(1));
		//DUC->endLine();

		//DUC->beginLine();
		//DUC->drawLine(bottomBackLeft, Vec3(1), topBackLeft, Vec3(1));
		//DUC->endLine();

		//DUC->beginLine();
		//DUC->drawLine(bottomFrontRight, Vec3(1), topFrontRight, Vec3(1));
		//DUC->endLine();

		//DUC->beginLine();
		//DUC->drawLine(bottomBackRight, Vec3(1), topBackRight, Vec3(1));
		//DUC->endLine();

	}
}

void RBSS::externalForcesCalculations(float timeElapsed){
	m_externalForce = m_useGravity ? GRAVITY : Vec3(0, 0, 0);
}

void RBSS::simulateTimestep(float timeStep){
	//cout << "Simulation internal " << timeStep << endl;

	for (size_t i = 0; i < m_rigidbodies.size(); i++)
	{
		auto& rb = m_rigidbodies[i];

		rb.position += rb.velocity * timeStep;

		auto rotQuat = Quat(0, rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z);
		rb.rotation = (rb.rotation + (timeStep * .5 * rotQuat * rb.rotation));
		rb.rotation.unit();
		

		//Apply mouse input
		if (m_mousePressed && !m_ignoreMouseInput)
		{
			//Get Difference in Mouse Movement
			Point2D mouseDiff;
			mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
			mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
			//Get Position of Mouse Cursor in 3D space, scaled down
			Vec3 mouseVector = ViewportToWorldpoint(DUC->g_camera.GetWorldMatrix(), DUC->g_camera.GetViewMatrix(), mouseDiff) * 0.001f;
			mouseVector.y *= -1;
			//Draw Mouse Vector as Visual Guide
			DUC->drawSphere(mouseVector, Vec3(0.8f));
			DUC->beginLine();
			DUC->drawLine(Vec3(0, 0, 0), Vec3(1, 1, 0), mouseVector, Vec3(1, 1, 0));
			DUC->endLine();

			cout << "mousevec: " << mouseVector << "\n";
			cout << "vel: " << rb.velocity << "\n";

			rb.velocity += mouseVector;
		}

		//Apply damping
		rb.velocity *= m_velocityDamping;
		rb.angularVelocity *= m_angularVelocityDamping;

		//cout << "vel: " << rb.velocity << "\n";
		//cout << "angular: " << rb.angularVelocity << "\n";

	}
}


#pragma region ExtraFunctions
int RBSS::getNumberOfRigidBodies() {
	return m_rigidbodies.size();
}
Vec3 RBSS::getPositionOfRigidBody(int i) {
	return m_rigidbodies[i].position;
}
Vec3 RBSS::getLinearVelocityOfRigidBody(int i){
	return m_rigidbodies[i].velocity;
}
Vec3 RBSS::getAngularVelocityOfRigidBody(int i){
	return m_rigidbodies[i].angularVelocity;

}
void RBSS::applyForceOnBody(int i, Vec3 loc, Vec3 force){

}
void RBSS::addRigidBody(Vec3 position, Vec3 size, int mass){
	Rigidbody rb;
	rb.position = position;
	rb.size = size;
	rb.mass = mass;
	rb.rotation = Quat(0,0,0,0);

	m_rigidbodies.push_back(rb);
}
void RBSS::setOrientationOf(int i, Quat orientation){
	m_rigidbodies[i].rotation = orientation;
}
void RBSS::setVelocityOf(int i, Vec3 velocity) {
	m_rigidbodies[i].velocity = velocity;
}
#pragma endregion