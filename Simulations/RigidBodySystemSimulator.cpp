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
		setOrientationOf(0, Quat(0, 0, deg2rad(9090)));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

		break;
	case 1:
		addRigidBody(Vec3(-1, 0, 0), Vec3(1, 2, 1), 2);
		setOrientationOf(0, Quat(0, 0, deg2rad(90)));
		applyForceOnBody(0, Vec3(-1, .5, 0), Vec3(10, 0, 0));

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


	for (size_t i = 0; i < m_rigidbodies.size(); i++)
	{
		m_rigidbodies[i].PrecomputeInertia();
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
		m.initTRS(rb.position, rb.rotation.getRotMat(), rb.size);

		//m.initRotationY(45);

		auto directXMat = m.toDirectXMatrix();
		DUC->drawRigidBody(directXMat);

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

	std::vector<Collision> cols;
	FindCollisions(cols);

	#pragma region MAGIC
	for (size_t i = 0; i < cols.size(); i++)
	{
		auto& col = cols[i];
		auto& rb1 = m_rigidbodies[col.rb1];
		auto& rb2 = m_rigidbodies[col.rb2];

		//velocity at collision point
		auto vCol1 = rb1.velocity + cross(rb1.angularVelocity, col.position);
		auto vCol2 = rb2.velocity + cross(rb2.angularVelocity, col.position);

		//relative velocity
		auto vRel = vCol1 - vCol2;
		auto dotRelativeNormal = dot(vRel, col.normal);
		if (dotRelativeNormal > 0) continue;

		//J formula
		const float c = 0.5f;
		auto inertiaRb1 = rb1.GetCurrentInertiaTensor();
		auto inertiaRb2 = rb2.GetCurrentInertiaTensor();

		auto inertiaInverseRb1 = inertiaRb1;
		auto inertiaInverseRb2 = inertiaRb2;
		inertiaInverseRb1.transpose();
		inertiaInverseRb2.transpose();

		float nominator = -(1 + c) * dotRelativeNormal;
		auto cross1 = cross(rb1.position, col.normal);
		auto cross2 = cross(rb2.position, col.normal);
		float denominator = 
			(1.0f / rb1.mass) + 
			(1.0f / rb2.mass) + 
			dot(
				(cross(
					inertiaInverseRb1.transformVector(cross1), rb1.position)) +
				(cross(
					inertiaInverseRb2.transformVector(cross2), rb2.position)),
				col.normal
			);
		float j = nominator / denominator;

		//Add impulse to velocity
		rb1.velocity += j * col.normal / rb1.mass;
		rb2.velocity -= j * col.normal / rb2.mass;

		//L = I * w
		auto l1 = inertiaRb1.transformVector(rb1.angularVelocity);
		auto l2 = inertiaRb2.transformVector(rb2.angularVelocity);

		auto l1Prime = l1 + cross(rb1.position, j * col.normal);
		auto l2Prime = l2 - cross(rb2.position, j * col.normal);

		//w = I^-1 * L
		rb1.angularVelocity = inertiaInverseRb1.transformVector(l1Prime);
		rb1.angularVelocity = inertiaInverseRb2.transformVector(l2Prime);
	}
	#pragma endregion



	for (size_t i = 0; i < m_rigidbodies.size(); i++)
	{
		auto& rb = m_rigidbodies[i];

		//apply external forces
		auto force = m_externalForce;

		//Integrate
		rb.position += rb.velocity * timeStep;

		//rb.angularMomentum += timeStep * rb.torque;
		auto inverseInertia = rb.GetCurrentInertiaTensor();
		rb.angularVelocity = inverseInertia.transformVector(rb.angularMomentum);

		auto rotQuat = Quat(0, rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z);
		rb.rotation = (rb.rotation + (timeStep * .5 * rotQuat * rb.rotation));
		rb.rotation = rb.rotation.unit();
		

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

			cout << "mousevec: " << mouseVector << "\n";
			cout << "vel: " << rb.velocity << "\n";

			rb.velocity += mouseVector;
		}

		//Apply damping
		//rb.velocity *= m_velocityDamping;
		/*rb.angularMomentum *= m_angularVelocityDamping;*/

		//cout << "vel: " << rb.velocity << "\n";
		cout << "ang: " << rb.angularVelocity << "\n";

		//cout << "vel: " << rb.velocity << "\n";
		//cout << "angular: " << rb.angularVelocity << "\n";

	}
}

void RBSS::FindCollisions(std::vector<Collision>& collisions) {

	//collision between rb <-> rb
	for (size_t i = 0; i < m_rigidbodies.size(); i++)
	{
		matrix4x4<double> m1, m2;
		m1.initTRS(m_rigidbodies[i].position, m_rigidbodies[i].rotation.getRotMat(), m_rigidbodies[i].size);

		for (size_t j = i + 1; j < m_rigidbodies.size(); j++)
		{
			m2.initTRS(m_rigidbodies[j].position, m_rigidbodies[j].rotation.getRotMat(), m_rigidbodies[j].size);

			auto col = checkCollisionSAT(m1, m2);
			if (col.isValid) {
				Collision x;
				x.rb1 = i;
				x.rb2 = j;
				x.position = col.collisionPointWorld;
				x.normal = col.normalWorld;
				x.depth = col.depth;

				collisions.push_back(x);
			}
		}
	}

	//collision between rb <-> ground
	for (size_t i = 0; i < m_rigidbodies.size(); i++)
	{
		

		//collision
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
	auto& rb = m_rigidbodies[i];

	auto torque = cross(loc, force);
	rb.angularMomentum = torque;
}
void RBSS::addRigidBody(Vec3 position, Vec3 size, int mass){
	Rigidbody rb;
	rb.position = position;
	rb.size = size;
	rb.mass = mass;
	rb.rotation = Quat(1,0,0,0);

	m_rigidbodies.push_back(rb);
}
void RBSS::setOrientationOf(int i, Quat orientation){
	m_rigidbodies[i].rotation = orientation;
}
void RBSS::setVelocityOf(int i, Vec3 velocity) {
	m_rigidbodies[i].velocity = velocity;
}
#pragma endregion