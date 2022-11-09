#pragma once
#include "util/vectorbase.h"

namespace GamePhysics {
	enum IntegrationMethod {
		Euler = 0, Leapfrog = 1, Midpoint = 2
	};

	const Vec3 GRAVITY = Vec3(0, -9.81f, 0);

	namespace Entities {
		struct MassPoint {
			float mass;
			Vec3 velocity;
			Vec3 position;

			MassPoint(float mass, Vec3 velocity, Vec3 position)
			{
				this->mass = mass;
				this->velocity = velocity;
				this->position = position;
			}

			//Create unmovable masspoint
			MassPoint(Vec3 position) : velocity()
			{
				this->position = position;
				this->mass = std::numeric_limits<float>::infinity();
			}

			

			bool isFixed() {
				return mass == std::numeric_limits<float>::infinity();
			}
		};

		struct Spring {
			float stiffness;
			float restLength;
			int massPointIndex1;
			int massPointIndex2;

			Spring(float stiffness, float restLength, int massPointIndex1, int massPointIndex2) {
				this->stiffness = stiffness;
				this->restLength = restLength;
				this->massPointIndex1 = massPointIndex1;
				this->massPointIndex2 = massPointIndex2;
			}
		};
	}
}