#pragma once
#include "util/vectorbase.h"
#include "util/matrixbase.h"
#include "util/quaternion.h"
namespace GamePhysics {
	const Vec3 GRAVITY(0.f, -9.81f, 0.f);

	struct Rigidbody {
		Vec3 size;
		Vec3 position;
		Quat rotation;
		Vec3 velocity;
		Vec3 angularVelocity;
		double mass;
		matrix4x4<double> inertia;
	};

	struct Collision {
		Vec3 position;
		Vec3 normal;
		float depth;
		int rb1;
		int rb2;
	};
}
