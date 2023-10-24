#pragma once

#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;


double get_dt_s_hrt(hrt_abstime &time_stamp);

class point
{
private:
	hrt_abstime	timestamp{0};
public:
	Vector3f pos; // [x y z]
	Vector3f vel; // [x y z]
	Vector3f acc; // [x y z]
	Vector3f jerk; // [x y z]
	Vector3f snap; // [x y z]
	Vector3f yaw; // [psi psi_dot psi_ddot]

	void start(void);
	void reset(void);

	point(/* args */);
	~point();
};

point::point(/* args */)
{
}

point::~point()
{
	pos = Vector3f(0.0f, 0.0f, 0.0f);
	vel = Vector3f(0.0f, 0.0f, 0.0f);
	acc = Vector3f(0.0f, 0.0f, 0.0f);
	jerk = Vector3f(0.0f, 0.0f, 0.0f);
	snap = Vector3f(0.0f, 0.0f, 0.0f);
	yaw = Vector3f(0.0f, 0.0f, 0.0f);
}




class trajectory
{
private:
	point initial_pt{};
	point current_pt{};

	void start(void);
	void reset(void);

public:
	trajectory(/* args */);
	~trajectory();
};

trajectory::trajectory(/* args */)
{
}

trajectory::~trajectory()
{

}

