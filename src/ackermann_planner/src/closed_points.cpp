#include "ackermann_planner/closed_points.hpp"

#include <cmath>

double normalizeAngleDegrees(double angle) {
	double normalizedAngle = fmod(angle, 360.0);

	if (normalizedAngle < 0) {
		normalizedAngle += 360.0;
	}

	return normalizedAngle;
}

ClosedPoint::ClosedPoint(double angleTolerance)
	:tolerance(angleTolerance)
{
	const uint slots = 360/tolerance;

	epoch=0;
	data = new bool[slots];
}

ClosedPoint::~ClosedPoint()
{
	delete[] data;
}

void ClosedPoint::validate(uint planeEpoch)
{
	if (epoch != planeEpoch)
		reset(planeEpoch);
}

void ClosedPoint::reset(uint planeEpoch)
{
	epoch = planeEpoch;
	for (int i=0; i<(360/tolerance)+1; i++) {
		data[i] = false;
	}
}

int ClosedPoint::getIndex(double angle)
{
	return angle/tolerance;
}

bool ClosedPoint::isClosed(double angle, uint planeEpoch)
{
	validate(planeEpoch);
	return data[getIndex(angle)];
}

void ClosedPoint::closeAngle(double angle, uint planeEpoch)
{
	validate(planeEpoch);
	data[getIndex(angle)] = true;
}


ClosedPointsPlane::ClosedPointsPlane(unsigned int cellsX,
									 unsigned int cellsY,
									 double angleTolerance)
: x_(cellsX), y_(cellsY)
{
	uint pointsNum = getIndex(cellsX, cellsY);
	epoch = 1;
	data = new ClosedPoint*[pointsNum];
	for (uint i=0; i<pointsNum; i++) {
		data[i] = new ClosedPoint(angleTolerance);
	}
}

ClosedPointsPlane::~ClosedPointsPlane()
{
	uint pointsNum = getIndex(x_, y_);

	for (uint i=0U; i<pointsNum; i++) {
		delete data[i];
	}

	delete [] data;
}


void ClosedPointsPlane::nextEpoch()
{
	epoch++;
}

bool
ClosedPointsPlane::isClosed(uint mx, uint my, double angle)
{
	return data[getIndex(mx,my)]->isClosed(
		normalizeAngleDegrees(angle),
		epoch
	);

}

void
ClosedPointsPlane::closePoint(uint mx, uint my, double angle)
{
	data[getIndex(mx,my)]->closeAngle(
		normalizeAngleDegrees(angle),
		epoch
	);
}
