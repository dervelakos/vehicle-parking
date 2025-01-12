#include "ackermann_planner/closed_points.hpp"

#include <cmath>
#include <iostream>

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
: x_(cellsX), y_(cellsY), angleTolerance_(angleTolerance)
{
	printf("Initialized with %u, %u\n", cellsX, cellsY);
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
	if(mx > x_ || my > y_){
		printf("Point outside of map!\n");
		return true;
	}

	ClosedPoint *p = data[getIndex(mx,my)];
	if (p == NULL){
		printf("Point not initialized!\n");
		return true;
	}

	return p->isClosed(
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

void
ClosedPointsPlane::shrink(uint newSize)
{
	ClosedPoint **newData = new ClosedPoint*[newSize];
	uint oldPointsNum = getIndex(x_, y_);

	for (uint i=0; i<newSize; i++) {
		newData[i] = data[i];
	}
	for (uint i=newSize; i<oldPointsNum; i++) {
		delete data[i];
	}

	delete [] data;
	data = newData;
}

bool
ClosedPointsPlane::expand(uint newSize)
{
	ClosedPoint **newData = new ClosedPoint*[newSize];
	uint oldPointsNum = getIndex(x_, y_);

	for (uint i=0; i<oldPointsNum; i++) {
		newData[i] = data[i];
	}
	for (uint i=oldPointsNum; i<newSize; i++) {
		newData[i] = new ClosedPoint(angleTolerance_);
		if (newData[i] == NULL)
			return false;
	}

	delete [] data;
	data = newData;
	return true;
}

bool
ClosedPointsPlane::resize(uint cellsX, uint cellsY)
{
	bool res = true;
	uint pointsNum = cellsY * cellsX + cellsX;
	uint oldPointsNum = getIndex(x_, y_);

	printf("Resized to %u, %u\n", cellsX, cellsY);
	nextEpoch();

	if (pointsNum > oldPointsNum)
		res = expand(pointsNum);
	else
		shrink(pointsNum);

	x_ = cellsX;
	y_ = cellsY;
	return res;
}

uint
ClosedPointsPlane::getCellsX()
{
	return x_;
}

uint
ClosedPointsPlane::getCellsY()
{
	return y_;
}
