#ifndef ACKERMAN_PLANNER__CLOSED_POINTS_HPP_
#define ACKERMAN_PLANNER__CLOSED_POINTS_HPP_

typedef unsigned int uint;

class ClosedPoint {
public:
	ClosedPoint(double angleTolerance);
	~ClosedPoint();

	bool isClosed(double angle, uint epoch);
	void closeAngle(double angle, uint epoch);

private:
	void validate(uint epoch);
	void reset(uint epoch);

	int getIndex(double angle);

	uint epoch;
	bool* data;
	double tolerance;
};

class ClosedPointsPlane {
public:
	ClosedPointsPlane(uint cellsX,
				 uint cellsY,
				 double angleTolerance);
	~ClosedPointsPlane();

	void nextEpoch();

	bool isClosed(uint mx, uint my, double angle);
	void closePoint(uint mx, uint my, double angle);

protected:

	inline unsigned int getIndex(unsigned int mx, unsigned int my)
	{
		return my * x_ + mx;
	}

	int epoch;
	unsigned int x_;
	unsigned int y_;

	ClosedPoint** data;

};

#endif // ACKERMAN_PLANNER__CLOSED_POINTS_HPP_
