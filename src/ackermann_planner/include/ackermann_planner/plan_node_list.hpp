#ifndef PLAN_NODE_LIST__PLAN_NODE_LIST_HPP_
#define PLAN_NODE_LIST__PLAN_NODE_LIST_HPP_

#include <string>

#include "geometry_msgs/msg/pose.hpp"

class PlanNode {
public:
	PlanNode(geometry_msgs::msg::Pose pose,
			int iteration,
			double cost,
			double angle,
			double distance,
			PlanNode *parent);

	PlanNode* getNext();
	PlanNode* getPrev();

	void setNext(PlanNode* node);
	void setPrev(PlanNode* node);

	geometry_msgs::msg::Pose getPose();
	double getCost();
	int getIteration();
	PlanNode* getParent();
	double getAngle();
	double getDistance();

	void insertAfter(PlanNode* node);
	void insertBefore(PlanNode* node);
private:
	geometry_msgs::msg::Pose pose_;
	int iteration_;
	double cost_;
	PlanNode *parent_;
	double angle_;
	double distance_;

	PlanNode *next;
	PlanNode *prev;
};

class PlanNodeList {
public:
	PlanNodeList();
	void insertDownwards(PlanNode *node);
	void insertUpwards(PlanNode *node);
	void insertNode(PlanNode *node);
	PlanNode* removeLeastCost();
	std::string printList();
	void emptyList();

private:
	PlanNode *lastPeak;
	PlanNode *cheapest;
};


#endif // PLAN_NODE_LIST__PLAN_NODE_LIST_HPP_
