#ifndef PLAN_NODE_LIST__PLAN_NODE_LIST_HPP_
#define PLAN_NODE_LIST__PLAN_NODE_LIST_HPP_

#include "geometry_msgs/msg/pose.hpp"

class PlanNode {
public:
	PlanNode(geometry_msgs::msg::Pose pose, int iteration, double cost, PlanNode *parent);

	PlanNode* getNext();
	PlanNode* getPrev();

	void setNext(PlanNode* node);
	void setPrev(PlanNode* node);

	geometry_msgs::msg::Pose getPose();
	double getCost();
	int getIteration();
	PlanNode* getParent();

	void insertAfter(PlanNode* node);
	void insertBefore(PlanNode* node);
private:
	geometry_msgs::msg::Pose pose_;
	int iteration_;
	double cost_;
	PlanNode *parent_;

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
	void printList();

private:
	PlanNode *lastPeak;
	PlanNode *cheapest;
};


#endif // PLAN_NODE_LIST__PLAN_NODE_LIST_HPP_
