#include "ackermann_planner/plan_node_list.hpp"

#include <sstream>

PlanNode::PlanNode(geometry_msgs::msg::Pose pose,
		int iteration,
		double cost,
		double angle,
		double distance,
		PlanNode* parent)
: pose_(pose), iteration_(iteration), cost_(cost), parent_(parent),
  angle_(angle), distance_(distance)
{
	this->next = NULL;
	this->prev = NULL;
}

PlanNode* PlanNode::getNext() {
	return next;
}

PlanNode* PlanNode::getPrev() {
	return prev;
}

double PlanNode::getCost() {
	return cost_;
}

double PlanNode::getAngle() {
	return angle_;
}

double PlanNode::getDistance() {
	return distance_;
}

int PlanNode::getIteration() {
	return iteration_;
}

PlanNode* PlanNode::getParent() {
	return parent_;
}

geometry_msgs::msg::Pose PlanNode::getPose() {
	return pose_;
}

void PlanNode::setNext(PlanNode* node) {
	next = node;
}

void PlanNode::setPrev(PlanNode* node) {
	prev = node;
}

void PlanNode::insertAfter(PlanNode* node) {
	PlanNode* oldNext = next;

	next = node;
	node->setPrev(this);
	node->setNext(oldNext);
	if (oldNext)
		oldNext->setPrev(this);
}

void PlanNode::insertBefore(PlanNode* node) {
	PlanNode* oldPrev = prev;

	prev = node;
	node->setNext(this);
	node->setPrev(oldPrev);
	if (oldPrev)
		oldPrev->setNext(this);

}

PlanNodeList::PlanNodeList() {
	lastPeak = NULL;
	cheapest = NULL;
}

void PlanNodeList::insertDownwards(PlanNode *node) {
	PlanNode *cur = lastPeak;
	while(1) {
		if (cur->getPrev()) {
			if (cur->getPrev()->getCost() < node->getCost())
				break;
			cur = cur->getPrev();
		}else
			break;
	}
	//Insert here
	cur->insertBefore(node);
}

void PlanNodeList::insertUpwards(PlanNode *node) {
	PlanNode *cur = lastPeak;
	while(1) {
		if (cur->getNext()) {
			if (cur->getNext()->getCost() > node->getCost())
				break;
			cur = cur->getNext();
		} else
			break;
	}

	//Insert here
	cur->insertAfter(node);
}

void PlanNodeList::insertNode(PlanNode *node) {
	if (lastPeak == NULL) {
		lastPeak = node;
		cheapest = node;
		return;
	}

	if (node->getCost() < lastPeak->getCost())
		insertDownwards(node);
	else
		insertUpwards(node);

	lastPeak = node;
	if (node->getCost() < cheapest->getCost())
		cheapest = node;
}

PlanNode* PlanNodeList::removeLeastCost() {
	PlanNode *tmp;

	//Empty list
	if (cheapest == NULL)
		return NULL;

	tmp = cheapest;
	cheapest = cheapest->getNext();

	if (cheapest)
		cheapest->setPrev(NULL);
	tmp->setNext(NULL);

	if(tmp == lastPeak)
		lastPeak = cheapest;


	return tmp;
}

std::string PlanNodeList::printList() {
	std::stringstream ss;  // Use a stringstream to build the stringa

	PlanNode *cur = cheapest;
	//printf("Cheapest: %p, lastPeak: %p\n", (void*)cheapest, (void*)lastPeak);
	ss << "Cheapest: " << (void*)cheapest << ", lastPeak: " << (void*)lastPeak << "\n";
	while(cur) {
		ss << "Pose: " << cur->getPose().position.x << "," << cur->getPose().position.y
			<< " , Cost: " << cur->getCost() << "\n";
		//printf("Pose: %lf,%lf , Cost: %lf\n",
		//		cur->getPose().position.x,
		//		cur->getPose().position.y,
		//		cur->getCost());
		cur = cur->getNext();
	}

	return ss.str();
}

void PlanNodeList::emptyList() {
	PlanNode *tmp = removeLeastCost();
	while(tmp != NULL){
		if (tmp)
			delete tmp;
		tmp = removeLeastCost();
	}

	cheapest = NULL;
	lastPeak = NULL;
}
