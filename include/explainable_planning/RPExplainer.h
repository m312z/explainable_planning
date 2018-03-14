#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "std_srvs/Empty.h"

#include "explainable_planning_msgs/IntegerChoice.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"

#ifndef KCL_explainer
#define KCL_explainer

/**
 * This file defines the RPExplainer class.
 * RPExplainer is a node through which the user can interact with the planner and
 * planning system to request explanations and perform interactive planning.
 */
namespace KCL_rosplan {

	class RPExplainer
	{

	private:

		ros::NodeHandle *node_handle;

		std::string action_dispatch_topic;
		std::string action_feedback_topic;
		std::string applicable_action_topic;

		ros::ServiceClient query_knowledge_client;
		ros::ServiceClient get_operator_details_client;
		ros::ServiceClient get_operators_client;
		ros::ServiceClient get_instances_client;

		/* action selection */
		std::vector<rosplan_knowledge_msgs::DomainFormula> grounding_list;
		std::vector<rosplan_knowledge_msgs::DomainFormula> partial_grounding_list;
		rosplan_dispatch_msgs::CompletePlan applicable_actions;

		void groundOperator(int parameter, std::vector<std::string> &parameterList);

		/* plan dispatch */
		bool plan_recieved;
		rosplan_dispatch_msgs::CompletePlan current_plan;
		std::map<int,bool> action_received;
		std::map<int,bool> action_completed;
		int current_action;

		bool dispatchAction(rosplan_dispatch_msgs::ActionDispatch action);

		bool checkConditions(rosplan_dispatch_msgs::ActionDispatch msg);
		bool checkConditionList(rosplan_dispatch_msgs::ActionDispatch msg,
					std::vector<rosplan_knowledge_msgs::DomainFormula> &conditionList,
					std::vector<diagnostic_msgs::KeyValue> &paramList, bool negative);

	public:

		/* constructor */
		RPExplainer(ros::NodeHandle &nh);

		/* srevices */
		bool generateApplicableActions(explainable_planning_msgs::IntegerChoice::Request &req, explainable_planning_msgs::IntegerChoice::Response &res);
		bool moveExecution(explainable_planning_msgs::IntegerChoice::Request &req, explainable_planning_msgs::IntegerChoice::Response &res);
		bool executeApplicableAction(explainable_planning_msgs::IntegerChoice::Request &req, explainable_planning_msgs::IntegerChoice::Response &res);

		/* action publishers */
		ros::Publisher action_dispatch_publisher;
		ros::Publisher action_feedback_publisher;
		ros::Publisher applicable_action_publisher;

		void planCallback(const rosplan_dispatch_msgs::CompletePlan plan);
		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
	};
}
#endif
