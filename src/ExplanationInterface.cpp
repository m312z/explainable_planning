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

	void planCallback(const rosplan_dispatch_msgs::CompletePlan plan) {
		for(int i=0; i<plan.plan.size(); i++) {
			// print action
			std::cout << i << ":\t(" << plan.plan[i].name;
			for(int j=0; j<plan.plan[i].parameters.size(); j++) {
				std::cout << " " << plan.plan[i].parameters[j].value;
			}
			std::cout << ")" << std::endl;
		}
	}

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "explanations_text_interface");
		ros::NodeHandle nh("~");

		// planning interface
		std::string opTopic = "/kcl_rosplan/get_domain_operators";
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string actionTopic = "/rosplan_plan_explainer/get_actions";
		std::string applyTopic = "/rosplan_plan_explainer/apply_action";
		std::string moveTopic = "/rosplan_plan_explainer/move_execution";

		ros::ServiceClient op_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>(opTopic);
		ros::ServiceClient problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		ros::ServiceClient planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		ros::ServiceClient parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		ros::ServiceClient action_client = nh.serviceClient<explainable_planning_msgs::IntegerChoice>(actionTopic);
		ros::ServiceClient apply_client = nh.serviceClient<explainable_planning_msgs::IntegerChoice>(applyTopic);
		ros::ServiceClient move_client = nh.serviceClient<explainable_planning_msgs::IntegerChoice>(moveTopic);

		// subscribers
		std::string aas_topic = "/rosplan_plan_explainer/applicable_actions";
		std::string lps_topic = "/rosplan_parsing_interface/complete_plan";

		ros::Subscriber lps = nh.subscribe(lps_topic, 100, &planCallback);
		ros::Subscriber aas = nh.subscribe(aas_topic, 100, &planCallback);

		ros::spinOnce();

		rosplan_knowledge_msgs::GetDomainOperatorService opsrv;
		op_client.call(opsrv);

		std::string in;

		while (ros::ok()) {

			std::cout << "Generating a new plan..." << std::endl;

			// generate a plan
			std_srvs::Empty empty;
			problem_client.call(empty);
			ros::Duration(1).sleep(); // sleep for a second
			planning_client.call(empty);
			ros::Duration(1).sleep(); // sleep for a second
			parsing_client.call(empty);
			ros::Duration(1).sleep(); // sleep for a second

			ros::spinOnce();

			//------------------------------------------//
			// move to the correct position in the plan //
			//------------------------------------------//

			std::cout << "Choose an action (enter integer) or type \"quit\":" << std::endl;
			std::getline (std::cin, in);
			if(in == "quit") return 0;

			std::cout << "Finding state at " << in << "..." << std::endl;

			explainable_planning_msgs::IntegerChoice srv;
			srv.request.option = std::atoi(in.c_str());
			move_client.call(srv);
			ros::Duration(1).sleep(); // sleep for a second

			//--------------------//
			// choose an operator //
			//--------------------//

			for(int i=0; i<opsrv.response.operators.size(); i++) {
				// print op name
				std::cout << i << ":\t(" << opsrv.response.operators[i].name << ")" << std::endl;
			}

			std::cout << "Choose an operator to apply (enter integer) or type \"quit\":" << std::endl;
			std::getline (std::cin, in);
			if(in == "quit") return 0;

			std::cout << "Finding applicable actions..." << std::endl;
			srv.request.option = std::atoi(in.c_str());
			action_client.call(srv);
			ros::Duration(1).sleep(); // sleep for a second

			ros::spinOnce();

			//------------------//
			// choose an action //
			//------------------//

			std::cout << "Choose an new action to apply (enter integer) or type \"quit\":" << std::endl;
			std::getline (std::cin, in);
			if(in == "quit") return 0;

			std::cout << "Applying action..." << std::endl;
			srv.request.option = std::atoi(in.c_str());
			apply_client.call(srv);
		}

		return 0;
	}
