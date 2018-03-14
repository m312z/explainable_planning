#include "explainable_planning/RPExplainer.h"

namespace KCL_rosplan {

	RPExplainer::RPExplainer(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		query_knowledge_client = node_handle->serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		get_operator_details_client = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/kcl_rosplan/get_domain_operator_details");
		get_operators_client = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>("/kcl_rosplan/get_domain_operators");
		get_instances_client = node_handle->serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");

		// action dispatch and subscription
		action_dispatch_topic = "action_dispatch";
		action_feedback_topic = "action_feedback";
		applicable_action_topic = "applicable_actions";
		nh.getParam("action_dispatch_topic", action_dispatch_topic);
		nh.getParam("action_feedback_topic", action_feedback_topic);
		nh.getParam("applicable_action_topic", applicable_action_topic);
		action_dispatch_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionDispatch>(action_dispatch_topic, 1, true);
		action_feedback_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionFeedback>(action_feedback_topic, 1, true);
		applicable_action_publisher = node_handle->advertise<rosplan_dispatch_msgs::CompletePlan>(applicable_action_topic, 1, true);
	}

	/*------------------------------*/
	/* applicable action generation */
	/*------------------------------*/

	/**
	 * Generate a set of applicable actions, given a choice of operator.
	 */
	bool RPExplainer::generateApplicableActions(explainable_planning_msgs::IntegerChoice::Request &req, explainable_planning_msgs::IntegerChoice::Response &res) {

		grounding_list.clear();
		applicable_actions.plan.clear();

		// get domain operator list
		rosplan_knowledge_msgs::GetDomainOperatorService opSrv;
		if(!get_operators_client.call(opSrv)) {
			ROS_ERROR("KCL: (%s) Could not call Knowledge Base for operator list.", ros::this_node::getName().c_str());
			return false;
		}

		// get correct operator
		if(req.option >= opSrv.response.operators.size() || req.option < 0) {
			ROS_WARN("KCL: (%s) Invalid operator selection.", ros::this_node::getName().c_str());
			return false;
		}
		rosplan_knowledge_msgs::DomainFormula op = opSrv.response.operators[req.option];
		ROS_INFO("KCL: (%s) Generating grounded actions for operator %s.", ros::this_node::getName().c_str(), op.name.c_str());

		// ground operator into action list and check preconditions for each
		grounding_list.push_back(op);
		for(int i=0; i<op.typed_parameters.size(); i++) {

			// get possible instances for grounding
			rosplan_knowledge_msgs::GetInstanceService instanceSrv;
			instanceSrv.request.type_name = op.typed_parameters[i].value;
			if(!get_instances_client.call(instanceSrv)) {
				ROS_ERROR("KCL: (%s) Could not call Knowledge Base for instance list for %s.", ros::this_node::getName().c_str(), instanceSrv.request.type_name.c_str());
				return false;
			}

			// ground this parameter
			groundOperator(i, instanceSrv.response.instances);
		}

		// loop through now fully grounded actions to check preconditions
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator oit = grounding_list.begin();
		for(; oit < grounding_list.end(); ) {

			// create action message
			rosplan_dispatch_msgs::ActionDispatch action;
			action.action_id = 0;
			action.name = oit->name;

			// copy parameters to action message
			for(int j=0; j < oit->typed_parameters.size(); j++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = oit->typed_parameters[j].key;
				pair.value = oit->typed_parameters[j].value;
				action.parameters.push_back(pair);
			}

			// check if the action is applicable
			if(checkConditions(action)) {
				applicable_actions.plan.push_back(action);
				oit++;
			} else {
				grounding_list.erase(oit);
			}
		}

		// publish the list of applicable actions
		applicable_action_publisher.publish(applicable_actions);

		return true;
	}

	/**
	 * Take the contents of grounding_list and ground the selected parameter using the values in parameterList.
	 * Store the result in grounding_list.
	 */
	void RPExplainer::groundOperator(int parameter, std::vector<std::string> &parameterList) {

		partial_grounding_list.clear();

		// loop through ungrounded actions
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator oit = grounding_list.begin();
		for(; oit != grounding_list.end(); oit++) {

			// for each instance to be bound to the parameter
			for(int i=0; i < parameterList.size(); i++) {

				// generate a new (partially) grounded opertor
				rosplan_knowledge_msgs::DomainFormula newop;
				newop.name = oit->name;

				// copy existing parameters to new operator
				for(int j=0; j < oit->typed_parameters.size(); j++) {
					diagnostic_msgs::KeyValue pair;
					pair.key = oit->typed_parameters[j].key;
					pair.value = oit->typed_parameters[j].value;
					newop.typed_parameters.push_back(pair);
				}

				// set new parameter
				newop.typed_parameters[parameter].value = parameterList[i];

				// save in partial list
				partial_grounding_list.push_back(newop);
			}
		}

		// store the results in grounding_list
		grounding_list.clear();
		grounding_list = partial_grounding_list;
		partial_grounding_list.clear();
	}

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/**
	 * Service method to dispatch a single applicable action selected by the user.
	 * Invalidates the currently executing plan.
	 */
	bool RPExplainer::executeApplicableAction(explainable_planning_msgs::IntegerChoice::Request &req, explainable_planning_msgs::IntegerChoice::Response &res) {

		// make sure the action is in range
		if(req.option >= applicable_actions.plan.size() || req.option < 0) {
			ROS_WARN("KCL: (%s) Invalid action selection.", ros::this_node::getName().c_str());
			return false;
		}

		rosplan_dispatch_msgs::ActionDispatch action = applicable_actions.plan[req.option];
		action.action_id = current_action;

		// invalidate any remaining planned actions
		current_plan.plan.clear();
		plan_recieved = false;

		// invalidate any applicable actions
		applicable_actions.plan.clear();

		// dispatch single action
		return dispatchAction(action);
	}

	/**
	 * Dispatches a single action. Waits for the action to complete.
	 * @returns true if the action has been succesfully dispatched.
	 */
	bool RPExplainer::dispatchAction(rosplan_dispatch_msgs::ActionDispatch action) {

		// check action preconditions
		if(!checkConditions(action)) {

			ROS_INFO("KCL: (%s) Action not applicable; preconditions are not achieved [%i, %s]", ros::this_node::getName().c_str(), action.action_id, action.name.c_str());

			// publish feedback (precondition false)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = action.action_id;
			fb.status = "precondition false";
			action_feedback_publisher.publish(fb);
			ros::spinOnce();

			return false;

		} else {

			// dispatch action
			ROS_INFO("KCL: (%s) Dispatching action [%i, %s]",
					ros::this_node::getName().c_str(), 
					action.action_id,
					action.name.c_str());

			action_dispatch_publisher.publish(action);

			// wait for action to complete
			ros::Rate loop_rate(10);
			while (ros::ok() && !action_completed[current_action]) {
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

		return true;
	}

	/**
	 *	Returns true of the actions preconditions are true in the current state. Calls the Knowledge Base.
	 */
	bool RPExplainer::checkConditions(rosplan_dispatch_msgs::ActionDispatch msg) {

		// get domain opertor details
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!get_operator_details_client.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
			return false;
		}

		// check preconditions
		if(!checkConditionList(msg, srv.response.op.at_start_simple_condition, srv.response.op.formula.typed_parameters, false)) return false;
		if(!checkConditionList(msg, srv.response.op.over_all_simple_condition, srv.response.op.formula.typed_parameters, false)) return false;
		if(!checkConditionList(msg, srv.response.op.at_start_neg_condition, srv.response.op.formula.typed_parameters, true)) return false;
		if(!checkConditionList(msg, srv.response.op.over_all_neg_condition, srv.response.op.formula.typed_parameters, true)) return false;

		return true;
	}

	/**
	 * Check the conditions from one list against the current state.
	 * @param msg			the action message whose conditions are checked
	 * @param conditionList	the list of conditions to check
	 * @param paramList		list of pairs: operator parameter label, parameter type
	 * @param negative		true if the condition list is a set of negative conditions
	 */
	bool RPExplainer::checkConditionList(rosplan_dispatch_msgs::ActionDispatch msg,
					std::vector<rosplan_knowledge_msgs::DomainFormula> &conditionList,
					std::vector<diagnostic_msgs::KeyValue> &paramList, bool negative) {

		// setup service call
		rosplan_knowledge_msgs::KnowledgeQueryService querySrv;

		// iterate through conditions
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit = conditionList.begin();
		for(; cit!=conditionList.end(); cit++) {

			// create condition
			rosplan_knowledge_msgs::KnowledgeItem condition;
			condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			condition.attribute_name = cit->name;
			condition.is_negative = negative;

			// populate parameters
			for(int i=0; i<cit->typed_parameters.size(); i++) {

				// set parameter label to predicate label
				diagnostic_msgs::KeyValue param;
				param.key = cit->typed_parameters[i].key;

				// search for correct operator parameter value
				for(int j=0; j<msg.parameters.size() && j<paramList.size(); j++) {
					if(paramList[j].key == cit->typed_parameters[i].key) {
						param.value = msg.parameters[j].value;
					}
				}
				condition.values.push_back(param);
			}
			querySrv.request.knowledge.push_back(condition);
		}

		// check conditions in knowledge base
		if (query_knowledge_client.call(querySrv)) {

			return querySrv.response.all_true;

		} else {
			ROS_ERROR("KCL: (%s) Failed to call service /kcl_rosplan/query_knowledge_base", ros::this_node::getName().c_str());
		}
		return false;
	}

	/*-----------------*/
	/* action feedback */
	/*-----------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void RPExplainer::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		// create error if the action is unrecognised
		ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());
		if(current_action != (unsigned int)msg->action_id)
			ROS_WARN("KCL: (%s) Unexpected action ID: %d; current action: %d", ros::this_node::getName().c_str(), msg->action_id, current_action);

		// action enabled
		if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled")))
			action_received[msg->action_id] = true;

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved"))
			action_completed[msg->action_id] = true;

		// action completed (failed)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed"))
			action_completed[msg->action_id] = true;
	}

	/*---------------*/
	/* plan dispatch */
	/*---------------*/

	/**
	 * Move execution to the chosen action.
	 */
	bool RPExplainer::moveExecution(explainable_planning_msgs::IntegerChoice::Request &req, explainable_planning_msgs::IntegerChoice::Response &res) {

		if(!plan_recieved) {
			ROS_WARN("KCL: (%s) No valid plan has been recieved.", ros::this_node::getName().c_str());
			return false;
		}

		ROS_INFO("KCL: (%s) Moving execution to action %d.", ros::this_node::getName().c_str(), req.option);

		while (ros::ok() && current_plan.plan.size() > current_action && current_action < req.option) {

			// dispatch next action
			dispatchAction(current_plan.plan[current_action]);

			// get ready for next action
			current_action++;
			action_received[current_action] = false;
			action_completed[current_action] = false;
		}

		return true;
	}

	/*-------------------*/
	/* Plan subscription */
	/*-------------------*/

	void RPExplainer::planCallback(const rosplan_dispatch_msgs::CompletePlan plan) {
		ROS_INFO("KCL: (%s) Plan recieved.", ros::this_node::getName().c_str());
		plan_recieved = true;
		current_plan = plan;
		current_action = 0;
		action_received.clear();
		action_completed.clear();
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"rosplan_plan_explainer");
		ros::NodeHandle nh("~");

		KCL_rosplan::RPExplainer ex(nh);
	
		// plan subscription
		std::string planTopic = "complete_plan";
		nh.getParam("plan_topic", planTopic);
		ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::RPExplainer::planCallback, &ex);

		// action feedback subscription
		std::string feedbackTopic = "action_feedback";
		nh.getParam("action_feedback_topic", feedbackTopic);
		ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1, &KCL_rosplan::RPExplainer::feedbackCallback, &ex);

		// start the plan parsing services
		ros::ServiceServer service1 = nh.advertiseService("move_execution", &KCL_rosplan::RPExplainer::moveExecution, dynamic_cast<KCL_rosplan::RPExplainer*>(&ex));
		ros::ServiceServer service2 = nh.advertiseService("get_actions", &KCL_rosplan::RPExplainer::generateApplicableActions, dynamic_cast<KCL_rosplan::RPExplainer*>(&ex));
		ros::ServiceServer service3 = nh.advertiseService("apply_action", &KCL_rosplan::RPExplainer::executeApplicableAction, dynamic_cast<KCL_rosplan::RPExplainer*>(&ex));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
