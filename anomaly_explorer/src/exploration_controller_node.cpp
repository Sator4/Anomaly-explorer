// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <memory>
#include <string>

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "shared_interfaces/msg/exploration_status.hpp"

using shared_interfaces::msg::ExplorationStatus;

class ExplorationController : public rclcpp::Node
{
public:
	ExplorationController()
	: rclcpp::Node("exploration_controller"), state_(STARTING)
	{
	}

	void init()
	{
		domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
		planner_client_ = std::make_shared<plansys2::PlannerClient>();
		problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
		executor_client_ = std::make_shared<plansys2::ExecutorClient>();

		
		// auto exploration_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void {
		// 	RCLCPP_INFO(this->get_logger(), "%s", (msg->data).c_str());
		// };
		// subscription_ = this->create_subscription<std_msgs::msg::String>("/exploration_status", 10, exploration_callback);

		init_knowledge();
	}

	void init_knowledge()
	{
	    problem_expert_->addInstance(plansys2::Instance{"ar", "area"});
	    problem_expert_->addInstance(plansys2::Instance{"an_pos", "position"});

	    // problem_expert_->addPredicate(plansys2::Predicate("(and(not(area_explored ar)))"));

	}

	void step()
	{
	switch (state_) {
		case STARTING:
			{
				// Set the goal for next state
				problem_expert_->setGoal(plansys2::Goal("(and(area_explored ar))"));

				// Compute the plan
				auto domain = domain_expert_->getDomain();
				auto problem = problem_expert_->getProblem();
				auto plan = planner_client_->getPlan(domain, problem);

				if (!plan.has_value()) {
					std::cout << "Could not find plan to reach goal " <<
						parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
					break;
				}

				// Execute the plan
				if (executor_client_->start_plan_execution(plan.value())) {
					state_ = EXPLORE;
				}
			}
			break;
		case EXPLORE:
			{
				executor_client_->execute_and_check_plan();
				auto feedback = executor_client_->getFeedBack();

				for (const auto & feedback : feedback.action_execution_status) {
					std::cout << feedback.action << " " << feedback.completion * 100.0 << "% | ";
					if (feedback.action == "explore"){
						if (feedback.message_status == ExplorationStatus::ANOMALY_FOUND){
							std::cout << "Anomaly found" << std::endl;
							problem_expert_->addPredicate(plansys2::Predicate("(anomaly_detected an_pos)"));
							executor_client_->cancel_plan_execution();
							auto plan = planner_client_->getPlan(domain_expert_->getDomain(), problem_expert_->getProblem());
							executor_client_->start_plan_execution(plan.value());
							state_ = INVESTIGATE;
						}
					}
				}
				std::cout << std::endl;
			}
			break;
		case INVESTIGATE:
			{
				executor_client_->execute_and_check_plan();
				auto feedback = executor_client_->getFeedBack();

				for (const auto & feedback : feedback.action_execution_status) {
					std::cout << feedback.action << " " << feedback.completion * 100.0 << "% | ";
					if (feedback.action == "investigate"){
						if (feedback.message_status == ExplorationStatus::FINISHED_INVESTIGATING){
							std::cout << "Investigation finished" << std::endl;
							auto plan = planner_client_->getPlan(domain_expert_->getDomain(), problem_expert_->getProblem());
							executor_client_->start_plan_execution(plan.value());
							state_ = EXPLORE; // переделать, чтобы значение изменялось в зависимости от текущего действия
						}
					}
				}
				std::cout << std::endl;
			}
			break;
		default:
			break;
		}
	}

private:
	typedef enum {STARTING, EXPLORE, INVESTIGATE} StateType;
	// rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	StateType state_;

	std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
	std::shared_ptr<plansys2::PlannerClient> planner_client_;
	std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
	std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{

	rclcpp::init(argc, argv);
	
	auto node = std::make_shared<ExplorationController>();
	
	node->init();
	
	rclcpp::Rate rate(5);
	while (rclcpp::ok()) {
		node->step();
		
		rate.sleep();
		rclcpp::spin_some(node->get_node_base_interface());
	}

	node.reset();
	rclcpp::shutdown();

	return 0;
}
