#include "GraspPolicyFactory.hpp"


std::vector<std::string> GraspPolicyFactory::getAvailablePolicies()
{

}

GraspPolicyPtr GraspPolicyFactory::makePolicy(
		const std::string& id,
		dynamics::DynamicWorkcell* dwc,
		rw::models::JointDevice* dev)
{

}
