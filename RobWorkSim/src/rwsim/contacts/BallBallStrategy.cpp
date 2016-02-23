/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "BallBallStrategy.hpp"
#include "ContactStrategyTracking.hpp"

#include <rw/geometry/Sphere.hpp>

using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;
using namespace rwsim::contacts;

class BallBallStrategy::BallTracking: public ContactStrategyTracking::StrategyData {
public:
	BallTracking() {};
	virtual ~BallTracking() {};

	virtual const ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const {
		RW_ASSERT(index < userData.size());
		return userData[index];
	}

	virtual void setUserData(std::size_t index, const ContactStrategyTracking::UserData::Ptr data) {
		RW_ASSERT(index < userData.size());
		userData[index] = data;
	}

	virtual void remove(std::size_t index) {
		RW_ASSERT(index < userData.size());
		userData.erase(userData.begin()+index);
		modelIDs.erase(modelIDs.begin()+index);
	}

	virtual StrategyData* copy() const {
		BallTracking* tracking = new BallTracking();
		tracking->modelIDs = modelIDs;
		tracking->userData = userData;
		return tracking;
	}

	virtual std::size_t getSize() const {
		return userData.size();
	}

	bool find(std::size_t a, std::size_t b, std::size_t& res) const {
		bool found = false;
		for (std::size_t i = 0; i < modelIDs.size(); i++) {
			if (modelIDs[i].first == a && modelIDs[i].second == b) {
				if (found) {
					RW_THROW("Tracking information between a pair of balls was stored more than once, but there can only be one contact between such a pair!");
				}
				res = i;
				found = true;
			}
		}
		return found;
	}

public:
	std::vector<std::pair<std::size_t, std::size_t> > modelIDs;
	std::vector<ContactStrategyTracking::UserData::Ptr> userData;
};

BallBallStrategy::BallBallStrategy()
{
}

BallBallStrategy::~BallBallStrategy()
{
}

bool BallBallStrategy::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	if (geoA->getType() == GeometryData::SpherePrim && geoB->getType() == GeometryData::SpherePrim)
		return true;
	return false;
}

bool BallBallStrategy::findContact(Contact &c,
	const Sphere* a, const Vector3D<>& wPa,
	const Sphere* b, const Vector3D<>& wPb, bool distCheck) const
{
	const Vector3D<> cVec = wPb-wPa;
	double cVecLen = cVec.norm2();
	if (!distCheck || cVecLen <= a->getRadius() + b->getRadius()) {
		const Vector3D<> cVecNorm = cVec/cVecLen;
		c.setPointA(wPa+a->getRadius()*cVecNorm);
		c.setPointB(wPb-b->getRadius()*cVecNorm);
		c.setNormal(cVecNorm);
		c.setDepth(a->getRadius() + b->getRadius() - cVecLen);
		return true;
	}
	return false;
}

std::vector<Contact> BallBallStrategy::findContacts(
	ProximityModel::Ptr a, const Transform3D<>& wTa,
	ProximityModel::Ptr b, const Transform3D<>& wTb,
	ContactStrategyData& data,
	ContactStrategyTracking& tracking,
	rwsim::log::SimulatorLogScope* log) const
{
	std::vector<Contact> res;
	const GeometryModel::Ptr mA = a.cast<GeometryModel>();
	const GeometryModel::Ptr mB = b.cast<GeometryModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new BallTracking());
	BallTracking* const ballTracking = dynamic_cast<BallTracking*>(tracking.getStrategyData());
	RW_ASSERT(ballTracking);
	std::vector<std::pair<std::size_t,std::size_t> > newModels;
	std::vector<ContactStrategyTracking::UserData::Ptr> newUserData;
	for (std::size_t i = 0; i < mA->models.size(); i++) {
		for (std::size_t j = 0; j < mB->models.size(); j++) {
			std::size_t oldId;
			const bool oldContact = ballTracking->find(i,j,oldId);
			const GeometryModel::Type modelA = mA->models[i];
			const GeometryModel::Type modelB = mB->models[j];
			const Transform3D<> poseA = wTa*modelA.transform;
			const Transform3D<> poseB = wTb*modelB.transform;
			Contact c;
			if (findContact(c, modelA.geo, poseA.P(), modelB.geo, poseB.P(), !oldContact)) {
				c.setTransform(inverse(poseA)*poseB);
				c.setModelA(mA);
				c.setModelB(mB);
				c.setFrameA(modelA.frame);
				c.setFrameB(modelB.frame);
				res.push_back(c);
				if (oldContact) {
					newModels.push_back(ballTracking->modelIDs[oldId]);
					newUserData.push_back(ballTracking->userData[oldId]);
				} else {
					newModels.push_back(std::make_pair(i,j));
					newUserData.push_back(NULL);
				}
			}
		}
	}
	ballTracking->modelIDs = newModels;
	ballTracking->userData = newUserData;
	return res;
}

std::vector<Contact> BallBallStrategy::updateContacts(
	ProximityModel::Ptr a, const Transform3D<>& wTa,
	ProximityModel::Ptr b, const Transform3D<>& wTb,
	ContactStrategyData& data,
	ContactStrategyTracking& tracking,
	rwsim::log::SimulatorLogScope* log) const
{
	std::vector<Contact> res;
	const GeometryModel::Ptr mA = a.cast<GeometryModel>();
	const GeometryModel::Ptr mB = b.cast<GeometryModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new BallTracking());
	BallTracking* const ballTracking = dynamic_cast<BallTracking*>(tracking.getStrategyData());
	RW_ASSERT(ballTracking);
	for (std::size_t i = 0; i < ballTracking->modelIDs.size(); i++) {
		const std::pair<std::size_t, std::size_t>& modelIDs = ballTracking->modelIDs[i];
		const GeometryModel::Type modelA = mA->models[modelIDs.first];
		const GeometryModel::Type modelB = mB->models[modelIDs.second];
		const Transform3D<> poseA = wTa*modelA.transform;
		const Transform3D<> poseB = wTb*modelB.transform;
		Contact c;
		RW_ASSERT(findContact(c, modelA.geo, poseA.P(), modelB.geo, poseB.P(), false));
		c.setTransform(inverse(poseA)*poseB);
		c.setModelA(mA);
		c.setModelB(mB);
		c.setFrameA(modelA.frame);
		c.setFrameB(modelB.frame);
		res.push_back(c);
	}
	return res;
}

std::string BallBallStrategy::getName() {
	return "BallBallStrategy";
}
