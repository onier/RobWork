/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BallPlaneStrategy.hpp"
#include "ContactStrategyTracking.hpp"

#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Sphere.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;
using namespace rwsim::contacts;

class BallPlaneStrategy::BallPlaneTracking: public ContactStrategyTracking::StrategyData {
public:
	BallPlaneTracking() {};
	virtual ~BallPlaneTracking() {};

	virtual const ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const {
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
		BallPlaneTracking* tracking = new BallPlaneTracking();
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
					RW_THROW("Tracking information between a plane and a ball was stored more than once, but there can only be one contact between such a pair!");
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

BallPlaneStrategy::BallPlaneStrategy()
{
}

BallPlaneStrategy::~BallPlaneStrategy()
{
}

bool BallPlaneStrategy::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	RW_ASSERT(!geoA.isNull());
	RW_ASSERT(!geoB.isNull());
	if (geoA->getType() == GeometryData::SpherePrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::SpherePrim)
		return true;
	return false;
}

bool BallPlaneStrategy::findContact(Contact &c,
	const Sphere* const a,	const Vector3D<>& wPa,
	const Plane* const b,	const Transform3D<>& wTb,
	bool distCheck) const
{
	const Vector3D<> pPlane = wTb.P();
	const Vector3D<> nPlane = wTb.R()*b->normal();
	const double d = dot(wPa-pPlane,nPlane);
	const double r = a->getRadius();
	if (!distCheck || std::fabs(d) <= r) {
		const double sign = Math::sign(d);
		c.setPointA(wPa-sign*r*nPlane);
		c.setPointB(wPa-d*nPlane);
		c.setNormal(-sign*nPlane);
		c.setDepth(r-sign*d);
		c.setTransform(Transform3D<>(-wPa)*wTb);
		return true;
	}
	return false;
}

std::vector<Contact> BallPlaneStrategy::findContacts(
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
		tracking.setStrategyData(new BallPlaneTracking());
	BallPlaneTracking* const ballTracking = dynamic_cast<BallPlaneTracking*>(tracking.getStrategyData());
	RW_ASSERT(ballTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two balls.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two planes.");
	GeometryModel::Ptr balls;
	GeometryModel::Ptr planes;
	Transform3D<> wTball;
	Transform3D<> wTplane;
	if (mA->modelsB.size() > 0) {
		planes = mA;
		balls = mB;
		wTplane = wTa;
		wTball = wTb;
	} else {
		planes = mB;
		balls = mA;
		wTplane = wTb;
		wTball = wTa;
	}

	std::vector<std::pair<std::size_t,std::size_t> > newModels;
	std::vector<ContactStrategyTracking::UserData::Ptr> newUserData;
	for (std::size_t i = 0; i < balls->modelsA.size(); i++) {
		const GeometryModel::TypeA ball = balls->modelsA[i];
		for (std::size_t j = 0; j < planes->modelsB.size(); j++) {
			const GeometryModel::TypeB plane = planes->modelsB[j];
			std::size_t oldId;
			const bool oldContact = ballTracking->find(i,j,oldId);
			Contact c;
			const Transform3D<> ballPose = wTball*ball.transform;
			const Transform3D<> planePose = wTplane*plane.transform;
			if (findContact(c, ball.geo, ballPose.P(), plane.geo, planePose, !oldContact)) {
				c.setTransform(inverse(ballPose)*planePose);
				c.setModelA(balls);
				c.setModelB(planes);
				c.setFrameA(ball.frame);
				c.setFrameB(plane.frame);
				if (!oldContact) {
					res.push_back(c);
					newModels.push_back(std::make_pair(i,j));
					newUserData.push_back(NULL);
				} else {
					//if (c.getDepth() > -1e-3) {
						res.push_back(c);
						newModels.push_back(ballTracking->modelIDs[oldId]);
						newUserData.push_back(ballTracking->userData[oldId]);
					//}
				}
			}
		}
	}
	ballTracking->modelIDs = newModels;
	ballTracking->userData = newUserData;
	return res;
}

std::vector<Contact> BallPlaneStrategy::updateContacts(
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
		tracking.setStrategyData(new BallPlaneTracking());
	BallPlaneTracking* const ballTracking = dynamic_cast<BallPlaneTracking*>(tracking.getStrategyData());
	RW_ASSERT(ballTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two balls.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two planes.");
	GeometryModel::Ptr balls;
	GeometryModel::Ptr planes;
	Transform3D<> wTball;
	Transform3D<> wTplane;
	if (mA->modelsB.size() > 0) {
		planes = mA;
		balls = mB;
		wTplane = wTa;
		wTball = wTb;
	} else {
		planes = mB;
		balls = mA;
		wTplane = wTb;
		wTball = wTa;
	}

	std::vector<std::size_t> remove;
	for (std::size_t i = 0; i < ballTracking->modelIDs.size(); i++) {
		const std::pair<std::size_t, std::size_t>& modelIDs = ballTracking->modelIDs[i];
		const GeometryModel::TypeA ball = balls->modelsA[modelIDs.first];
		const GeometryModel::TypeB plane = planes->modelsB[modelIDs.second];
		Contact c;
		const Transform3D<> ballPose = wTball*ball.transform;
		const Transform3D<> planePose = wTplane*plane.transform;
		if (findContact(c, ball.geo, ballPose.P(), plane.geo, planePose, false)) {
			c.setTransform(inverse(ballPose)*planePose);
			c.setModelA(balls);
			c.setModelB(planes);
			c.setFrameA(ball.frame);
			c.setFrameB(plane.frame);
			//if (c.getDepth() > -1e-3)
				res.push_back(c);
			//else
//				remove.push_back(i);
		}
	}
	for (std::size_t n = 0; n < remove.size(); n++) {
		ballTracking->remove(remove[n]-n);
	}
	return res;
}

std::string BallPlaneStrategy::getName() {
	return "BallPlaneStrategy";
}

