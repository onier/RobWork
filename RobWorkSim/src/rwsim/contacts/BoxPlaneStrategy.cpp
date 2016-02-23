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

#include "BoxPlaneStrategy.hpp"
#include "ContactStrategyTracking.hpp"
#include "GeometricUtil.hpp"

#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Box.hpp>

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::contacts;

namespace {
struct TrackInfo {
	TrackInfo(): modelIDa(0), modelIDb(0), userData(NULL), location(false,false,false) {}
	std::size_t modelIDa;
	std::size_t modelIDb;
	ContactStrategyTracking::UserData::Ptr userData;
	std::tuple<bool,bool,bool> location;

	static std::size_t findLocation(const std::vector<TrackInfo> tinfos, const TrackInfo& info) {
		for (std::size_t i = 0; i < tinfos.size(); i++) {
			if (tinfos[i].location == info.location)
				return i;
		}
		return tinfos.size();
	}
};

class BoxPlaneTracking: public ContactStrategyTracking::StrategyData {
public:
	BoxPlaneTracking() {};
	virtual ~BoxPlaneTracking() {};

	virtual const ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const {
		RW_ASSERT(index < info.size());
		return info[index].userData;
	}

	virtual void setUserData(std::size_t index, const ContactStrategyTracking::UserData::Ptr data) {
		RW_ASSERT(index < info.size());
		info[index].userData = data;
	}

	virtual void remove(std::size_t index) {
		RW_ASSERT(index < info.size());
		info.erase(info.begin()+index);
	}

	virtual StrategyData* copy() const {
		BoxPlaneTracking* tracking = new BoxPlaneTracking();
		tracking->info = info;
		return tracking;
	}

	virtual std::size_t getSize() const {
		return info.size();
	}

	bool find(std::size_t a, std::size_t b, std::vector<TrackInfo>& res) const {
		res.clear();
		for (std::size_t i = 0; i < info.size(); i++) {
			if (info[i].modelIDa == a && info[i].modelIDb == b) {
				res.push_back(info[i]);
			}
		}
		return res.size() > 0;
	}

public:
	std::vector<TrackInfo> info;
};

struct Parameters {
	Parameters(const Box* a, const Transform3D<>& wTa, const Plane* b, const Transform3D<>& wTb):
		boxCenter(wTa.P()),
		size(a->getParameters()/2.),
		n(wTb.R()*b->normal()),
		d(b->d()),
		x(wTa.R().getCol(0)),
		y(wTa.R().getCol(1)),
		z(wTa.R().getCol(2)),
		dotX(dot(x,n)),
		dotY(dot(y,n)),
		dotZ(dot(z,n))
	{
	}
	const Vector3D<> boxCenter;
	const Q size;
	const Vector3D<> n;
	const double d;
	const Vector3D<> x, y, z;
	const double dotX, dotY, dotZ;
};

Contact determineContact(const Parameters& p, const TrackInfo& tinfo) {
	// Determine the direction of x, y and z
	const double sgnX = std::get<0>(tinfo.location) ? +1 : -1;
	const double sgnY = std::get<1>(tinfo.location) ? +1 : -1;
	const double sgnZ = std::get<2>(tinfo.location) ? +1 : -1;

	// Determine the contact
	const Vector3D<> point = p.boxCenter+sgnX*p.size[0]*p.x+sgnY*p.size[1]*p.y+sgnZ*p.size[2]*p.z;
	Contact contact;
	contact.setPointA(point);
	contact.setPointB(GeometricUtil::projectPointOnPlane(point,p.n*p.d,p.n));
	contact.setNormal(-p.n);
	contact.setDepth();
	return contact;
}

bool determineContacts(std::vector<Contact>& contacts,
	const Box* a, const Transform3D<>& wTa,
	const Plane* b, const Transform3D<>& wTb,
	std::vector<TrackInfo>& tinfos, bool findNew)
{
	contacts.clear();

	// Determine geometric parameters for current configuration
	const Parameters p(a, wTa, b, wTb);

	// First update the old contacts
	for (std::size_t i = 0; i < tinfos.size(); i++) {
		const TrackInfo& info = tinfos[i];
		contacts.push_back(determineContact(p, info));
	}

	if (!findNew)
		return contacts.size() > 0;

	// Determine the direction towards the deepest point
	const double sgnX = -Math::sign(p.dotX);
	const double sgnY = -Math::sign(p.dotY);
	const double sgnZ = -Math::sign(p.dotZ);

	std::vector<TrackInfo> deepestFaceInfos;

	// Determine the deepest contact
	{
		TrackInfo info;
		info.location = std::make_tuple(sgnX > 0, sgnY > 0, sgnZ > 0);
		const std::size_t id = TrackInfo::findLocation(tinfos,info);
		if (id == tinfos.size()) {
			// The contact was not already found as a tracked contact - determine it
			const Contact c = determineContact(p,info);

			// Exit early if closest corner is not in contact
			if (c.getDepth() < 0)
				return false;

			// Else add the contact and some tracking info
			contacts.push_back(c);
			tinfos.push_back(info);
			deepestFaceInfos.push_back(info);
		} else {
			// The contact was already tracked - check if we can exit early (if it is not in contact)
			const Contact& c = contacts[id];
			if (c.getDepth() < 0)
				return false;
			deepestFaceInfos.push_back(tinfos[id]);
		}
	}

	// Now the neighbour points are tested
	for (int i = 0; i < 3; i++) {
		TrackInfo info;
		if (i == 0)
			info.location = std::make_tuple(sgnX <= 0, sgnY > 0, sgnZ > 0);
		else if (i == 1)
			info.location = std::make_tuple(sgnX > 0, sgnY <= 0, sgnZ > 0);
		else if (i == 2)
			info.location = std::make_tuple(sgnX > 0, sgnY > 0, sgnZ <= 0);

		const std::size_t id = TrackInfo::findLocation(tinfos,info);
		if (id == tinfos.size()) {
			const Contact c = determineContact(p,info);

			if (c.getDepth() >= 0) {
				contacts.push_back(c);
				tinfos.push_back(info);
				deepestFaceInfos.push_back(info);
			}
		} else {
			deepestFaceInfos.push_back(tinfos[id]);
		}
	}

	// If 3 contacts where found, we have a face in contact and add the last point no matter if it penetrating or not.
	if (deepestFaceInfos.size() == 3) {
		// Exactly one of the following three should be true
		const bool eqX = (std::get<0>(deepestFaceInfos[0].location) == std::get<0>(deepestFaceInfos[1].location)) && (std::get<0>(deepestFaceInfos[1].location) == std::get<0>(deepestFaceInfos[2].location));
		const bool eqY = (std::get<1>(deepestFaceInfos[0].location) == std::get<1>(deepestFaceInfos[1].location)) && (std::get<1>(deepestFaceInfos[1].location) == std::get<1>(deepestFaceInfos[2].location));
		const bool eqZ = (std::get<2>(deepestFaceInfos[0].location) == std::get<2>(deepestFaceInfos[1].location)) && (std::get<2>(deepestFaceInfos[1].location) == std::get<2>(deepestFaceInfos[2].location));

		RW_ASSERT((eqX && !eqY && !eqZ) || (!eqX && eqY && !eqZ) || (!eqX && !eqY && eqZ));

		TrackInfo info;
		if (eqX)
			info.location = std::make_tuple(sgnX > 0, sgnY <= 0, sgnZ <= 0);
		else if (eqY)
			info.location = std::make_tuple(sgnX <= 0, sgnY > 0, sgnZ <= 0);
		else if (eqZ)
			info.location = std::make_tuple(sgnX <= 0, sgnY <= 0, sgnZ > 0);

		if (TrackInfo::findLocation(tinfos,info) == tinfos.size()) {
			const Contact c = determineContact(p,info);

			contacts.push_back(c);
			tinfos.push_back(info);
		}
	}

	return contacts.size() > 0;
}

}

BoxPlaneStrategy::BoxPlaneStrategy() {
}

BoxPlaneStrategy::~BoxPlaneStrategy() {
}

bool BoxPlaneStrategy::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	RW_ASSERT(!geoA.isNull());
	RW_ASSERT(!geoB.isNull());
	if (geoA->getType() == GeometryData::BoxPrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::BoxPrim)
		return true;
	return false;
}

std::vector<Contact> BoxPlaneStrategy::findContacts(
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
		tracking.setStrategyData(new BoxPlaneTracking());
	BoxPlaneTracking* const boxTracking = dynamic_cast<BoxPlaneTracking*>(tracking.getStrategyData());
	RW_ASSERT(boxTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two boxes.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two planes.");
	GeometryModel::Ptr boxes;
	GeometryModel::Ptr planes;
	Transform3D<> wTbox;
	Transform3D<> wTplane;
	if (mA->modelsB.size() > 0) {
		planes = mA;
		boxes = mB;
		wTplane = wTa;
		wTbox = wTb;
	} else {
		planes = mB;
		boxes = mA;
		wTplane = wTb;
		wTbox = wTa;
	}

	std::vector<TrackInfo> newInfo;
	for (std::size_t i = 0; i < boxes->modelsA.size(); i++) {
		const GeometryModel::TypeA box = boxes->modelsA[i];
		for (std::size_t j = 0; j < planes->modelsB.size(); j++) {
			const GeometryModel::TypeB plane = planes->modelsB[j];
			const Transform3D<> boxPose = wTbox*box.transform;
			const Transform3D<> planePose = wTplane*plane.transform;
			std::vector<Contact> contacts;
			std::vector<TrackInfo> info;
			boxTracking->find(i,j,info);
			determineContacts(contacts, box.geo, boxPose, plane.geo, planePose, info, true);
			RW_ASSERT(contacts.size() == info.size());
			BOOST_FOREACH(Contact& c, contacts) {
				c.setTransform(inverse(boxPose)*planePose);
				c.setModelA(boxes);
				c.setModelB(planes);
				c.setFrameA(box.frame);
				c.setFrameB(plane.frame);
				res.push_back(c);
			}
			BOOST_FOREACH(TrackInfo& ti, info) {
				ti.modelIDa = i;
				ti.modelIDb = j;
			}
			newInfo.insert(newInfo.end(),info.begin(),info.end());
		}
	}
	boxTracking->info = newInfo;
	return res;
}

std::vector<Contact> BoxPlaneStrategy::updateContacts(
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
		tracking.setStrategyData(new BoxPlaneTracking());
	const BoxPlaneTracking* const boxTracking = dynamic_cast<BoxPlaneTracking*>(tracking.getStrategyData());
	RW_ASSERT(boxTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two boxes.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two planes.");
	GeometryModel::Ptr boxes;
	GeometryModel::Ptr planes;
	Transform3D<> wTbox;
	Transform3D<> wTplane;
	if (mA->modelsB.size() > 0) {
		planes = mA;
		boxes = mB;
		wTplane = wTa;
		wTbox = wTb;
	} else {
		planes = mB;
		boxes = mA;
		wTplane = wTb;
		wTbox = wTa;
	}

	for (std::size_t i = 0; i < boxes->modelsA.size(); i++) {
		const GeometryModel::TypeA box = boxes->modelsA[i];
		for (std::size_t j = 0; j < planes->modelsB.size(); j++) {
			const GeometryModel::TypeB plane = planes->modelsB[j];
			const Transform3D<> boxPose = wTbox*box.transform;
			const Transform3D<> planePose = wTplane*plane.transform;
			std::vector<TrackInfo> info;
			if (boxTracking->find(i,j,info)) {
				std::vector<Contact> contacts;
				determineContacts(contacts, box.geo, boxPose, plane.geo, planePose, info, false);
				RW_ASSERT(contacts.size() == info.size());
				const Transform3D<> boxTplane = inverse(boxPose)*planePose;
				BOOST_FOREACH(Contact& c, contacts) {
					c.setTransform(boxTplane);
					c.setModelA(boxes);
					c.setModelB(planes);
					c.setFrameA(box.frame);
					c.setFrameB(plane.frame);
					res.push_back(c);
				}
			}
		}
	}
	RW_ASSERT(res.size() == boxTracking->info.size());

	return res;
}

std::string BoxPlaneStrategy::getName() {
	return "BoxPlaneStrategy";
}
