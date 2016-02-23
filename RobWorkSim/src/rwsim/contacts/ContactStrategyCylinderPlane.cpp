/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ContactStrategyCylinderPlane.hpp"
#include "GeometricUtil.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Tube.hpp>
#include <rwsim/contacts/ContactStrategyTracking.hpp>
#include <rwsim/contacts/ContactModelGeometry.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::contacts;

struct ContactStrategyCylinderPlane::TrackInfo {
	TrackInfo(): modelIDa(0), modelIDb(0), cylInGeoA(false), userData(NULL), location(BOTH) {}
	std::size_t modelIDa;
	std::size_t modelIDb;
	bool cylInGeoA;
	ContactStrategyTracking::UserData::Ptr userData;
	Location location;
	Vector3D<> dCyl; // relative to plane
};

class ContactStrategyCylinderPlane::TrackingCylinderPlane: public ContactStrategyTracking::StrategyData {
public:
	TrackingCylinderPlane() {};
	virtual ~TrackingCylinderPlane() {};

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
		TrackingCylinderPlane* tracking = new TrackingCylinderPlane();
		tracking->info = info;
		return tracking;
	}

	virtual std::size_t getSize() const {
		return info.size();
	}

	bool find(bool cylInGeoA, std::size_t a, std::size_t b, std::vector<TrackInfo>& res) const {
		res.clear();
		for (std::size_t i = 0; i < info.size(); i++) {
			if (info[i].cylInGeoA == cylInGeoA && info[i].modelIDa == a && info[i].modelIDb == b) {
				res.push_back(info[i]);
			}
		}
		return res.size() > 0;
	}

public:
	std::vector<TrackInfo> info;
};

class ContactStrategyCylinderPlane::CylinderData: public GeometryModel::GeometryModel<Cylinder*>::CustomData {
public:
	typedef rw::common::Ptr<const CylinderData> Ptr;
	CylinderData(Cylinder* cyl)
	{
		const double r = cyl->getRadius();
		const double l = cyl->getHeight();
		halfLength = l/2.;
		maxDistFromCenter = sqrt(r*r+halfLength*halfLength);
	}
	CylinderData(Tube* tube)
	{
		const double r = tube->getInnerRadius();
		const double l = tube->getHeight();
		halfLength = l/2.;
		maxDistFromCenter = sqrt(r*r+halfLength*halfLength);
	}
	virtual ~CylinderData() {};
	double halfLength;
	double maxDistFromCenter;
};

ContactStrategyCylinderPlane::ContactStrategyCylinderPlane() {
}

ContactStrategyCylinderPlane::~ContactStrategyCylinderPlane() {
}

bool ContactStrategyCylinderPlane::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::CylinderPrim)
		return true;
	else if (geoA->getType() == GeometryData::CylinderPrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	return false;
}

std::vector<Contact> ContactStrategyCylinderPlane::findContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	const GeometryModel::Ptr mA = a.cast<GeometryModel>();
	const GeometryModel::Ptr mB = b.cast<GeometryModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new TrackingCylinderPlane());
	TrackingCylinderPlane* const cpTracking = dynamic_cast<TrackingCylinderPlane*>(tracking.getStrategyData());
	RW_ASSERT(cpTracking);

	std::vector<Contact> contacts;
	std::vector<TrackInfo> newInfo;

	for (std::size_t idA = 0; idA < mA->modelsA.size(); idA++) {
		for (std::size_t idB = 0; idB < mB->modelsB.size(); idB++) {
			GeometryModel::TypeA cylinder = mA->modelsA[idA];
			GeometryModel::TypeB plane = mB->modelsB[idB];
			if (cylinder.customData == NULL)
				cylinder.customData = ownedPtr(new CylinderData(cylinder.geo));
			const CylinderData::Ptr cylData = cylinder.customData.cast<CylinderData>();
			RW_ASSERT(!(cylData == NULL));
			const Transform3D<> pegPose = wTa*cylinder.transform;
			const Transform3D<> planePose = wTb*plane.transform;
			std::vector<TrackInfo> tinfos;
			cpTracking->find(true,idA,idB,tinfos);
			std::vector<Contact> cs = updateContactsCase(cylinder.geo,pegPose,plane.geo,planePose,cylData,tinfos,true);
			BOOST_FOREACH(Contact& c, cs) {
				c.setTransform(inverse(pegPose)*planePose);
				c.setModelA(mA.get());
				c.setModelB(mB.get());
				c.setFrameA(cylinder.frame);
				c.setFrameB(plane.frame);
				contacts.push_back(c);
			}
			BOOST_FOREACH(const TrackInfo& tinfo, tinfos) {
				newInfo.push_back(tinfo);
				newInfo.back().cylInGeoA = true;
				newInfo.back().modelIDa = idA;
				newInfo.back().modelIDb = idB;
				newInfo.back().dCyl = inverse(planePose.R())*pegPose.R().getCol(2);
			}
		}
	}
	for (std::size_t idA = 0; idA < mB->modelsA.size(); idA++) {
		for (std::size_t idB = 0; idB < mA->modelsB.size(); idB++) {
			GeometryModel::TypeA cylinder = mB->modelsA[idA];
			GeometryModel::TypeB plane = mA->modelsB[idB];
			if (cylinder.customData == NULL)
				cylinder.customData = ownedPtr(new CylinderData(cylinder.geo));
			const CylinderData::Ptr cylData = cylinder.customData.cast<CylinderData>();
			RW_ASSERT(!(cylData == NULL));
			const Transform3D<> pegPose = wTb*cylinder.transform;
			const Transform3D<> planePose = wTa*plane.transform;
			std::vector<TrackInfo> tinfos;
			cpTracking->find(false,idA,idB,tinfos);
			std::vector<Contact> cs = updateContactsCase(cylinder.geo,pegPose,plane.geo,planePose,cylData,tinfos,true);
			BOOST_FOREACH(Contact& c, cs) {
				c.setTransform(inverse(pegPose)*planePose);
				c.setModelA(mA.get());
				c.setModelB(mB.get());
				c.setFrameA(cylinder.frame);
				c.setFrameB(plane.frame);
				contacts.push_back(c);
			}
			BOOST_FOREACH(const TrackInfo& tinfo, tinfos) {
				newInfo.push_back(tinfo);
				newInfo.back().cylInGeoA = false;
				newInfo.back().modelIDa = idA;
				newInfo.back().modelIDb = idB;
				newInfo.back().dCyl = inverse(planePose.R())*pegPose.R().getCol(2);
			}
		}
	}

	cpTracking->info = newInfo;
	RW_ASSERT(contacts.size() == cpTracking->info.size());
	return contacts;
}

std::vector<Contact> ContactStrategyCylinderPlane::updateContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	const GeometryModel::Ptr mA = a.cast<GeometryModel>();
	const GeometryModel::Ptr mB = b.cast<GeometryModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	RW_ASSERT(mA->modelsA.size() > 0 || mA->modelsB.size() > 0);
	RW_ASSERT(mB->modelsA.size() > 0 || mB->modelsB.size() > 0);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new TrackingCylinderPlane());
	TrackingCylinderPlane* const cpTracking = dynamic_cast<TrackingCylinderPlane*>(tracking.getStrategyData());
	RW_ASSERT(cpTracking);

	std::vector<Contact> contacts;
	std::vector<TrackInfo> newInfo;
	std::vector<TrackInfo>::iterator it;
	for (it = cpTracking->info.begin(); it != cpTracking->info.end(); it++) {
		const TrackInfo& info = *it;
		const std::size_t modelIDa = info.modelIDa;
		const std::size_t modelIDb = info.modelIDb;
		const bool cylInGeoA = info.cylInGeoA;
		std::vector<TrackInfo> tinfos;
		while (it->cylInGeoA == info.cylInGeoA && it->modelIDa == info.modelIDa && it->modelIDb == info.modelIDb) {
			tinfos.push_back(*it);
			it++;
			if (it == cpTracking->info.end())
				break;
		}
		it--;
		GeometryModel::TypeA cylinder;
		GeometryModel::TypeB plane;
		Transform3D<> cylinderPose;
		Transform3D<> planePose;
		if (cylInGeoA) {
			RW_ASSERT(mA->modelsA.size() > 0);
			RW_ASSERT(mB->modelsB.size() > 0);
			cylinder = mA->modelsA[modelIDa];
			plane = mB->modelsB[modelIDb];
			cylinderPose = wTa*cylinder.transform;
			planePose = wTb*plane.transform;
		} else {
			RW_ASSERT(mB->modelsA.size() > 0);
			RW_ASSERT(mA->modelsB.size() > 0);
			cylinder = mB->modelsA[modelIDa];
			plane = mA->modelsB[modelIDb];
			cylinderPose = wTb*cylinder.transform;
			planePose = wTa*plane.transform;
		}
		if (cylinder.customData == NULL)
			cylinder.customData = ownedPtr(new CylinderData(cylinder.geo));
		const CylinderData::Ptr cylData = cylinder.customData.cast<CylinderData>();
		std::vector<Contact> cs = updateContactsCase(cylinder.geo,cylinderPose,plane.geo,planePose,cylData, tinfos, false);
		RW_ASSERT(cs.size() == tinfos.size());
		BOOST_FOREACH(Contact& c, cs) {
			c.setModelA(mA.get());
			c.setModelB(mB.get());
			c.setFrameA(cylinder.frame);
			c.setFrameB(plane.frame);
			c.setTransform(inverse(cylinderPose)*planePose);
			contacts.push_back(c);
		}
		BOOST_FOREACH(const TrackInfo& tinfo, tinfos) {
			newInfo.push_back(tinfo);
			newInfo.back().modelIDa = modelIDa;
			newInfo.back().modelIDb = modelIDb;
			newInfo.back().cylInGeoA = cylInGeoA;
			newInfo.back().dCyl = inverse(planePose.R())*cylinderPose.R().getCol(2);
		}
	}
	cpTracking->info = newInfo;
	RW_ASSERT(contacts.size() == cpTracking->info.size());

	return contacts;
}

std::string ContactStrategyCylinderPlane::getName() {
	return "ContactStrategyCylinderPlane";
}

std::vector<Contact> ContactStrategyCylinderPlane::getNearest(
		rw::common::Ptr<rw::geometry::Cylinder> cylinder, Transform3D<> cylPose,
		rw::common::Ptr<rw::geometry::Plane> plane, Transform3D<> planePose,
		Location end)
{
	CylinderData::Ptr data = ownedPtr(new CylinderData(cylinder.get()));
	return getNearest(cylinder,cylPose,plane,planePose,data,end);
}

std::vector<Contact> ContactStrategyCylinderPlane::sampleEnd(
		rw::common::Ptr<rw::geometry::GeometryData> cylinderOrTube, Transform3D<> pose,
		rw::common::Ptr<rw::geometry::Plane> plane, Transform3D<> planePose,
		Location end, unsigned int samples)
{
	CylinderData::Ptr data = NULL;
	if (const rw::common::Ptr<Cylinder> cylinder = cylinderOrTube.cast<Cylinder>()) {
		data = ownedPtr(new CylinderData(cylinder.get()));
	} else if (const rw::common::Ptr<Tube> tube = cylinderOrTube.cast<Tube>()) {
		data = ownedPtr(new CylinderData(tube.get()));
	} else {
		RW_THROW("ContactStrategyCylinderPlane (sampleEnd): only works on cylinder or tube!");
		return std::vector<Contact>();
	}
	return sampleEnd(cylinderOrTube,pose,plane,planePose,data,end,samples);
}

ContactStrategyCylinderPlane::Case ContactStrategyCylinderPlane::getCurrentCase(const std::vector<TrackInfo>& tinfos) {
	if (tinfos.size() == 0) {
		return NONE;
	} else if (tinfos.size() == 1) {
		if (tinfos[0].location == TOP)
			return TEDGE;
		else if (tinfos[0].location == BOTTOM)
			return BEDGE;
	} else if (tinfos.size() == 2) {
		if (tinfos[0].location == TOP && tinfos[1].location == BOTTOM)
			return SURFACE;
		else if (tinfos[0].location == BOTTOM && tinfos[1].location == TOP)
			return SURFACE;
		else if (tinfos[0].location == TOP && tinfos[1].location == TOP)
			return _TEND2;
		else if (tinfos[0].location == BOTTOM && tinfos[1].location == BOTTOM)
			return _BEND2;
	} else if (tinfos.size() == 3) {
		if (tinfos[0].location == TOP && tinfos[1].location == TOP && tinfos[2].location == TOP)
			return TEND;
		else if (tinfos[0].location == BOTTOM && tinfos[1].location == BOTTOM && tinfos[2].location == BOTTOM)
			return BEND;
	}
	RW_THROW("ContactStrategyCylinderPlane (getCurrentCase): invalid contact combination.");
	return NONE;
}

/*
std::vector<ContactStrategyCylinderPlane::MetaContact> ContactStrategyCylinderPlane::updateContacts(rw::common::Ptr<Cylinder> cylinder, Transform3D<> cylPose, rw::common::Ptr<Plane> plane, Transform3D<> planePose, rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew) {
	std::vector<MetaContact> metaContacts;

	if (!findNew && tinfos.size() == 0)
		return metaContacts;

	// Put the infos in two bins according to cylinder end
	std::vector<TrackInfo> top;
	std::vector<TrackInfo> bottom;
	BOOST_FOREACH(const TrackInfo& info, tinfos) {
		RW_ASSERT(info.location != BOTH);
		if (info.location == TOP)
			top.push_back(info);
		else if (info.location == BOTTOM)
			bottom.push_back(info);
	}
	tinfos.clear();

	RW_ASSERT(top.size() <= 3);
	RW_ASSERT(bottom.size() <= 3);

	// Find the nearest points at each end and add them as contacts
	const Vector3D<> pPlane = planePose.P();
	const Vector3D<> dPlane = planePose.R().getCol(2);
	const Vector3D<> pCyl = cylPose.P();
	const Vector3D<> dCyl = cylPose.R().getCol(2);
	const Vector3D<> chCross = normalize(cross(cross(dPlane,dCyl),dCyl));
	if (top.size() > 0 || findNew) {
		RW_ASSERT(top.size() == 1 || bottom.size() == 0);
		if (bottom.size() == 0) {
			// There is a possibility that the cylinder is parallel to the plane
			Vector3D<> dirOld = top[0].dCyl;
			Vector3D<> dirCur = inverse(planePose.R())*dCyl;
			dirOld[2] = 0;
			dirCur[2] = 0;
			if (dot(dirOld,dirCur) < 0) {
				RW_THROW("opposite at top");
			}
		}
		const Vector3D<> c = pCyl+dCyl*cylData->halfLength;
		const Vector3D<> cylPoint = c+cylinder->getRadius()*chCross;
		const Vector3D<> planePoint = GeometricUtil::projectPointOnPlane(cylPoint,pPlane,dPlane);
		metaContacts.resize(metaContacts.size()+1);
		metaContacts.back().second = TOP;
		Contact& contact = metaContacts.back().first;
		contact.setPointA(cylPoint);
		contact.setPointB(planePoint);
		contact.setNormal(-dPlane);
		contact.setDepth();
		if (top.size() == 1)
			tinfos.push_back(top[0]);
		else {
			if (contact.getDepth() < 0)
				metaContacts.clear();
			else {
				TrackInfo newInfo;
				newInfo.location = TOP;
				tinfos.push_back(newInfo);
			}
		}
	}
	if (bottom.size() > 0 || findNew) {
		if (bottom.size() == 1) {
			std::vector<Vector3D<> > points;
			if (top.size() == 0) {
				// There is a possibility that the cylinder is parallel to the plane
				Vector3D<> dirOld = bottom[0].dCyl;
				Vector3D<> dirCur = inverse(planePose.R())*dCyl;
				dirOld[2] = 0;
				dirCur[2] = 0;
				if (dot(dirOld,dirCur) < 0) {
					const Vector3D<> c = pCyl-dCyl*cylData->halfLength;
					const Vector3D<> cylPoint = c-cylinder->getRadius()*chCross;
					points.push_back(cylPoint);
				}
			}
			const Vector3D<> c = pCyl-dCyl*cylData->halfLength;
			const Vector3D<> cylPoint = c+cylinder->getRadius()*chCross;
			const Vector3D<> planePoint = GeometricUtil::projectPointOnPlane(cylPoint,pPlane,dPlane);
			metaContacts.resize(metaContacts.size()+1);
			metaContacts.back().second = BOTTOM;
			Contact& contact = metaContacts.back().first;
			contact.setPointA(cylPoint);
			contact.setPointB(planePoint);
			contact.setNormal(-dPlane);
			contact.setDepth();
			if (bottom.size() == 1)
				tinfos.push_back(bottom[0]);
			else {
				if (contact.getDepth() < 0)
					metaContacts.resize(metaContacts.size()-1);
				else {
					TrackInfo newInfo;
					newInfo.location = BOTTOM;
					tinfos.push_back(newInfo);
				}
			}
		} else if (bottom.size() == 2) {
			RW_ASSERT(top.size() == 0);
			RW_THROW("2 known contacts not implemented yet");
		} else if (bottom.size() == 3) {
			RW_ASSERT(top.size() == 0);
			RW_THROW("3 known contacts not implemented yet");
		}
	}

	return metaContacts;
}
*/
std::vector<Contact> ContactStrategyCylinderPlane::updateContactsCase(
		rw::common::Ptr<Cylinder> cylinder,	Transform3D<> cylPose,
		rw::common::Ptr<Plane> plane,		Transform3D<> planePose,
		rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew)
{
	std::vector<Contact> contacts;

	if (tinfos.size() == 0 && !findNew)
		return contacts;

	const Case curCase = getCurrentCase(tinfos);
	switch (curCase) {
	case NONE:
	{
		RW_ASSERT(tinfos.size() == 0);
		if (findNew) {
			// First do a broad-phase check
			const Vector3D<> pPlane = planePose.P();
			const Vector3D<> dPlane = planePose.R().getCol(2);
			const Vector3D<> pCyl = cylPose.P();
			if (dot(pCyl-pPlane,dPlane) > cylData->maxDistFromCenter)
				return contacts;

			Case newCase;

			const Vector3D<> dCyl = cylPose.R().getCol(2);
			const double dotDirs = dot(dPlane,dCyl);
			if (dotDirs == 0) {
				newCase = SURFACE;
			} else if (fabs(dotDirs) < 1) {
				if (dotDirs < 0)
					newCase = TEDGE;
				else if (dotDirs > 0)
					newCase = BEDGE;
			} else {
				if (dotDirs < 0)
					newCase = TEND;
				else if (dotDirs > 0)
					newCase = BEND;
			}

			switch(newCase) {
			case BEND:
				return updateContactsBottomEnd(curCase, cylinder,cylPose,plane,planePose,cylData,tinfos,true);
			case BEDGE:
				return updateContactsBottomEdge(cylinder,cylPose,plane,planePose,cylData,tinfos,true);
			case SURFACE:
				return updateContactsSurface(cylinder,cylPose,plane,planePose,cylData,tinfos,true);
			case TEDGE:
				return updateContactsTopEdge(cylinder,cylPose,plane,planePose,cylData,tinfos,true);
			case TEND:
				RW_THROW("Can not go from NONE to TEND yet!");
			default:
				break;
			}
		}
		break;
	}
	case BEND:
	{
		RW_THROW("No BEND yet!");
		break;
	}
	case BEDGE:
	{
		Case newCase;

		const Vector3D<> dPlane = planePose.R().getCol(2);
		const Vector3D<> dCyl = cylPose.R().getCol(2);
		if (dot(dPlane,dCyl) > 0) {
			Vector3D<> dirOld = tinfos[0].dCyl;
			Vector3D<> dirCur = inverse(planePose.R())*dCyl;
			dirOld[2] = 0;
			dirCur[2] = 0;
			if (dot(dirOld,dirCur) < 0)
				newCase = BEND;
			else
				newCase = BEDGE;
		} else {
			newCase = SURFACE;
		}

		switch(newCase) {
		case BEDGE:
			return updateContactsBottomEdge(cylinder,cylPose,plane,planePose,cylData,tinfos);
		case SURFACE:
			return updateContactsSurface(cylinder,cylPose,plane,planePose,cylData,tinfos,findNew);
		case BEND:
			return updateContactsBottomEnd(curCase, cylinder,cylPose,plane,planePose,cylData,tinfos,findNew);
		default:
			break;
		}
		break;
	}
	case SURFACE:
		return updateContactsSurface(cylinder,cylPose,plane,planePose,cylData,tinfos);
	case TEDGE:
	{
		Case newCase;

		const Vector3D<> dPlane = planePose.R().getCol(2);
		const Vector3D<> dCyl = cylPose.R().getCol(2);
		if (dot(dPlane,dCyl) < 0) {
			Vector3D<> dirOld = tinfos[0].dCyl;
			Vector3D<> dirCur = inverse(planePose.R())*dCyl;
			dirOld[2] = 0;
			dirCur[2] = 0;
			if (dot(dirOld,dirCur) < 0)
				newCase = TEND;
			else
				newCase = TEDGE;
		} else {
			newCase = SURFACE;
		}

		switch(newCase) {
		case TEND:
			RW_THROW("Can not go from TEDGE to TEND yet!");
		case TEDGE:
			return updateContactsTopEdge(cylinder,cylPose,plane,planePose,cylData,tinfos);
		case SURFACE:
			return updateContactsSurface(cylinder,cylPose,plane,planePose,cylData,tinfos,findNew);
		default:
			break;
		}
		break;
	}
	case TEND:
	{
		RW_THROW("No TEND yet!");
		break;
	}
	case _BEND2:
	{
		RW_THROW("No _BEND2 yet!");
		break;
	}
	case _TEND2:
	{
		RW_THROW("No _TEND2 yet!");
		break;
	}
	}

	return contacts;
}

std::vector<Contact> ContactStrategyCylinderPlane::updateContactsTopEdge(
		rw::common::Ptr<Cylinder> cylinder,	Transform3D<> cylPose,
		rw::common::Ptr<Plane> plane,		Transform3D<> planePose,
		rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew)
{
	if (tinfos.size() == 0 && findNew) {
		const std::vector<Contact> contacts = getNearest(cylinder, cylPose, plane, planePose, cylData, TOP);
		RW_ASSERT(contacts.size() == 1);
		if (contacts[0].getDepth() > 0) {
			tinfos.resize(1);
			tinfos[0].location = TOP;
			return contacts;
		} else {
			return std::vector<Contact>();
		}
	} else if (tinfos.size() == 1) {
		RW_ASSERT(tinfos[0].location == TOP);
		return getNearest(cylinder, cylPose, plane, planePose, cylData, TOP);
	}
	RW_THROW("ContactStrategyCylinderPlane (updateContactsTopEdge): Error happened!");
	return std::vector<Contact>();
}

std::vector<Contact> ContactStrategyCylinderPlane::updateContactsBottomEdge(
		rw::common::Ptr<Cylinder> cylinder,	Transform3D<> cylPose,
		rw::common::Ptr<Plane> plane,		Transform3D<> planePose,
		rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew)
{
	if (tinfos.size() == 0 && findNew) {
		const std::vector<Contact> contacts = getNearest(cylinder, cylPose, plane, planePose, cylData, BOTTOM);
		RW_ASSERT(contacts.size() == 1);
		if (contacts[0].getDepth() > 0) {
			tinfos.resize(1);
			tinfos[0].location = BOTTOM;
			return contacts;
		} else {
			return std::vector<Contact>();
		}
	} else if (tinfos.size() == 1) {
		RW_ASSERT(tinfos[0].location == BOTTOM);
		return getNearest(cylinder, cylPose, plane, planePose, cylData, BOTTOM);
	}
	RW_THROW("ContactStrategyCylinderPlane (updateContactsBottomEdge): Error happened!");
	return std::vector<Contact>();
}

std::vector<Contact> ContactStrategyCylinderPlane::updateContactsSurface(
		rw::common::Ptr<Cylinder> cylinder,	Transform3D<> cylPose,
		rw::common::Ptr<Plane> plane,		Transform3D<> planePose,
		rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew)
{
	if (tinfos.size() == 0 && findNew) {
		const std::vector<Contact> contacts = getNearest(cylinder, cylPose, plane, planePose, cylData, BOTH);
		std::vector<Contact> penetrating;
		if (contacts[0].getDepth() > 0) {
			tinfos.resize(tinfos.size()+1);
			tinfos.back().location = TOP;
			penetrating.push_back(contacts[0]);
		}
		if (contacts[1].getDepth() > 0) {
			tinfos.resize(tinfos.size()+1);
			tinfos.back().location = BOTTOM;
			penetrating.push_back(contacts[1]);
		}
		return penetrating;
	} else if (tinfos.size() == 1 && !findNew) {
		return getNearest(cylinder, cylPose, plane, planePose, cylData, tinfos[0].location);
	} else if (tinfos.size() == 1 && findNew) {
		const std::vector<Contact> nearest = getNearest(cylinder, cylPose, plane, planePose, cylData, BOTH);
		std::vector<Contact> contacts;
		if (tinfos[0].location == TOP) {
			contacts.push_back(nearest[0]);
			if (nearest[1].getDepth() > 0) {
				TrackInfo info;
				info.location = BOTTOM;
				tinfos.push_back(info);
				contacts.push_back(nearest[1]);
			}
		} else {
			if (nearest[0].getDepth() > 0) {
				TrackInfo info;
				info.location = TOP;
				tinfos.insert(tinfos.begin(),info);
				contacts.push_back(nearest[0]);
			}
			contacts.push_back(nearest[1]);
		}
		return contacts;
	} else if (tinfos.size() == 2) {
		const std::size_t topId = (tinfos[0].location == TOP) ? 0 : 1;
		const std::size_t bottomId = (tinfos[1].location == BOTTOM) ? 1 : 0;
		RW_ASSERT(tinfos[topId].location == TOP);
		RW_ASSERT(tinfos[bottomId].location == BOTTOM);
		const std::vector<Contact> contacts = getNearest(cylinder, cylPose, plane, planePose, cylData, BOTH);
		RW_ASSERT(contacts.size() == 2);
		std::vector<Contact> orderedContacts(2);
		orderedContacts[topId] = contacts[0];
		orderedContacts[bottomId] = contacts[1];
		return orderedContacts;
	}
	RW_THROW("ContactStrategyCylinderPlane (updateContactsSurface): Error happened!");
	return std::vector<Contact>();
}

std::vector<Contact> ContactStrategyCylinderPlane::updateContactsBottomEnd(Case curCase,
		rw::common::Ptr<Cylinder> cylinder,	Transform3D<> cylPose,
		rw::common::Ptr<Plane> plane,		Transform3D<> planePose,
		rw::common::Ptr<const CylinderData> cylData, std::vector<TrackInfo>& tinfos, bool findNew)
{
	switch(curCase) {
	case NONE:
	{
		RW_ASSERT(tinfos.size() == 0);
		if (findNew) {
			const std::vector<Contact> contacts = sampleEnd(cylinder, cylPose, plane, planePose, cylData, BOTTOM);
			std::vector<Contact> penetrating;
			BOOST_FOREACH(const Contact& contact, contacts) {
				if (contact.getDepth() > 0) {
					tinfos.resize(tinfos.size()+1);
					tinfos.back().location = BOTTOM;
					penetrating.push_back(contact);
				}
			}
			return penetrating;
		}
	}
	break;
	case BEDGE:
	{
		RW_ASSERT(tinfos.size() == 1);
		const std::vector<Contact> contacts = sampleEnd(cylinder, cylPose, plane, planePose, cylData, BOTTOM);
		if (findNew) {
			std::vector<Contact> penetrating;
			BOOST_FOREACH(const Contact& contact, contacts) {
				if (contact.getDepth() > 0) {
					tinfos.resize(tinfos.size()+1);
					tinfos.back().location = BOTTOM;
					penetrating.push_back(contact);
				}
			}
			return penetrating;
		} else {
			std::vector<Contact> penetrating;
			BOOST_FOREACH(const Contact& contact, contacts) {
				if (contact.getDepth() > 0) {
					tinfos.resize(tinfos.size()+1);
					tinfos.back().location = BOTTOM;
					penetrating.push_back(contact);
				}
			}
			return penetrating;
		}
	}
	break;
	case _BEND2:
		break;
	default:
		break;
	}
	if (tinfos.size() == 0 && findNew) {
		const std::vector<Contact> contacts = getNearest(cylinder, cylPose, plane, planePose, cylData, BOTTOM);
		RW_ASSERT(contacts.size() == 1);
		if (contacts[0].getDepth() > 0) {
			tinfos.resize(1);
			tinfos[0].location = BOTTOM;
			return contacts;
		} else {
			return std::vector<Contact>();
		}
	} else if (tinfos.size() == 1) {
		RW_ASSERT(tinfos[0].location == BOTTOM);
		return getNearest(cylinder, cylPose, plane, planePose, cylData, BOTTOM);
	}
	RW_THROW("ContactStrategyCylinderPlane (updateContactsBottomEnd): Error happened!");
	return std::vector<Contact>();
}

std::vector<Contact> ContactStrategyCylinderPlane::getNearest(
		rw::common::Ptr<Cylinder> cylinder,	Transform3D<> cylPose,
		rw::common::Ptr<Plane> plane,		Transform3D<> planePose,
		rw::common::Ptr<const CylinderData> cylData, Location end)
{
	std::vector<Contact> contacts;

	const Vector3D<> pPlane = planePose.P();
	const Vector3D<> dPlane = planePose.R().getCol(2);
	const Vector3D<> pCyl = cylPose.P();
	const Vector3D<> dCyl = cylPose.R().getCol(2);
	const Vector3D<> chCross = normalize(cross(cross(dPlane,dCyl),dCyl));

	if (end == TOP || end == BOTH) {
		const Vector3D<> c = pCyl+dCyl*cylData->halfLength;
		const Vector3D<> cylPoint = c+cylinder->getRadius()*chCross;
		const Vector3D<> planePoint = GeometricUtil::projectPointOnPlane(cylPoint,pPlane,dPlane);
		contacts.resize(contacts.size()+1);
		Contact& contact = contacts.back();
		contact.setPointA(cylPoint);
		contact.setPointB(planePoint);
		contact.setNormal(-dPlane);
		contact.setDepth();
	}
	if (end == BOTTOM || end == BOTH) {
		const Vector3D<> c = pCyl-dCyl*cylData->halfLength;
		const Vector3D<> cylPoint = c+cylinder->getRadius()*chCross;
		const Vector3D<> planePoint = GeometricUtil::projectPointOnPlane(cylPoint,pPlane,dPlane);
		contacts.resize(contacts.size()+1);
		Contact& contact = contacts.back();
		contact.setPointA(cylPoint);
		contact.setPointB(planePoint);
		contact.setNormal(-dPlane);
		contact.setDepth();
	}

	return contacts;
}

std::vector<Contact> ContactStrategyCylinderPlane::sampleEnd(
		rw::common::Ptr<GeometryData> cylinderOrTube,	Transform3D<> cylPose,
		rw::common::Ptr<Plane> plane,					Transform3D<> planePose,
		rw::common::Ptr<const CylinderData> cylData, Location end, const unsigned int samples)
{
	double r = 0;
	if (const rw::common::Ptr<Cylinder> cyl = cylinderOrTube.cast<Cylinder>()) {
		r = cyl->getRadius();
	} else if (const rw::common::Ptr<Tube> tube = cylinderOrTube.cast<Tube>()) {
		r = tube->getInnerRadius();
	} else {
		RW_THROW("ContactStrategyCylinderPlane (sampleEnd): only works on cylinder or tube!");
		return std::vector<Contact>();
	}
	RW_ASSERT(!(cylData == NULL));

	std::vector<Contact> contacts(samples);

	const Vector3D<> pPlane = planePose.P();
	const Vector3D<> dPlane = planePose.R().getCol(2);
	const Vector3D<> pCyl = cylPose.P();
	const Vector3D<> dCyl = cylPose.R().getCol(2);
	const double dotDirs = dot(dPlane,dCyl);

	Vector3D<> chCross;
	if (dotDirs == 0)
		chCross = cylPose.R().getCol(0);
	else
		chCross = normalize(cross(cross(dPlane,dCyl),dCyl));

	Vector3D<> c;
	if (end == TOP)
		c = pCyl+dCyl*cylData->halfLength;
	else if (end == BOTTOM)
		c = pCyl-dCyl*cylData->halfLength;

	for (unsigned int i = 0; i < samples; i++) {
		Vector3D<> cylPoint = c;
		if (i == 0)
			cylPoint += r*chCross;
		else {
			const double angle = (double)i*2.*Pi/((double)samples);
			cylPoint += EAA<>(dCyl,angle).toRotation3D()*(r*chCross);
		}
		const Vector3D<> planePoint = GeometricUtil::projectPointOnPlane(cylPoint,pPlane,dPlane);
		Contact& contact = contacts[i];
		contact.setPointA(cylPoint);
		contact.setPointB(planePoint);
		contact.setNormal(-dPlane);
		contact.setDepth();
	}

	return contacts;
}
