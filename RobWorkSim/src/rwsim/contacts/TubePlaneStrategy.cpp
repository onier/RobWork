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

#include "TubePlaneStrategy.hpp"
#include "ContactStrategyTracking.hpp"
#include "GeometricUtil.hpp"

#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Tube.hpp>

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::contacts;

#define END_SAMPLES 4

namespace {
struct TrackInfo {
	TrackInfo(): modelIDa(0), modelIDb(0), userData(NULL), atTop(false) {}
	std::size_t modelIDa;
	std::size_t modelIDb;
	ContactStrategyTracking::UserData::Ptr userData;
	bool atTop;
	Vector3D<> wPtube;

	static std::vector<std::size_t> findLocation(const std::vector<TrackInfo> tinfos, const TrackInfo& info) {
		std::vector<std::size_t> ids;
		for (std::size_t i = 0; i < tinfos.size(); i++) {
			if (tinfos[i].atTop == info.atTop)
				ids.push_back(i);
		}
		return ids;
	}
};

class TubePlaneTracking: public ContactStrategyTracking::StrategyData {
public:
	TubePlaneTracking() {};
	virtual ~TubePlaneTracking() {};

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
		TubePlaneTracking* tracking = new TubePlaneTracking();
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
	Parameters(const Tube* a, const Transform3D<>& wTa, const Plane* b, const Transform3D<>& wTb):
		tubeCenter(wTa.P()),
		r(a->getInnerRadius()+a->getThickness()),
		h(a->getHeight()),
		hHalf(h/2.),
		dTube(wTa.R().getCol(2)),
		dTubeHhalf(wTa.R().getCol(2)*hHalf),
		n(wTb.R()*b->normal()),
		pPlane(wTb.P()+b->d()*n),
		dotDir(dot(dTube,n)),
		ndCross(cross(n,dTube)),
		chCross(ndCross.norm2() == 0 ? GeometricUtil::getPerpendicular(dTube) : normalize(cross(ndCross,dTube))),
		chCrossR(chCross*r)
	{
	}
	const Vector3D<> tubeCenter;
	const double r;
	const double h;
	const double hHalf;
	const Vector3D<> dTube;
	const Vector3D<> dTubeHhalf;
	const Vector3D<> n;
	const Vector3D<> pPlane;
	const double dotDir;
	const Vector3D<> ndCross;
	const Vector3D<> chCross;
	const Vector3D<> chCrossR;
};

Contact determineContactAtEdge(const Parameters& p, bool atTop) {
	Contact contact;
	Vector3D<> c;
	if (atTop)
		c = p.tubeCenter+p.dTubeHhalf;
	else
		c = p.tubeCenter-p.dTubeHhalf;
	const Vector3D<> cylPoint = c+p.chCrossR;
	const Vector3D<> planePoint = GeometricUtil::projectPointOnPlane(cylPoint,p.pPlane,p.n);
	contact.setPointA(cylPoint);
	contact.setPointB(planePoint);
	contact.setNormal(-p.n);
	contact.setDepth();
	return contact;
}

std::vector<Contact> determineContactsAtEnd(const Parameters& p, bool atTop, const unsigned int samples) {
	std::vector<Contact> contacts(samples-1);

	Vector3D<> c;
	if (atTop)
		c = p.tubeCenter+p.dTubeHhalf;
	else
		c = p.tubeCenter-p.dTubeHhalf;

	for (unsigned int i = 1; i < samples; i++) {
		const double angle = (double)i*2.*Pi/((double)samples);
		const Vector3D<> cylPoint = c + EAA<>(p.dTube,angle).toRotation3D()*(p.chCrossR);
		const Vector3D<> planePoint = GeometricUtil::projectPointOnPlane(cylPoint,p.pPlane,p.n);
		Contact& contact = contacts[i-1];
		contact.setPointA(cylPoint);
		contact.setPointB(planePoint);
		contact.setNormal(-p.n);
		contact.setDepth();
	}

	return contacts;
}

void determineContacts(std::vector<Contact>& contacts,
	const Tube* a, const Transform3D<>& wTa,
	const Plane* b, const Transform3D<>& wTb,
	std::vector<TrackInfo>& tinfos, bool findNew)
{
	contacts.clear();
	std::vector<TrackInfo> newInfo;

	// Determine geometric parameters for current configuration
	const Parameters p(a, wTa, b, wTb);

	// First update the old contacts
	if (tinfos.size() > 0) {
		std::vector<TrackInfo> infoTop;
		std::vector<TrackInfo> infoBottom;
		for (std::size_t i = 0; i < tinfos.size(); i++) {
			const TrackInfo& info = tinfos[i];
			if (info.atTop)
				infoTop.push_back(info);
			else
				infoBottom.push_back(info);
		}
		if (infoTop.size() == 1) {
			contacts.push_back(determineContactAtEdge(p, true));
			newInfo.push_back(infoTop[0]);
		} else if (infoTop.size() > 1) {
			RW_ASSERT(infoTop.size() <= END_SAMPLES);
			contacts.push_back(determineContactAtEdge(p, true));
			std::size_t infosToAdd = END_SAMPLES;
			{
				std::vector<TrackInfo>::iterator it = infoTop.begin();
				std::vector<TrackInfo>::iterator itNear = infoTop.begin();
				for (it++; it != infoTop.end(); it++) {
					if ((it->wPtube - contacts.back().getPointA()).norm2() < (itNear->wPtube - contacts.back().getPointA()).norm2())
						itNear = it;
				}
				newInfo.push_back(*itNear);
				infoTop.erase(itNear);
				infosToAdd--;
			}
			const std::vector<Contact> endContacts = determineContactsAtEnd(p, true, END_SAMPLES);
			contacts.insert(contacts.end(),endContacts.begin(),endContacts.end());
			std::size_t i;
			for (i = 0; i < endContacts.size() && infoTop.size() > 0; i++) {
				std::vector<TrackInfo>::iterator it = infoTop.begin();
				std::vector<TrackInfo>::iterator itNear = infoTop.begin();
				for (it++; it != infoTop.end(); it++) {
					if ((it->wPtube - endContacts[i].getPointA()).norm2() < (itNear->wPtube - endContacts[i].getPointA()).norm2())
						itNear = it;
				}
				newInfo.push_back(*itNear);
				infoTop.erase(itNear);
				infosToAdd--;
			}
			for (std::size_t k = 0; k < infosToAdd; k++) {
				TrackInfo inf;
				inf.atTop = true;
				newInfo.push_back(inf);
			}
		}
		if (infoBottom.size() == 1) {
			contacts.push_back(determineContactAtEdge(p, false));
			newInfo.push_back(infoBottom[0]);
		} else if (infoBottom.size() > 1) {
			RW_ASSERT(infoBottom.size() <= END_SAMPLES);
			contacts.push_back(determineContactAtEdge(p, false));
			std::size_t infosToAdd = END_SAMPLES;
			{
				std::vector<TrackInfo>::iterator it = infoBottom.begin();
				std::vector<TrackInfo>::iterator itNear = infoBottom.begin();
				for (it++; it != infoBottom.end(); it++) {
					if ((it->wPtube - contacts.back().getPointA()).norm2() < (itNear->wPtube - contacts.back().getPointA()).norm2())
						itNear = it;
				}
				newInfo.push_back(*itNear);
				infoBottom.erase(itNear);
				infosToAdd--;
			}
			const std::vector<Contact> endContacts = determineContactsAtEnd(p, false, END_SAMPLES);
			contacts.insert(contacts.end(),endContacts.begin(),endContacts.end());
			std::size_t i;
			for (i = 0; i < endContacts.size() && infoBottom.size() > 0; i++) {
				std::vector<TrackInfo>::iterator it = infoBottom.begin();
				std::vector<TrackInfo>::iterator itNear = infoBottom.begin();
				for (it++; it != infoBottom.end(); it++) {
					if ((it->wPtube - endContacts[i].getPointA()).norm2() < (itNear->wPtube - endContacts[i].getPointA()).norm2())
						itNear = it;
				}
				newInfo.push_back(*itNear);
				infoBottom.erase(itNear);
				infosToAdd--;
			}
			for (std::size_t k = 0; k < infosToAdd; k++) {
				TrackInfo inf;
				inf.atTop = false;
				newInfo.push_back(inf);
			}
		}
	}

	RW_ASSERT(newInfo.size() == contacts.size());
	if (!findNew) {
		tinfos = newInfo;
		return;
	}

	// Determine the deepest contact if at top
	if (p.dotDir <= 0) {
		TrackInfo info;
		info.atTop = true;
		const std::vector<std::size_t> ids = TrackInfo::findLocation(newInfo,info);
		if (ids.size() == 0) {
			// The contact was not already found as a tracked contact - determine it
			const Contact c = determineContactAtEdge(p,info.atTop);

			// Exit early if closest point is not in contact
			if (c.getDepth() < 0) {
				tinfos = newInfo;
				return;
			}

			// Else add the contact and some tracking info
			contacts.push_back(c);
			newInfo.push_back(info);
		} else {
			// The contact was already tracked - check if we can exit early (if it is not in contact)
			bool exit = true;
			BOOST_FOREACH(std::size_t id, ids) {
				const Contact& c = contacts[id];
				if (c.getDepth() >= 0)
					exit = false;
			}
			if (exit) {
				tinfos = newInfo;
				return;
			}
		}
	}
	// Determine the deepest contact if at bottom
	if (p.dotDir >= 0) {
		TrackInfo info;
		info.atTop = false;
		const std::vector<std::size_t> ids = TrackInfo::findLocation(newInfo,info);
		if (ids.size() == 0) {
			// The contact was not already found as a tracked contact - determine it
			const Contact c = determineContactAtEdge(p,info.atTop);

			// Exit early if closest point is not in contact
			if (c.getDepth() < 0) {
				tinfos = newInfo;
				return;
			}

			// Else add the contact and some tracking info
			contacts.push_back(c);
			newInfo.push_back(info);
		} else {
			// The contact was already tracked - check if we can exit early (if it is not in contact)
			bool exit = true;
			BOOST_FOREACH(std::size_t id, ids) {
				const Contact& c = contacts[id];
				if (c.getDepth() >= 0)
					exit = false;
			}
			if (exit) {
				tinfos = newInfo;
				return;
			}
		}
	}

	if (p.dotDir == 0) {
		tinfos = newInfo;
		return;
	}

	// Now the top end points are added if needed
	if (p.dotDir < 0) {
		TrackInfo info;
		info.atTop = true;
		const std::vector<std::size_t> ids = TrackInfo::findLocation(newInfo,info);
		if (ids.size() == 1) {
			const std::vector<Contact> endContacts = determineContactsAtEnd(p, info.atTop, END_SAMPLES);
			bool add = false;
			BOOST_FOREACH(const Contact& c, endContacts) {
				if (c.getDepth() > 0) {
					add = true;
					break;
				}
			}
			if (add) {
				const TrackInfo oldInfo = newInfo[ids[0]];
				contacts.insert(contacts.end(),endContacts.begin(),endContacts.end());
				int minId = -1;
				double minDepth = contacts[ids[0]].getDepth();
				for (std::size_t k = 0; k < END_SAMPLES-1; k++) {
					const double depth = endContacts[k].getDepth();
					if (depth < minDepth) {
						minId = k;
						minDepth = depth;
					}
				}
				if (minId >= 0)
					newInfo[ids[0]].userData = NULL;
				for (std::size_t k = 0; k < END_SAMPLES-1; k++) {
					if (minId == (int) k) {
						newInfo.push_back(oldInfo);
					} else {
						TrackInfo inf;
						inf.atTop = info.atTop;
						newInfo.push_back(inf);
					}
				}
			}
		}
	}

	// Now the bottom end points are added if needed
	if (p.dotDir > 0) {
		TrackInfo info;
		info.atTop = false;
		const std::vector<std::size_t> ids = TrackInfo::findLocation(newInfo,info);
		if (ids.size() == 1) {
			const std::vector<Contact> endContacts = determineContactsAtEnd(p, info.atTop, END_SAMPLES);
			bool add = false;
			BOOST_FOREACH(const Contact& c, endContacts) {
				if (c.getDepth() > 0) {
					add = true;
					break;
				}
			}
			if (add) {
				const TrackInfo oldInfo = newInfo[ids[0]];
				contacts.insert(contacts.end(),endContacts.begin(),endContacts.end());
				int minId = -1;
				double minDepth = contacts[ids[0]].getDepth();
				for (std::size_t k = 0; k < END_SAMPLES-1; k++) {
					const double depth = endContacts[k].getDepth();
					if (depth < minDepth) {
						minId = k;
						minDepth = depth;
					}
				}
				if (minId >= 0)
					newInfo[ids[0]].userData = NULL;
				for (std::size_t k = 0; k < END_SAMPLES-1; k++) {
					if (minId == (int) k) {
						newInfo.push_back(oldInfo);
					} else {
						TrackInfo inf;
						inf.atTop = info.atTop;
						newInfo.push_back(inf);
					}
				}
			}
		}
	}
	tinfos = newInfo;
	RW_ASSERT(tinfos.size() == contacts.size());
}

}

TubePlaneStrategy::TubePlaneStrategy() {
}

TubePlaneStrategy::~TubePlaneStrategy() {
}

bool TubePlaneStrategy::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	RW_ASSERT(!geoA.isNull());
	RW_ASSERT(!geoB.isNull());
	if (geoA->getType() == GeometryData::TubePrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::TubePrim)
		return true;
	return false;
}

std::vector<Contact> TubePlaneStrategy::findContacts(
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
		tracking.setStrategyData(new TubePlaneTracking());
	TubePlaneTracking* const tubeTracking = dynamic_cast<TubePlaneTracking*>(tracking.getStrategyData());
	RW_ASSERT(tubeTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two tubes.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two planes.");
	GeometryModel::Ptr tubes;
	GeometryModel::Ptr planes;
	Transform3D<> wTtube;
	Transform3D<> wTplane;
	if (mA->modelsB.size() > 0) {
		planes = mA;
		tubes = mB;
		wTplane = wTa;
		wTtube = wTb;
	} else {
		planes = mB;
		tubes = mA;
		wTplane = wTb;
		wTtube = wTa;
	}

	std::vector<TrackInfo> newInfo;
	for (std::size_t i = 0; i < tubes->modelsA.size(); i++) {
		const GeometryModel::TypeA tube = tubes->modelsA[i];
		for (std::size_t j = 0; j < planes->modelsB.size(); j++) {
			const GeometryModel::TypeB plane = planes->modelsB[j];
			const Transform3D<> tubePose = wTtube*tube.transform;
			const Transform3D<> planePose = wTplane*plane.transform;
			std::vector<Contact> contacts;
			std::vector<TrackInfo> info;
			tubeTracking->find(i,j,info);
			determineContacts(contacts, tube.geo, tubePose, plane.geo, planePose, info, true);
			RW_ASSERT(contacts.size() == info.size());
			BOOST_FOREACH(Contact& c, contacts) {
				c.setTransform(inverse(tubePose)*planePose);
				c.setModelA(tubes);
				c.setModelB(planes);
				c.setFrameA(tube.frame);
				c.setFrameB(plane.frame);
				res.push_back(c);
			}
			for (std::size_t k = 0; k < info.size(); k++) {
				info[k].modelIDa = i;
				info[k].modelIDb = j;
				info[k].wPtube = contacts[k].getPointA();
			}
			newInfo.insert(newInfo.end(),info.begin(),info.end());
		}
	}
	tubeTracking->info = newInfo;
	return res;
}

std::vector<Contact> TubePlaneStrategy::updateContacts(
	ProximityModel::Ptr a, const Transform3D<>& wTa,
	ProximityModel::Ptr b, const Transform3D<>& wTb,
	ContactStrategyData& data,
	ContactStrategyTracking& tracking,
	rwsim::log::SimulatorLogScope* log) const
{
	std::vector<Contact> res;
	std::vector<TrackInfo> newInfo;
	const GeometryModel::Ptr mA = a.cast<GeometryModel>();
	const GeometryModel::Ptr mB = b.cast<GeometryModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new TubePlaneTracking());
	TubePlaneTracking* const tubeTracking = dynamic_cast<TubePlaneTracking*>(tracking.getStrategyData());
	RW_ASSERT(tubeTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two tubes.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two planes.");
	GeometryModel::Ptr tubes;
	GeometryModel::Ptr planes;
	Transform3D<> wTtube;
	Transform3D<> wTplane;
	if (mA->modelsB.size() > 0) {
		planes = mA;
		tubes = mB;
		wTplane = wTa;
		wTtube = wTb;
	} else {
		planes = mB;
		tubes = mA;
		wTplane = wTb;
		wTtube = wTa;
	}

	for (std::size_t i = 0; i < tubes->modelsA.size(); i++) {
		const GeometryModel::TypeA tube = tubes->modelsA[i];
		for (std::size_t j = 0; j < planes->modelsB.size(); j++) {
			const GeometryModel::TypeB plane = planes->modelsB[j];
			const Transform3D<> tubePose = wTtube*tube.transform;
			const Transform3D<> planePose = wTplane*plane.transform;
			std::vector<TrackInfo> info;
			if (tubeTracking->find(i,j,info)) {
				std::vector<Contact> contacts;
				determineContacts(contacts, tube.geo, tubePose, plane.geo, planePose, info, false);
				RW_ASSERT(contacts.size() == info.size());
				const Transform3D<> boxTplane = inverse(tubePose)*planePose;
				BOOST_FOREACH(Contact& c, contacts) {
					c.setTransform(boxTplane);
					c.setModelA(tubes);
					c.setModelB(planes);
					c.setFrameA(tube.frame);
					c.setFrameB(plane.frame);
					res.push_back(c);
				}
				for (std::size_t k = 0; k < info.size(); k++) {
					info[k].modelIDa = i;
					info[k].modelIDb = j;
					info[k].wPtube = contacts[k].getPointA();
				}
				newInfo.insert(newInfo.end(),info.begin(),info.end());
			}
		}
	}
	tubeTracking->info = newInfo;
	RW_ASSERT(res.size() == tubeTracking->info.size());
	return res;
}

std::string TubePlaneStrategy::getName() {
	return "TubePlaneStrategy";
}
