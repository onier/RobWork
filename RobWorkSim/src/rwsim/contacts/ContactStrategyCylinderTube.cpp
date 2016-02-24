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

#include "ContactStrategyCylinderTube.hpp"
#include "ContactStrategyTracking.hpp"
#include "ContactModelGeometry.hpp"
#include "GeometricUtil.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Tube.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/log/LogMessage.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsim::log;

struct ContactStrategyCylinderTube::TrackInfo {
	TrackInfo(): modelIDa(0), modelIDb(0), userData(NULL), cylinderRegion(CYLALL), tubeRegion(TUBEALL), caseInfo(0) {}
	std::size_t modelIDa;
	std::size_t modelIDb;
	rwsim::contacts::ContactStrategyTracking::UserData::Ptr userData;
	CylinderRegion cylinderRegion;
	TubeRegion tubeRegion;
	double caseInfo;
};

class ContactStrategyCylinderTube::TrackingCylinderTube: public rwsim::contacts::ContactStrategyTracking::StrategyData {
public:
	TrackingCylinderTube();
	virtual ~TrackingCylinderTube();
	virtual const rwsim::contacts::ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const;
	virtual void setUserData(std::size_t index, const rwsim::contacts::ContactStrategyTracking::UserData::Ptr data);
	virtual void remove(std::size_t index);
	virtual StrategyData* copy() const;
	virtual std::size_t getSize() const;
	bool find(std::size_t a, std::size_t b, std::vector<TrackInfo>& res) const;

public:
	std::vector<TrackInfo> info;
};

ContactStrategyCylinderTube::TrackingCylinderTube::TrackingCylinderTube() {};
ContactStrategyCylinderTube::TrackingCylinderTube::~TrackingCylinderTube() {};

const ContactStrategyTracking::UserData::Ptr ContactStrategyCylinderTube::TrackingCylinderTube::getUserData(std::size_t index) const {
	RW_ASSERT(index < info.size());
	return info[index].userData;
}

void ContactStrategyCylinderTube::TrackingCylinderTube::setUserData(std::size_t index, const ContactStrategyTracking::UserData::Ptr data) {
	RW_ASSERT(index < info.size());
	info[index].userData = data;
}

void ContactStrategyCylinderTube::TrackingCylinderTube::remove(std::size_t index) {
	RW_ASSERT(index < info.size());
	info.erase(info.begin()+index);
}

ContactStrategyTracking::StrategyData* ContactStrategyCylinderTube::TrackingCylinderTube::copy() const {
	TrackingCylinderTube* tracking = new TrackingCylinderTube();
	tracking->info = info;
	return tracking;
}

std::size_t ContactStrategyCylinderTube::TrackingCylinderTube::getSize() const {
	return info.size();
}

bool ContactStrategyCylinderTube::TrackingCylinderTube::find(std::size_t a, std::size_t b, std::vector<TrackInfo>& res) const {
	res.clear();
	for (std::size_t i = 0; i < info.size(); i++) {
		if (info[i].modelIDa == a && info[i].modelIDb == b) {
			res.push_back(info[i]);
		}
	}
	return res.size() > 0;
}

struct ContactStrategyCylinderTube::Characteristics {
	double R;
	double r;
	double L;
	double l;
	Vector3D<> pc;
	Vector3D<> pt;
	Vector3D<> dc; // Cylinder z-axis
	Vector3D<> dt; // Tube z-axis
	Vector3D<> proj_pc;

	struct Circle {
		Circle() {}
		Circle(Vector3D<> c, Vector3D<> n): c(c), n(n) {}
		Vector3D<> c;
		Vector3D<> n;
		Vector3D<> high;
		Vector3D<> low;
	};

	Circle cTop;
	Circle cBottom;
	Circle tTop;
	Circle tBottom;

	Vector3D<> pcTopHigh;
	Vector3D<> pcTopLow;
	Vector3D<> pcBotHigh;
	Vector3D<> pcBotLow;
	Vector3D<> ptTopHigh;
	Vector3D<> ptTopLow;
	Vector3D<> ptBotHigh;
	Vector3D<> ptBotLow;
};

ContactStrategyCylinderTube::ContactStrategyCylinderTube():
	ContactStrategyGeometry<Cylinder*, Tube*>(),
	_rounding(0)
{
}

ContactStrategyCylinderTube::~ContactStrategyCylinderTube() {
}

bool ContactStrategyCylinderTube::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	if (geoA->getType() == GeometryData::TubePrim && geoB->getType() == GeometryData::CylinderPrim)
		return true;
	else if (geoA->getType() == GeometryData::CylinderPrim && geoB->getType() == GeometryData::TubePrim)
		return true;
	return false;
}

std::vector<Contact> ContactStrategyCylinderTube::findContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		SimulatorLogScope* log) const
{
	if (log != NULL) {
		LogMessage::Ptr msg = ownedPtr(new LogMessage(log));
		msg->setFilename(__FILE__);
		msg->setLine(__LINE__);
		msg->setDescription("Find Cylinder-Tube Contacts");
		log->appendChild(msg);
	}
	const GeometryModel::Ptr mA = a.cast<GeometryModel>();
	const GeometryModel::Ptr mB = b.cast<GeometryModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new TrackingCylinderTube());
	TrackingCylinderTube* const ctTracking = dynamic_cast<TrackingCylinderTube*>(tracking.getStrategyData());
	RW_ASSERT(ctTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two cylinders.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two tubes.");
	GeometryModel::Ptr cylinders;
	GeometryModel::Ptr tubes;
	Transform3D<> wTcylinder;
	Transform3D<> wTtube;
	if (mA->modelsB.size() > 0) {
		tubes = mA;
		cylinders = mB;
		wTtube = wTa;
		wTcylinder = wTb;
	} else {
		tubes = mB;
		cylinders = mA;
		wTtube = wTb;
		wTcylinder = wTa;
	}

	std::vector<Contact> contacts;
	std::vector<TrackInfo> newInfo;

	for (std::size_t i = 0; i < cylinders->modelsA.size(); i++) {
		const GeometryModel::TypeA cylinder = cylinders->modelsA[i];
		for (std::size_t j = 0; j < tubes->modelsB.size(); j++) {
			const GeometryModel::TypeB tube = tubes->modelsB[j];
			const Transform3D<> pegPose = wTcylinder*cylinder.transform;
			const Transform3D<> tubePose = wTtube*tube.transform;
			const Characteristics c = getCharacteristics(cylinder.geo,pegPose,tube.geo,tubePose);
			std::vector<TrackInfo> tinfos;
			std::vector<MetaContact> cs;
			bool update = false;
			if (ctTracking->find(i,j,tinfos)) {
				cs = updateContacts(c,tinfos,true,log);
				update = true;
			} else {
				cs = findContacts(c, getThreshold());
				tinfos.resize(cs.size());
				update = false;
			}
			std::vector<TrackInfo>::iterator it = tinfos.begin();
			BOOST_FOREACH(MetaContact& mcontact, cs) {
				mcontact.contact.setTransform(inverse(pegPose)*tubePose);
				mcontact.contact.setModelA(cylinders);
				mcontact.contact.setModelB(tubes);
				mcontact.contact.setFrameA(cylinder.frame);
				mcontact.contact.setFrameB(tube.frame);
				contacts.push_back(mcontact.contact);
				if (!update) {
					TrackInfo tinfo;
					tinfo.modelIDa = i;
					tinfo.modelIDb = j;
					tinfo.cylinderRegion = mcontact.cylinderRegion;
					tinfo.tubeRegion = mcontact.tubeRegion;
					tinfo.caseInfo = mcontact.caseInfo;
					RW_ASSERT(tinfo.cylinderRegion != CYLALL);
					RW_ASSERT(tinfo.tubeRegion != TUBEALL);
					newInfo.push_back(tinfo);
				}
			}
			if (update) {
				BOOST_FOREACH(const TrackInfo& tinfo, tinfos) {
					newInfo.push_back(tinfo);
					newInfo.back().modelIDa = i;
					newInfo.back().modelIDb = j;
					RW_ASSERT(tinfo.cylinderRegion != CYLALL);
					RW_ASSERT(tinfo.tubeRegion != TUBEALL);
				}
			}
		}
	}

	ctTracking->info = newInfo;
	RW_ASSERT(contacts.size() == ctTracking->info.size());
	return contacts;
}

std::vector<Contact> ContactStrategyCylinderTube::updateContacts(
		ProximityModel::Ptr a,
		const Transform3D<>& wTa,
		ProximityModel::Ptr b,
		const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		SimulatorLogScope* log) const
{
	if (log != NULL) {
		LogMessage::Ptr msg = ownedPtr(new LogMessage(log));
		msg->setFilename(__FILE__);
		msg->setLine(__LINE__);
		msg->setDescription("Update Cylinder-Tube Contacts");
		log->appendChild(msg);
	}
	const GeometryModel::Ptr mA = a.cast<GeometryModel>();
	const GeometryModel::Ptr mB = b.cast<GeometryModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	RW_ASSERT(mA->modelsA.size() > 0 || mA->modelsB.size() > 0);
	RW_ASSERT(mB->modelsA.size() > 0 || mB->modelsB.size() > 0);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new TrackingCylinderTube());
	TrackingCylinderTube* const ctTracking = dynamic_cast<TrackingCylinderTube*>(tracking.getStrategyData());
	RW_ASSERT(ctTracking);

	if (mA->modelsA.size() > 0 && mB->modelsA.size() > 0)
		RW_THROW("Can not find contacts between two cylinders.");
	if (mA->modelsB.size() > 0 && mB->modelsB.size() > 0)
		RW_THROW("Can not find contacts between two tubes.");
	GeometryModel::Ptr cylinders;
	GeometryModel::Ptr tubes;
	Transform3D<> wTcylinder;
	Transform3D<> wTtube;
	if (mA->modelsB.size() > 0) {
		tubes = mA;
		cylinders = mB;
		wTtube = wTa;
		wTcylinder = wTb;
	} else {
		tubes = mB;
		cylinders = mA;
		wTtube = wTb;
		wTcylinder = wTa;
	}

	std::vector<Contact> contacts;
	std::vector<TrackInfo> newInfo;
	for (std::size_t i = 0; i < cylinders->modelsA.size(); i++) {
		const GeometryModel::TypeA cylinder = cylinders->modelsA[i];
		for (std::size_t j = 0; j < tubes->modelsB.size(); j++) {
			const GeometryModel::TypeB tube = tubes->modelsB[j];
			const Transform3D<> cylinderPose = wTcylinder*cylinder.transform;
			const Transform3D<> tubePose = wTtube*tube.transform;
			std::vector<TrackInfo> tinfos;
			if (ctTracking->find(i,j,tinfos)) {
				LogMessage::Ptr msg = NULL;
				if (log != NULL) {
					msg = ownedPtr(new LogMessage(log));
					msg->setFilename(__FILE__);
					msg->setLine(__LINE__);
					msg->setDescription("Tracked");
					log->appendChild(msg);
					msg->stream() << "Found " << tinfos.size() << " tracked contacts." << std::endl;
					BOOST_FOREACH(const TrackInfo& info, tinfos) {
						msg->stream() << " - cyl: " << info.cylinderRegion << " tube: " << info.tubeRegion << " info: " << info.caseInfo << std::endl;
					}
				}
				const Characteristics c = getCharacteristics(cylinder.geo,cylinderPose,tube.geo,tubePose);
				std::vector<MetaContact> cs = updateContacts(c, tinfos, false, log);
				RW_ASSERT(cs.size() == tinfos.size());
				BOOST_FOREACH(MetaContact& mcontact, cs) {
					mcontact.contact.setModelA(cylinders);
					mcontact.contact.setModelB(tubes);
					mcontact.contact.setFrameA(cylinder.frame);
					mcontact.contact.setFrameB(tube.frame);
					mcontact.contact.setTransform(inverse(cylinderPose)*tubePose);
					contacts.push_back(mcontact.contact);
				}
				if (log != NULL) {
					msg->stream() << "Updated " << tinfos.size() << " tracked contacts." << std::endl;
					BOOST_FOREACH(const TrackInfo& info, tinfos) {
						msg->stream() << " - cyl: " << info.cylinderRegion << " tube: " << info.tubeRegion << " info: " << info.caseInfo << std::endl;
					}
				}
				BOOST_FOREACH(const TrackInfo& tinfo, tinfos) {
					newInfo.push_back(tinfo);
					newInfo.back().modelIDa = i;
					newInfo.back().modelIDb = j;
				}
			}
		}
	}

	ctTracking->info = newInfo;
	RW_ASSERT(contacts.size() == ctTracking->info.size());
	return contacts;
}

std::string ContactStrategyCylinderTube::getName() {
	return "ContactStrategyCylinderTube";
}

double ContactStrategyCylinderTube::getThreshold() const {
	return 0;
	/*double threshold = _propertyMap.get<double>("ContactStrategyCylinderTubeThreshold", -0.1);
	if (threshold == -0.1)
		threshold = _propertyMap.get<double>("MaxSepDistance", 0.0005);
    return threshold;*/
}

double ContactStrategyCylinderTube::getRoundingRadius() const {
	return _rounding;
}

void ContactStrategyCylinderTube::setRoundingRadius(double radius) {
	_rounding = radius;
}

std::string ContactStrategyCylinderTube::cylinderRegionToString(CylinderRegion region) {
	switch(region) {
	case SURFACE:
		return "Surface";
	case TOPPLANE:
		return "Top Plane";
	case BOTTOMPLANE:
		return "Bottom Plane";
	case CYLTOPEDGE:
		return "Top Edge";
	case CYLBOTTOMEDGE:
		return "Bottom Edge";
	default:
		return "(invalid)";
	}
}
std::string ContactStrategyCylinderTube::tubeRegionToString(TubeRegion region) {
	switch(region) {
	case INNERSURFACE:
		return "Inner Surface";
	case OUTERSURFACE:
		return "Outer Surface";
	case TUBETOPEDGE:
		return "Top Edge";
	case TUBEBOTTOMEDGE:
		return "Bottom Edge";
	default:
		return "(invalid)";
	}
}

const double ContactStrategyCylinderTube::NO_THRESHOLD = -1;

bool ContactStrategyCylinderTube::cylSurfaceTubeEdge(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold, TubeRegion tubeRegion) {
	bool found = false;
	// Case 3
	// Top
	if (tubeRegion == TUBETOPEDGE || tubeRegion == TUBEALL) {
		// Check if tube edge lies inside the rounded surface region of the cylinder
		if (std::min<double>(fabs(dot(c.ptTopHigh-c.pc,c.dc)),fabs(dot(c.ptTopLow-c.pc,c.dc))) < c.l/2.) {
			////std::cout << "cd: 3 top: cylSurfaceTubeEdge()" << std::endl;
			const Vector3D<> proj_ct = GeometricUtil::projectPointOnPlane(c.tTop.c, c.pc, c.dc);
			// Check that the projected center lies close enough to the tube for there to be a valid contact
			bool closeEnough = false;
			if (threshold < 0)
				closeEnough = true;
			else if ((proj_ct-c.pc).norm2() <= c.R+c.r)
				closeEnough = true;
			if (closeEnough) {
				////std::cout << "cd: 3 top: cylSurfaceTubeEdge() bp2 passed" << std::endl;
				// if line passes through circle: take the farthest point, if it passes outside take the nearest? (perpendicular is a border case where both are there?)
				const std::vector<GeometricUtil::PointPair> closestAll = GeometricUtil::closestPointsOnCircleToLine(c.tTop.c, c.R, c.tTop.n, c.pc, c.dc);
				BOOST_FOREACH(const GeometricUtil::PointPair& closest, closestAll) {
					const Vector3D<> tube = closest.first;
					const Vector3D<> line = closest.second;
					MetaContact mcontact;
					mcontact.cylinderRegion = SURFACE;
					mcontact.tubeRegion = TUBETOPEDGE;
					const Vector3D<> n = normalize(tube-line);
					//std::cout << " - contact " << dot(n,c.tTop.n) << std::endl;
					if (dot(n,c.tTop.n) < 0) {
						mcontact.contact.setPointA(line+n*c.r);
						mcontact.contact.setPointB(tube);
						mcontact.contact.setNormal(n);
						mcontact.contact.setDepth();
						// Do a final check that the found point is in fact on the round part of the cylinder (and not the edge)
						//std::cout << fabs(dot(tube-c.pc,c.dc)) << "<" << c.l/2. << " " << c.pc << " " << tube << std::endl;
						if (fabs(dot(tube-c.pc,c.dc)) < c.l/2.) {
							//std::cout << "cd: 3 top: cylSurfaceTubeEdge() on surface check passed" << std::endl;
							// Also check that threshold is ok
							bool thresOK = false;
							if (threshold < 0)
								thresOK = true;
							else if (mcontact.contact.getDepth() >= -threshold)
								thresOK = true;
							if (thresOK) {
								//std::cout << "cd: 3 top: cylSurfaceTubeEdge() final threshold check passed" << std::endl;
								mcontacts.push_back(mcontact);
								found = true;
							}
						}
					}
				}
			}
		}
	}
	// Bottom
	if (tubeRegion == TUBEBOTTOMEDGE || tubeRegion == TUBEALL) {
		// Check if tube edge lies inside the rounded surface region of the cylinder
		if (std::min<double>(fabs(dot(c.ptBotHigh-c.pc,c.dc)),fabs(dot(c.ptBotLow-c.pc,c.dc))) < c.l/2.) {
			//std::cout << "cd: 3 bottom: cylSurfaceTubeEdge()" << std::endl;
			const Vector3D<> proj_ct = GeometricUtil::projectPointOnPlane(c.tBottom.c, c.pc, c.dc);
			// Check that the projected center lies close enough to the tube for there to be a valid contact
			bool closeEnough = false;
			if (threshold < 0)
				closeEnough = true;
			else if ((proj_ct-c.pc).norm2() <= c.R+c.r)
				closeEnough = true;
			if (closeEnough) {
				const std::vector<GeometricUtil::PointPair> closestAll = GeometricUtil::closestPointsOnCircleToLine(c.tTop.c, c.R, c.tTop.n, c.pc, c.dc);
				BOOST_FOREACH(const GeometricUtil::PointPair& closest, closestAll) {
					const Vector3D<> tube = closest.first;
					const Vector3D<> line = closest.second;
					MetaContact mcontact;
					mcontact.cylinderRegion = SURFACE;
					mcontact.tubeRegion = TUBEBOTTOMEDGE;
					const Vector3D<> n = normalize(tube-line);
					mcontact.contact.setPointA(line+n*c.r);
					mcontact.contact.setPointB(tube);
					mcontact.contact.setNormal(n);
					mcontact.contact.setDepth();
					// Do a final check that the found point is in fact on the round part of the cylinder (and not the edge)
					if (fabs(dot(tube-c.pc,c.dc)) < c.l/2.) {
						// Also check that threshold is ok
						bool thresOK = false;
						if (threshold < 0)
							thresOK = true;
						else if (mcontact.contact.getDepth() >= -threshold)
							thresOK = true;
						if (thresOK) {
							mcontacts.push_back(mcontact);
							found = true;
						}
					}
				}
			}
		}
	}
	return found;
}

bool ContactStrategyCylinderTube::cylPlaneTubeEdge(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold, CylinderRegion cylRegion, TubeRegion tubeRegion) {
	bool found = false;
	// Case 6
	// Tube top edge
	if (tubeRegion == TUBETOPEDGE || tubeRegion == TUBEALL) {
		// Check if cylinder center lies in the tube edge region
		if (dot(c.pc-c.pt,c.tTop.n) > c.L/2. && (c.proj_pc-c.pt).norm2() > c.R) {
			//std::cout << "cd: 6 top: cylPlaneTubeEdge()" << std::endl;
			const bool cylTop = dot(c.dt,c.dc) < 0;
			const int sgn = cylTop ? 1 : -1;
			Vector3D<> closest = GeometricUtil::closestPointOnCircleToPlane(c.tTop.c, c.R, c.tTop.n, sgn*c.dc);
			Vector3D<> cc = c.pc+sgn*c.l/2.*c.dc;
			Vector3D<> cyl = GeometricUtil::projectPointOnPlane(closest, cc, sgn*c.dc);
			// Found point must lie at end-cap
			//std::cout << "end cap test " << (cyl-cc).norm2() << "<" << c.r << std::endl;
			if ((cyl-cc).norm2() < c.r) {
				//std::cout << "at end cap" << std::endl;
				MetaContact mcontact;
				if (cylTop)
					mcontact.cylinderRegion = TOPPLANE;
				else
					mcontact.cylinderRegion = BOTTOMPLANE;
				mcontact.tubeRegion = TUBETOPEDGE;
				mcontact.contact.setPointA(cyl);
				mcontact.contact.setPointB(closest);
				mcontact.contact.setNormal(sgn*c.dc);
				mcontact.contact.setDepth();
				// Check that threshold is ok
				bool thresholdOK = threshold < 0;
				if (!thresholdOK)
					thresholdOK = mcontact.contact.getDepth() >= -threshold;
				if (thresholdOK) {
					mcontacts.push_back(mcontact);
					found = true;
				}
			}
		}
	}
	// Tube bottom edge
	if (tubeRegion == TUBEBOTTOMEDGE || tubeRegion == TUBEALL) {
		// Check if cylinder center lies in the tube edge region
		if (dot(c.pc-c.pt,c.tBottom.n) > c.L/2. && (c.proj_pc-c.pt).norm2() > c.R) {
			////std::cout << "cd: 6 bot: cylPlaneTubeEdge()" << std::endl;
			const bool cylTop = dot(c.dt,c.dc) > 0;
			const int sgn = cylTop ? 1 : -1;
			const Vector3D<> closest = GeometricUtil::closestPointOnCircleToPlane(c.tBottom.c, c.R, c.tBottom.n, sgn*c.dc);
			const Vector3D<> cc = c.pc+sgn*c.l/2.*c.dc;
			const Vector3D<> cyl = GeometricUtil::projectPointOnPlane(closest, cc, sgn*c.dc);
			// Found point must lie at end-cap
			if ((cyl-cc).norm2() < c.r) {
				MetaContact mcontact;
				if (cylTop)
					mcontact.cylinderRegion = TOPPLANE;
				else
					mcontact.cylinderRegion = BOTTOMPLANE;
				mcontact.tubeRegion = TUBEBOTTOMEDGE;
				mcontact.contact.setPointA(cyl);
				mcontact.contact.setPointB(closest);
				mcontact.contact.setNormal(sgn*c.dc);
				mcontact.contact.setDepth();
				// Check that threshold is ok
				bool thresholdOK = threshold < 0;
				if (!thresholdOK)
					thresholdOK = mcontact.contact.getDepth() >= -threshold;
				if (thresholdOK) {
					mcontacts.push_back(mcontact);
					found = true;
				}
			}
		}
	}
	return found;
}

bool ContactStrategyCylinderTube::cylEdgeTubeSurface(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold, CylinderRegion cylRegion, TubeRegion tubeRegion) {
	const double realThres = threshold > 0 ? threshold : 0;
	bool found = false;
	// Case 7+8:
	// Cylinder edges to Inner surface of tube
	// Cylinder edges to Outside surface of tube
	// Top
	if (cylRegion == CYLTOPEDGE || cylRegion == CYLALL) {
		// Check if edge lies inside the inner region of tube
		if (std::min<double>(fabs(dot(c.pcTopHigh-c.pt,c.dt)),fabs(dot(c.pcTopLow-c.pt,c.dt))) < c.L/2.) {
			//std::cout << "cd: 7+8 top: cylEdgeTubeSurface()" << std::endl;
			const Vector3D<> proj_cc = GeometricUtil::projectPointOnPlane(c.cTop.c, c.pt, c.dt);
			// Check that the projected center lies close enough to the tube for there to be a valid contact
			double dist = (proj_cc-c.pt).norm2();
			if (dist >= c.R-c.r-realThres && dist <= c.R+c.r+realThres) {
				const Vector3D<> u = normalize(cross(c.dc,c.dt));
				const Vector3D<> v = normalize(cross(c.dt,u));
				const double ru = c.r;
				const double rv = c.r*dot(c.dc,c.dt);
				const GeometricUtil::PointPair extremalPoints = GeometricUtil::extremalPointsOnEllipseToPoint(c.pt, proj_cc, u, ru, v, rv);

				// Farthest points (case 7)
				if ((tubeRegion == INNERSURFACE || tubeRegion == TUBEALL) && dist < c.R) {
					const Vector3D<> &proj_ellipse = extremalPoints.first;
					MetaContact mcontact;
					mcontact.cylinderRegion = CYLTOPEDGE;
					mcontact.tubeRegion = INNERSURFACE;
					const Vector3D<> n = normalize(proj_ellipse-c.pt);
					// Make sure that cylinder is tilted correct for the contact to make sense
					if (dot(proj_ellipse-n*c.r-c.proj_pc,n) > 0) {
						const Vector3D<> proj_circle = c.pt+n*c.R;
						const double height = dot(c.cTop.c-proj_ellipse,c.dc)/dot(c.dt,c.dc);
						mcontact.contact.setPointA(proj_ellipse+height*c.dt);
						mcontact.contact.setPointB(proj_circle+height*c.dt);
						mcontact.contact.setNormal(n);
						mcontact.contact.setDepth();
						// Do a final check that the found point is in fact on the inner part of the tube (and not the edge)
						if (fabs(dot(mcontact.contact.getPointB()-c.pt,c.dt)) < c.L/2.) {
							// Also check that threshold is ok
							bool thresholdOK = threshold < 0;
							if (!thresholdOK)
								thresholdOK = mcontact.contact.getDepth() >= -threshold;
							if (thresholdOK) {
								mcontacts.push_back(mcontact);
								found = true;
							}
						}
					}
				}

				// Closest points (case 8)
				if ((tubeRegion == OUTERSURFACE || tubeRegion == TUBEALL) && dist > c.R) {
					const Vector3D<> &proj_ellipse = extremalPoints.second;
					MetaContact mcontact;
					mcontact.cylinderRegion = CYLTOPEDGE;
					mcontact.tubeRegion = OUTERSURFACE;
					const Vector3D<> n = normalize(c.pt-proj_ellipse);
					// Make sure that cylinder is tilted correct for the contact to make sense
					if (dot(proj_ellipse-n*c.r-c.proj_pc,n) > 0) {
						const Vector3D<> proj_circle = c.pt-n*c.R;
						const double height = dot(c.cTop.c-proj_ellipse,c.dc)/dot(c.dt,c.dc);
						mcontact.contact.setPointA(proj_ellipse+height*c.dt);
						mcontact.contact.setPointB(proj_circle+height*c.dt);
						mcontact.contact.setNormal(n);
						mcontact.contact.setDepth();
						// Do a final check that the found point is in fact on the inner part of the tube (and not the edge)
						if (fabs(dot(mcontact.contact.getPointB()-c.pt,c.dt)) < c.L/2.) {
							// Also check that threshold is ok
							bool thresholdOK = threshold < 0;
							if (!thresholdOK)
								thresholdOK = mcontact.contact.getDepth() >= -threshold;
							if (thresholdOK) {
								mcontacts.push_back(mcontact);
								found = true;
							}
						}
					}
				}
			}
		}
	}
	// Bottom
	if (cylRegion == CYLBOTTOMEDGE || cylRegion == CYLALL) {
		// Check if edge lies inside the inner region of tube
		if (std::min<double>(fabs(dot(c.pcBotHigh-c.pt,c.dt)),fabs(dot(c.pcBotLow-c.pt,c.dt))) < c.L/2.) {
			//std::cout << "cd: 7+8 bottom: cylEdgeTubeSurface()" << std::endl;
			Vector3D<> proj_cc = GeometricUtil::projectPointOnPlane(c.cBottom.c, c.pt, c.dt);
			// Check that the projected center lies close enough to the tube for there to be a valid contact
			double dist = (proj_cc-c.pt).norm2();
			bool bp1 = threshold < 0;
			if (!bp1)
				bp1 = dist >= c.R-c.r-realThres && dist <= c.R+c.r+realThres;
			if (bp1) {
				//std::cout << " - inside threshold" << std::endl;
				Vector3D<> u = normalize(cross(c.dc,c.dt));
				Vector3D<> v = normalize(cross(c.dt,u));
				double ru = c.r;
				double rv = c.r*dot(c.dc,c.dt);
				const GeometricUtil::PointPair extremalPoints = GeometricUtil::extremalPointsOnEllipseToPoint(c.pt, proj_cc, u, ru, v, rv);

				// Farthest points (case 7)
				if ((tubeRegion == INNERSURFACE || tubeRegion == TUBEALL) && dist < c.R) {
					//std::cout << " - inner" << std::endl;
					const Vector3D<> &proj_ellipse = extremalPoints.first;
					//std::cout << " - - contact" << std::endl;
					MetaContact mcontact;
					mcontact.cylinderRegion = CYLBOTTOMEDGE;
					mcontact.tubeRegion = INNERSURFACE;
					const Vector3D<> n = normalize(proj_ellipse-c.pt);
					// Make sure that cylinder is tilted correct for the contact to make sense
					if (dot(proj_ellipse-n*c.r-c.proj_pc,n) > 0) {
						//std::cout << " - - - inside threshold a" << std::endl;
						const Vector3D<> proj_circle = c.pt+n*c.R;
						const double height = dot(c.cBottom.c-proj_ellipse,c.dc)/dot(c.dt,c.dc);
						mcontact.contact.setPointA(proj_ellipse+height*c.dt);
						mcontact.contact.setPointB(proj_circle+height*c.dt);
						mcontact.contact.setNormal(n);
						mcontact.contact.setDepth();
						// Do a final check that the found point is in fact on the inner part of the tube (and not the edge)
						if (fabs(dot(mcontact.contact.getPointB()-c.pt,c.dt)) < c.L/2.) {
							//std::cout << " - - - inside threshold b" << std::endl;
							// Also check that threshold is ok
							bool thresholdOK = threshold < 0;
							if (!thresholdOK)
								thresholdOK = mcontact.contact.getDepth() >= -threshold;
							if (thresholdOK) {
								//std::cout << " - - - inside threshold c" << std::endl;
								mcontacts.push_back(mcontact);
								found = true;
							}
						}
					}
				}
				// Closest points (case 8)
				if ((tubeRegion == OUTERSURFACE || tubeRegion == TUBEALL) && dist > c.R) {
					//std::cout << " - outer" << std::endl;
					const Vector3D<> &proj_ellipse = extremalPoints.second;
					//std::cout << " - - contact" << std::endl;
					MetaContact mcontact;
					mcontact.cylinderRegion = CYLBOTTOMEDGE;
					mcontact.tubeRegion = OUTERSURFACE;
					const Vector3D<> n = normalize(proj_ellipse-c.pt);
					// Make sure that cylinder is tilted correct for the contact to make sense
					if (dot(proj_cc-c.proj_pc,n) < 0) {
						//std::cout << " - - - inside threshold a" << std::endl;
						const Vector3D<> proj_circle = c.pt+n*c.R;
						const double height = dot(c.cBottom.c-proj_ellipse,c.dc)/dot(c.dt,c.dc);
						mcontact.contact.setPointA(proj_ellipse+height*c.dt);
						mcontact.contact.setPointB(proj_circle+height*c.dt);
						mcontact.contact.setNormal(n);
						mcontact.contact.setDepth();
						// Do a final check that the found point is in fact on the inner part of the tube (and not the edge)
						if (fabs(dot(mcontact.contact.getPointB()-c.pt,c.dt)) < c.L/2.) {
							//std::cout << " - - - inside threshold b" << std::endl;
							// Also check that threshold is ok
							bool thresholdOK = threshold < 0;
							if (!thresholdOK)
								thresholdOK = mcontact.contact.getDepth() >= -threshold;
							if (thresholdOK) {
								//std::cout << " - - - inside threshold c" << std::endl;
								mcontacts.push_back(mcontact);
								found = true;
							}
						}
					}
				}
			}
		}
	}
	return found;
}

bool ContactStrategyCylinderTube::cylEdgeTubeEdgeMakeContact(
		MetaContact& mcontact,
		const Characteristics& c,
		bool cylAtTop,
		bool tubeAtTop,
		const Vector3D<>& cylPoint,
		const Vector3D<>& tubePoint,
		double threshold)
{
	mcontact.cylinderRegion = cylAtTop ? CYLTOPEDGE : CYLBOTTOMEDGE;
	mcontact.tubeRegion = tubeAtTop ? TUBETOPEDGE : TUBEBOTTOMEDGE;
	mcontact.contact.setPointA(cylPoint);
	mcontact.contact.setPointB(tubePoint);
	const Vector3D<> diff = tubePoint-cylPoint;
	const Vector3D<> cTube = tubeAtTop ? c.tTop.c : c.tBottom.c;
	const Vector3D<> nTube = tubeAtTop ? c.tTop.n : c.tBottom.n;
	const Vector3D<> cCyl = cylAtTop ? c.cTop.c : c.cBottom.c;
	const Vector3D<> nCyl = cylAtTop ? c.cTop.n : c.cBottom.n;
	Vector3D<> n;
	//if (diff.norm2() > 0.01*c.r)
	{
		//n = normalize(diff);
		//mcontact.contact.setNormal(n);
		//mcontact.contact.setDepth();
		//std::cout << "normalized: " << mcontact.contact.getDepth() << " " << mcontact.contact.getNormal() << std::endl;
	}
	//else
	{
		Vector3D<> t1 = normalize(cross(nTube,tubePoint-cTube));
		Vector3D<> t2 = normalize(cross(nCyl,cylPoint-cCyl));
		n = normalize(cross(t1,t2));
		mcontact.contact.setNormal(n);
		mcontact.contact.setDepth();
		//std::cout << "crossed: " << mcontact.contact.getDepth() << " " << mcontact.contact.getNormal() << std::endl;
	}
	//mcontact.contact.setNormal(n);
	//mcontact.contact.setDepth();
	// Find correct sign for normal (penetration or not?)
	//bool penetration = false;
	if (dot(nCyl,n) < 0) {
		mcontact.contact.setNormal(-n);
		mcontact.contact.setDepth();
		//std::cout << "reversed: " << mcontact.contact.getDepth() << " " << mcontact.contact.getNormal() << std::endl;
	}
	/*const Vector3D<> proj_cyl = GeometricUtil::projectPointOnPlane(tubePoint,cCyl,nCyl);
	const double distFromTubePointToCylinderPlane = dot(tubePoint-cCyl,nCyl);
	if (distFromTubePointToCylinderPlane < 0 && distFromTubePointToCylinderPlane > -c.l/2.) {
		const double distFromCylinderAxisToTubePointProjection = (proj_cyl-cCyl).norm2();
		if (distFromCylinderAxisToTubePointProjection < c.r)
			penetration = true;
	}
	if ((penetration && mcontact.contact.getDepth() < 0) || (!penetration && mcontact.contact.getDepth() > 0)) {
		mcontact.contact.setNormal(-n);
		mcontact.contact.setDepth();
	}*/
	bool thresholdOK = threshold < 0;
	if (!thresholdOK)
		thresholdOK = mcontact.contact.getDepth() >= -threshold;
	if (thresholdOK) {
		const Vector3D<> dir = cross(c.dc,nTube);
		mcontact.caseInfo = dot(dir,mcontact.contact.getPointA()-cCyl);
		return true;
	}
	return false;
}

bool ContactStrategyCylinderTube::cylEdgeTubeEdge(const Characteristics& c, std::vector<MetaContact> &mcontacts, double threshold, CylinderRegion cylRegion, TubeRegion tubeRegion) {
	bool found = false;
	// Case 9
	// Edge to Edge contact
	// Cyl top - Tube top
	if ((cylRegion == CYLTOPEDGE || cylRegion == CYLALL) && (tubeRegion == TUBETOPEDGE || tubeRegion == TUBEALL)) {
		//std::cout << " - - cd: 9 cyl top tube top: cylEdgeTubeEdge()" << std::endl;
		const std::vector<std::pair<Vector3D<>, Vector3D<> > > closest = GeometricUtil::closestPointsOnCircleToCircle(c.tTop.c, c.tTop.n, c.R, c.cTop.c, c.cTop.n, c.r);
		for (std::size_t i = 0 ; i < closest.size(); i++) {
			MetaContact mcontact;
			//std::cout << " top pair " << i << ": " << closest[i].second << " " << closest[i].first << std::endl;
			if (cylEdgeTubeEdgeMakeContact(mcontact,c,true,true,closest[i].second,closest[i].first,threshold)) {
				mcontacts.push_back(mcontact);
				found = true;
			}
		}
	}
	// Cyl top - Tube bottom
	if ((cylRegion == CYLTOPEDGE || cylRegion == CYLALL) && (tubeRegion == TUBEBOTTOMEDGE || tubeRegion == TUBEALL)) {
		//std::cout << " - - cd: 9 cyl top tube bottom: cylEdgeTubeEdge()" << std::endl;
		const std::vector<std::pair<Vector3D<>, Vector3D<> > > closest = GeometricUtil::closestPointsOnCircleToCircle(c.tBottom.c, c.tBottom.n, c.R, c.cTop.c, c.cTop.n, c.r);
		for (std::size_t i = 0 ; i < closest.size(); i++) {
			MetaContact mcontact;
			if (cylEdgeTubeEdgeMakeContact(mcontact,c,true,false,closest[i].second,closest[i].first,threshold)) {
				mcontacts.push_back(mcontact);
				found = true;
			}
		}
	}
	// Cyl bottom - Tube top
	if ((cylRegion == CYLBOTTOMEDGE || cylRegion == CYLALL) && (tubeRegion == TUBETOPEDGE || tubeRegion == TUBEALL)) {
		//std::cout << " - - cd: 9 cyl bottom tube top: cylEdgeTubeEdge()" << std::endl;
		//std::cout << "closestPointOnCircleToCircle: " << c.tTop.c << " " << c.tTop.n << " " << c.R << " " << c.cBottom.c << " " << c.cBottom.n << " " << c.r << std::endl;
		const std::vector<std::pair<Vector3D<>, Vector3D<> > > closest = GeometricUtil::closestPointsOnCircleToCircle(c.tTop.c, c.tTop.n, c.R, c.cBottom.c, c.cBottom.n, c.r);
		for (std::size_t i = 0 ; i < closest.size(); i++) {
			MetaContact mcontact;
			if (cylEdgeTubeEdgeMakeContact(mcontact,c,false,true,closest[i].second,closest[i].first,threshold)) {
				mcontacts.push_back(mcontact);
				found = true;
			}
		}
	}
	// Cyl bottom - Tube bottom
	if ((cylRegion == CYLBOTTOMEDGE || cylRegion == CYLALL) && (tubeRegion == TUBEBOTTOMEDGE || tubeRegion == TUBEALL)) {
		//std::cout << " - - cd: 9 cyl bottom tube bottom: cylEdgeTubeEdge()" << std::endl;
		const std::vector<std::pair<Vector3D<>, Vector3D<> > > closest = GeometricUtil::closestPointsOnCircleToCircle(c.tBottom.c, c.tBottom.n, c.R, c.cBottom.c, c.cBottom.n, c.r);
		for (std::size_t i = 0 ; i < closest.size(); i++) {
			MetaContact mcontact;
			if (cylEdgeTubeEdgeMakeContact(mcontact,c,false,false,closest[i].second,closest[i].first,threshold)) {
				mcontacts.push_back(mcontact);
				found = true;
			}
		}
	}
	return found;
}

ContactStrategyCylinderTube::Characteristics ContactStrategyCylinderTube::getCharacteristics(rw::common::Ptr<Cylinder> peg, Transform3D<> pegPose, rw::common::Ptr<Tube> tube, Transform3D<> holePose) {
	//Cylinder* pegP = static_cast<Cylinder*>(peg.get());
	//Tube* holeP = static_cast<Tube*>(tube.get());
	Cylinder* pegP = peg.get();
	Tube* holeP = tube.get();

	// First some characteristic points are found on the tube and cylinder
	Characteristics characteristics;
	characteristics.R = holeP->getInnerRadius();
	characteristics.r = pegP->getRadius();
	characteristics.L = holeP->getHeight();
	characteristics.l = pegP->getHeight();
	characteristics.pc = pegPose.P();
	characteristics.pt = holePose.P();

	// Continue to find characteristics
	characteristics.dc = pegPose.R().getCol(2);
	characteristics.dt = holePose.R().getCol(2);
	characteristics.proj_pc = GeometricUtil::projectPointOnPlane(characteristics.pc, characteristics.pt, characteristics.dt);

	// Edge definitions (circles):
	characteristics.cTop = Characteristics::Circle(characteristics.pc+characteristics.dc*characteristics.l/2.,characteristics.dc);
	characteristics.cBottom = Characteristics::Circle(characteristics.pc-characteristics.dc*characteristics.l/2.,-characteristics.dc);
	characteristics.tTop = Characteristics::Circle(characteristics.pt+characteristics.dt*characteristics.L/2.,characteristics.dt);
	characteristics.tBottom = Characteristics::Circle(characteristics.pt-characteristics.dt*characteristics.L/2.,-characteristics.dt);

	{
		const Vector3D<> chCross = normalize(cross(cross(characteristics.dt,characteristics.dc),characteristics.dc));
		characteristics.pcTopHigh = characteristics.cTop.c-chCross*characteristics.r;
		characteristics.pcTopLow = characteristics.cTop.c+chCross*characteristics.r;
		characteristics.pcBotHigh = characteristics.cBottom.c-chCross*characteristics.r;
		characteristics.pcBotLow = characteristics.cBottom.c+chCross*characteristics.r;
	}

	{
		const Vector3D<> chCross = normalize(cross(cross(characteristics.dc,characteristics.dt),characteristics.dt));
		characteristics.ptTopHigh = characteristics.tTop.c-chCross*characteristics.R;
		characteristics.ptTopLow = characteristics.tTop.c+chCross*characteristics.R;
		characteristics.ptBotHigh = characteristics.tBottom.c-chCross*characteristics.R;
		characteristics.ptBotLow = characteristics.tBottom.c+chCross*characteristics.R;
	}

	return characteristics;
}

std::vector<ContactStrategyCylinderTube::MetaContact> ContactStrategyCylinderTube::findContacts(const Characteristics& characteristics, double threshold) {
	std::vector<MetaContact> res;

	// Make an early broad-phase bounding sphere test
	{
		const double boundingSphere_R = sqrt(characteristics.L/2.*characteristics.L/2.+characteristics.R*characteristics.R);
		const double boundingSphere_r = sqrt(characteristics.l/2.*characteristics.l/2.+characteristics.r*characteristics.r);
		if ((characteristics.pc-characteristics.pt).norm2() > boundingSphere_R+boundingSphere_r+threshold)
			return res;
	}

	//std::cout << "findContacts (no update)" << std::endl;
	// Case 7+8
	cylEdgeTubeSurface(characteristics, res, threshold);
	//std::cout << " - cylEdgeTubeSurface: " << res.size() << std::endl;
	// Case 3
	cylSurfaceTubeEdge(characteristics, res, threshold);
	//std::cout << " - cylSurfaceTubeEdge: " << res.size() << std::endl;
	// Case 9
	if (characteristics.L > characteristics.l) {
		// tube is longest
		if (dot(characteristics.pc-characteristics.pt,characteristics.dt) > 0) {
			if ((characteristics.tTop.c-characteristics.cTop.c).norm2() < (characteristics.tTop.c-characteristics.cBottom.c).norm2())
				cylEdgeTubeEdge(characteristics, res, threshold, CYLTOPEDGE, TUBETOPEDGE);
			else
				cylEdgeTubeEdge(characteristics, res, threshold, CYLBOTTOMEDGE, TUBETOPEDGE);
		} else {
			if ((characteristics.tBottom.c-characteristics.cTop.c).norm2() < (characteristics.tBottom.c-characteristics.cBottom.c).norm2())
				cylEdgeTubeEdge(characteristics, res, threshold, CYLTOPEDGE, TUBEBOTTOMEDGE);
			else
				cylEdgeTubeEdge(characteristics, res, threshold, CYLBOTTOMEDGE, TUBEBOTTOMEDGE);
		}
	} else {
		// cylinder is longest
		if (dot(characteristics.pt-characteristics.pc,characteristics.dc) > 0) {
			if ((characteristics.tTop.c-characteristics.cTop.c).norm2() < (characteristics.tBottom.c-characteristics.cTop.c).norm2())
				cylEdgeTubeEdge(characteristics, res, threshold, CYLTOPEDGE, TUBETOPEDGE);
			else
				cylEdgeTubeEdge(characteristics, res, threshold, CYLTOPEDGE, TUBEBOTTOMEDGE);
		} else {
			if ((characteristics.tTop.c-characteristics.cBottom.c).norm2() < (characteristics.tBottom.c-characteristics.cBottom.c).norm2())
				cylEdgeTubeEdge(characteristics, res, threshold, CYLBOTTOMEDGE, TUBETOPEDGE);
			else
				cylEdgeTubeEdge(characteristics, res, threshold, CYLBOTTOMEDGE, TUBEBOTTOMEDGE);
		}
	}
	//cylEdgeTubeEdge(characteristics, res, threshold);
	//std::cout << " - cylEdgeTubeEdge: " << res.size() << std::endl;
	// Case 6
	cylPlaneTubeEdge(characteristics, res, threshold);
	//std::cout << " - cylPlaneTubeEdge: " << res.size() << std::endl;

	std::vector<MetaContact> resFiltered;

	if (res.size() > 0) {
		//std::cout << "res found: " << res.size() << std::endl;
		bool penetration = false;
		for (std::size_t i = 0; i < res.size(); i++) {
			//std::cout << "new contact " << i << " - " << res[i].cylinderRegion << " " << res[i].tubeRegion << std::endl;
			if (res[i].contact.getDepth() > 0)
				penetration = true;
		}
		if (penetration) {
			//std::cout << "penetrates" << std::endl;
			MetaContact best = res[0];
			for (std::size_t k = 1; k < res.size(); k++) {
				if (res[k].contact.getDepth() > 0 && res[k].contact.getDepth() < best.contact.getDepth())
					best = res[k];
			}
			for (std::size_t k = 0; k < res.size(); k++) {
				if (res[k].cylinderRegion == best.cylinderRegion && res[k].tubeRegion == best.tubeRegion && res[k].contact.getDepth() > 0)
					resFiltered.push_back(res[k]);
			}
		} else {
			//std::cout << "no penetration" << std::endl;
			// No penetration
			MetaContact best = res[0];
			for (std::size_t k = 1; k < res.size(); k++) {
				if (res[k].contact.getDepth() > best.contact.getDepth())
					best = res[k];
			}
			resFiltered.push_back(best);
			//std::cout << "no penetration done" << std::endl;
		}
	}
	//std::cout << " - filtered: " << resFiltered.size() << std::endl;

	return resFiltered;
}

std::vector<ContactStrategyCylinderTube::MetaContact> ContactStrategyCylinderTube::updateContacts(const Characteristics& characteristics, std::vector<TrackInfo>& tinfos, bool findNew, SimulatorLogScope* log) {
	std::vector<MetaContact> res;

	LogMessage::Ptr msg = NULL;
	if (log != NULL) {
		msg = ownedPtr(new LogMessage(log));
		msg->setFilename(__FILE__);
		msg->setLine(__LINE__);
		msg->setDescription("Internal Update");
		log->appendChild(msg);
	}

	//std::cout << "updateContacts (find new: " << findNew << ")" << std::endl;

	bool case1 = false;
	bool case2 = false;
	bool case3t = false;
	bool case3b = false;
	bool case4t = false;
	bool case4b = false;
	bool case5t = false;
	bool case5b = false;
	bool case6tt = false;
	bool case6tb = false;
	bool case6bt = false;
	bool case6bb = false;
	bool case7t = false;
	bool case7b = false;
	bool case8t = false;
	bool case8b = false;
	bool case9tt = false;
	bool case9tb = false;
	bool case9bt = false;
	bool case9bb = false;

	//std::cout << "Regions pairs: " << tinfos.size() << std::endl;
	BOOST_FOREACH(const TrackInfo& tinfo, tinfos) {
		if (tinfo.cylinderRegion == CYLALL)
			RW_THROW("The CYLALL region not allowed in tracked info.");
		if (tinfo.tubeRegion == TUBEALL)
			RW_THROW("The TUBEALL region not allowed in tracked info.");
		if (!case1)
			case1 = tinfo.cylinderRegion == SURFACE && tinfo.tubeRegion == INNERSURFACE;
		if (!case2)
			case2 = tinfo.cylinderRegion == SURFACE && tinfo.tubeRegion == OUTERSURFACE;
		if (!case3t)
			case3t = tinfo.cylinderRegion == SURFACE && tinfo.tubeRegion == TUBETOPEDGE;
		if (!case3b)
			case3b = tinfo.cylinderRegion == SURFACE && tinfo.tubeRegion == TUBEBOTTOMEDGE;
		if (!case4t)
			case4t = tinfo.cylinderRegion == TOPPLANE && tinfo.tubeRegion == INNERSURFACE;
		if (!case4b)
			case4b = tinfo.cylinderRegion == BOTTOMPLANE && tinfo.tubeRegion == INNERSURFACE;
		if (!case5t)
			case5t = tinfo.cylinderRegion == TOPPLANE && tinfo.tubeRegion == OUTERSURFACE;
		if (!case5b)
			case5b = tinfo.cylinderRegion == BOTTOMPLANE && tinfo.tubeRegion == OUTERSURFACE;
		if (!case6tt)
			case6tt = tinfo.cylinderRegion == TOPPLANE && tinfo.tubeRegion == TUBETOPEDGE;
		if (!case6tb)
			case6tb = tinfo.cylinderRegion == TOPPLANE && tinfo.tubeRegion == TUBEBOTTOMEDGE;
		if (!case6bt)
			case6bt = tinfo.cylinderRegion == BOTTOMPLANE && tinfo.tubeRegion == TUBETOPEDGE;
		if (!case6bb)
			case6bb = tinfo.cylinderRegion == BOTTOMPLANE && tinfo.tubeRegion == TUBEBOTTOMEDGE;
		if (!case7t)
			case7t = tinfo.cylinderRegion == CYLTOPEDGE && tinfo.tubeRegion == INNERSURFACE;
		if (!case7b)
			case7b = tinfo.cylinderRegion == CYLBOTTOMEDGE && tinfo.tubeRegion == INNERSURFACE;
		if (!case8t)
			case8t = tinfo.cylinderRegion == CYLTOPEDGE && tinfo.tubeRegion == OUTERSURFACE;
		if (!case8b)
			case8b = tinfo.cylinderRegion == CYLBOTTOMEDGE && tinfo.tubeRegion == OUTERSURFACE;
		if (!case9tt)
			case9tt = tinfo.cylinderRegion == CYLTOPEDGE && tinfo.tubeRegion == TUBETOPEDGE;
		if (!case9tb)
			case9tb = tinfo.cylinderRegion == CYLTOPEDGE && tinfo.tubeRegion == TUBEBOTTOMEDGE;
		if (!case9bt)
			case9bt = tinfo.cylinderRegion == CYLBOTTOMEDGE && tinfo.tubeRegion == TUBETOPEDGE;
		if (!case9bb)
			case9bb = tinfo.cylinderRegion == CYLBOTTOMEDGE && tinfo.tubeRegion == TUBEBOTTOMEDGE;
	}

	const bool case3 = case3t || case3b;
	const bool case4 = case4t || case4b;
	const bool case5 = case5t || case5b;
	const bool case6 = case6tt || case6tb || case6bt || case6bb;
	const bool case7 = case7t || case7b;
	const bool case8 = case8t || case8b;
	const bool case9 = case9tt || case9tb || case9bt || case9bb;

	if (msg != NULL) {
		msg->stream() << "Tracked: " << tinfos.size() << std::endl;
		BOOST_FOREACH(const TrackInfo& tinfo, tinfos) {
			msg->stream() << " - cyl: " << tinfo.cylinderRegion << " tube: " << tinfo.tubeRegion << std::endl;
		}
		if (case1)
			msg->stream() << "Tracked Case 1" << std::endl;
		if (case2)
			msg->stream() << "Tracked Case 2" << std::endl;
		if (case3)
			msg->stream() << "Tracked Case 3" << std::endl;
		if (case4)
			msg->stream() << "Tracked Case 4" << std::endl;
		if (case5)
			msg->stream() << "Tracked Case 5" << std::endl;
		if (case6)
			msg->stream() << "Tracked Case 6" << std::endl;
		if (case7)
			msg->stream() << "Tracked Case 7" << std::endl;
		if (case8)
			msg->stream() << "Tracked Case 8" << std::endl;
		if (case9)
			msg->stream() << "Tracked Case 9" << std::endl;
	}

	// First check that there is a valid combination
	if (case1)
		RW_THROW("A contact between the inner surface of the tube and the cylinders rounded surface should be handled as a combination of case 3 & 7!");
	if (case2)
		RW_THROW("A contact between the outer surface of the tube and the cylinders rounded surface should be handled as a combination of case 3 & 8!");
	if (case4)
		RW_THROW("The end-planes of the cylinder should never be able to be in contact with the inner surface of the tube!");
	if (case5)
		RW_THROW("A contact between the cylinder end-planes and the outer surface of the tube should be handled as a combination of case 3 & 8!");
	if (case7 && case8)
		RW_THROW("There should not be a contact both on the outside and inside of the tube!");
	if (case8t && case8b)
		RW_THROW("Both the top and bottom of the cylinder can not be in contact with the outside of the tube at the same time!");
	if (case3) {
		if (case6)
			RW_THROW("The end-planes of the cylinder can not be in contact with the edges of the tube, when there is a contact between the cylinders rounded surface and an edge of the tube!");
		if ((case3t && case3b) && (case7 || case8 || case9))
			RW_THROW("If there is a contact between both the top and bottom edges of the tube to the round surface of the cylinder, there can be no more contacts!");
		if (case7t && case7b)
			RW_THROW("There can only be one contact on the inside of the tube if there is a contact at the edge of the tube to the round surface of the cylinder!");
		if (case7 && case9)
			RW_THROW("When there are a contact inside the tube and at the edge, there can not be a contact between the edges at the same time!");
		if (case8t && case8b)
			RW_THROW("There can only be one contact on the outside of the tube if there is a contact at the edge of the tube to the round surface of the cylinder!");
		if (case8 && case9)
			RW_THROW("When there are a contact outisde the tube and at the edge, there can not be a contact between the edges at the same time!");
		if ((case9tt && case9tb) || (case9tt && case9bt) || (case9tt && case9bb) ||
				(case9tb && case9bt) || (case9tb && case9bb) ||
				(case9bt && case9bb))
			RW_THROW("When there is a contact between the edge of the tube and the round surface of the cylinder, there can only be one pair of edges in contact at the same time!");
	} else if (case6) {
		if ((case6tt && case6tb) || (case6tt && case6bt) || (case6tt && case6bb) ||
				(case6tb && case6bt) || (case6tb && case6bb) ||
				(case6bt && case6bb))
			RW_THROW("Only one contact can exist at a time between a end-plane of the cylinder and an edge of the tube!");
		if (case7)
			RW_THROW("When the end-plane of the cylinder is in contact with the tube edge, there can not be a contact on the inside of the tube at the same time!");
		if (((case6tt || case6tb) && case8b) || ((case6bt || case6bb) && case8t))
			RW_THROW("If the top of the cylinder is in contact with an edge of the tube and at the same time in contact with the ouside of the tube, the cylinder bottom can have no contacts! - the same goes for the opposite!");
		if (case8 && case9)
			RW_THROW("Contacts for cases 6, 8 & 9 should not be there at the same time!");
		if ((case6tt && (case9bt || case9bb)) || (case6tb && (case9bt || case9bb)) || (case6bt && (case9tt || case9tb)) || (case6bb && (case9tt || case9tb)))
			RW_THROW("The contact between a cylinder end-plane and an edge of the tube, means that there can be no contacts with the opposite end of the cylinder!");
		if ((case9tt && case9tb) || (case9tt && case9bt) || (case9tt && case9bb) ||
				(case9tb && case9bt) || (case9tb && case9bb) ||
				(case9bt && case9bb))
			RW_THROW("When there is a contact between the cylinder end-plane and the edge of the tube, there can only be one pair of edges in contact at the same time!");
	} else if (case7) {
		if ((case7t && case7b) && (case8 || case9))
			RW_THROW("If both ends of the cylinder are in contact with the inside of the tube, there can be no other contacts!");
		if ((case7t && (case9tt || case9tb)) || (case7b && (case9bt || case9bb)))
			RW_THROW("When there is a contact between the cylinder and the inside of the tube, there can be a contact between a pair of edges only if it is the opposite end of the cylinder!");
		if ((case9tt && case9tb) || (case9tt && case9bt) || (case9tt && case9bb) ||
				(case9tb && case9bt) || (case9tb && case9bb) ||
				(case9bt && case9bb))
			RW_THROW("When there is a contact between the cylinder and the inside of the tube, there can only be one pair of edges in contact at the same time!");
	} else if (case8) {
		if ((case8t && (case9bt || case9bb)) || (case8b && (case9tt || case9tb)))
			RW_THROW("When there is a contact between the cylinder and the outside of the tube, there can be a contact between a pair of edges only if it is the opposite end of the cylinder!");
		if ((case9tt && case9tb) || (case9tt && case9bt) || (case9tt && case9bb) ||
				(case9tb && case9bt) || (case9tb && case9bb) ||
				(case9bt && case9bb))
			RW_THROW("When there is a contact between the cylinder and the outside of the tube, there can only be one pair of edges in contact at the same time!");
	} else if (case9) {
		unsigned int edgeContacts = 0;
		if (case9tt)
			edgeContacts++;
		if (case9bt)
			edgeContacts++;
		if (case9tb)
			edgeContacts++;
		if (case9bb)
			edgeContacts++;
		if (edgeContacts > 2)
			RW_THROW("There can maximum be two different edge pairs in contact at a time!");
	}

	std::vector<double> caseInfos;
	for (std::size_t i = 0; i < tinfos.size(); i++) {
		caseInfos.push_back(tinfos[i].caseInfo);
	}

	bool handled = false;
	// Now handle the remaining valid combinations
	if (case3) {
		if (case3t && case3b) {
		} else if (case7) {
			if (msg != NULL)
				msg->stream() << "Case 3+7" << std::endl;
			RW_ASSERT(tinfos.size() == 2);
			RW_ASSERT(tinfos[0].cylinderRegion == SURFACE);
			RW_ASSERT(tinfos[0].tubeRegion == TUBETOPEDGE || tinfos[0].tubeRegion == TUBEBOTTOMEDGE);
			RW_ASSERT(tinfos[1].cylinderRegion == CYLTOPEDGE || tinfos[1].cylinderRegion == CYLBOTTOMEDGE);
			RW_ASSERT(tinfos[1].tubeRegion == INNERSURFACE);
			const bool contactFirst = cylSurfaceTubeEdge(characteristics, res, -1, tinfos[0].tubeRegion);
			RW_ASSERT(res.size() <= 1);
			if (msg != NULL)
				msg->stream() << "Res: " << res.size() << std::endl;
			cylEdgeTubeSurface(characteristics, res, -1, tinfos[1].cylinderRegion, tinfos[1].tubeRegion);
			if (msg != NULL)
				msg->stream() << "Res: " << res.size() << std::endl;
			RW_ASSERT(res.size() <= 2);
			if (res.size() == 0) {
				tinfos.clear();
			} else if (res.size() == 1) {
				if (contactFirst)
					tinfos = std::vector<TrackInfo>(1,tinfos[0]);
				else
					tinfos = std::vector<TrackInfo>(1,tinfos[1]);
			}
			if(res.size() != 2)
				RW_THROW("STOP");
			handled = true;
		} else if (case8) {
		} else if (case9) {
		} else {
			//std::cout << "Case 3 once only: " << case3t << " " << case3b << std::endl;
			RW_ASSERT(tinfos.size() == 1);
			cylSurfaceTubeEdge(characteristics, res, -1, tinfos[0].tubeRegion);
			if (res.size() == 0)
				tinfos.clear();
			if (res.size() > 1)
				RW_THROW("case 3 with two contacts");
			if (findNew) {
				// Check case 7
				std::vector<MetaContact> newContacts;
				cylEdgeTubeSurface(characteristics, newContacts, 0, CYLALL, TUBEALL);
				BOOST_FOREACH(const MetaContact& mc, newContacts) {
					res.push_back(mc);
					TrackInfo ti;
					ti.cylinderRegion = mc.cylinderRegion;
					ti.tubeRegion = mc.tubeRegion;
					tinfos.push_back(ti);
				}
				if (msg != NULL && newContacts.size() > 0)
					msg->stream() << " - added contacts from case 7" << std::endl;
			}
			handled = true;
		}
	} else if (case6) {
		if (case8) {
		} else if (case9) {
		} else {
			//std::cout << "Case 6 only: " << case6tt << " " << case6tb << " " << case6bt << " " << case6bb << std::endl;
			RW_ASSERT(tinfos.size() == 1);
			cylPlaneTubeEdge(characteristics, res, -1, tinfos[0].cylinderRegion, tinfos[0].tubeRegion);
			if (res.size() == 0)
				tinfos.clear();
			handled = true;
		}
	} else if (case7) {
		if (case7t && case7b) {
			RW_THROW("both case 7s");
		} else if (case9) {
			RW_THROW("case 7 AND 9");
		} else {
			RW_ASSERT(tinfos.size() == 1);
			const TrackInfo tmpInfo = tinfos[0];
			tinfos.clear();
			//std::cout << "Case 7 ONCE only: " << case7t << " " << case7b << std::endl;
			const CylinderRegion cylRegion = tmpInfo.cylinderRegion;
			if (findNew) {
				// Test for case 3
				std::vector<MetaContact> newContacts;
				const bool sameDir = dot(characteristics.dc,characteristics.dt) > 0;
				TubeRegion region;
				if ((cylRegion == CYLBOTTOMEDGE && sameDir) || (cylRegion == CYLTOPEDGE && !sameDir))
					region = TUBETOPEDGE;
				else
					region = TUBEBOTTOMEDGE;
				cylSurfaceTubeEdge(characteristics, newContacts, 0, region);
				if (newContacts.size() > 0) {
					res.push_back(newContacts[0]);
					tinfos.resize(1);
					tinfos.back().cylinderRegion = cylRegion;
					tinfos.back().tubeRegion = region;
				}
			}
			std::vector<MetaContact> updatedContacts;
			cylEdgeTubeSurface(characteristics, updatedContacts, -1, cylRegion, tmpInfo.tubeRegion);
			if (updatedContacts.size() > 0) {
				res.push_back(updatedContacts[0]);
				tinfos.push_back(tmpInfo);
			}
			handled = true;
		}
	} else if (case8) {
		if (case8t && case8b) {
			RW_THROW("Case 8 twice");
		} else if (case9) {
			RW_THROW("Case 8 AND 9");
		} else {
			RW_THROW("Case 8 once");
		}
	} else if (case9) {
		//std::cout << " - Case 9 only: " << case9tt << " " << case9tb << " " << case9bt << " " << case9bb << std::endl;
		RW_ASSERT(tinfos.size() > 0 && tinfos.size() <= 2);

		bool moveTo7 = false;
		const Characteristics& c = characteristics;
		if (case9tt) {
			if (dot(c.pcTopHigh-c.pt,c.dt) < c.l/2)
				moveTo7 = true;
		} else if (case9tb) {
			if (dot(c.pcTopLow-c.pt,c.dt) > -c.l/2)
				moveTo7 = true;
		} else if (case9bt) {
			if (dot(c.pcBotHigh-c.pt,c.dt) < c.l/2)
				moveTo7 = true;
		} else if (case9bb) {
			if (dot(c.pcBotLow-c.pt,c.dt) > -c.l/2)
				moveTo7 = true;
		}
		std::vector<TrackInfo> tmpInfo = tinfos;
		tinfos.clear();
		if (moveTo7) {
			if (msg != NULL)
				msg->stream() << " - moving from case 9 to case 7" << std::endl;
			const CylinderRegion cylRegion = tmpInfo[0].cylinderRegion;
			std::vector<MetaContact> newContacts;
			if (findNew) {
				// Test for case 3
				const bool sameDir = dot(characteristics.dc,characteristics.dt) > 0;
				TubeRegion region;
				if ((cylRegion == CYLBOTTOMEDGE && sameDir) || (cylRegion == CYLTOPEDGE && !sameDir))
					region = TUBETOPEDGE;
				else
					region = TUBEBOTTOMEDGE;
				cylSurfaceTubeEdge(characteristics, newContacts, 0, region);
				RW_ASSERT(newContacts.size() <= 1);
				if (newContacts.size() > 0) {
					res.push_back(newContacts[0]);
					tinfos.resize(1);
					tinfos.back().cylinderRegion = SURFACE;
					tinfos.back().tubeRegion = region;
					if (msg != NULL)
						msg->stream() << " - added case 3 contact" << std::endl;
				}
			}
			std::vector<MetaContact> updatedContacts;
			cylEdgeTubeSurface(characteristics, updatedContacts, -1, cylRegion, INNERSURFACE);
			if (updatedContacts.size() == 0)
				RW_THROW("STOP");
			if (updatedContacts.size() > 0) {
				res.push_back(updatedContacts[0]);
				tinfos.push_back(tmpInfo[0]);
				tinfos.back().tubeRegion = INNERSURFACE;
				if (msg != NULL && newContacts.size() > 0)
					msg->stream() << " - added case 7 contact" << std::endl;
			}
		} else {
			std::vector<MetaContact> tmpRes;
			cylEdgeTubeEdge(characteristics, tmpRes, -1, tinfos[0].cylinderRegion, tinfos[0].tubeRegion);
			res.clear();
			if (msg != NULL)
				msg->stream() << " - edge to edge contacts found: " << tmpRes.size() << std::endl;
			if (tmpRes.size() == 2) {
				if (tmpRes[0].caseInfo*tmpRes[1].caseInfo > 0) {
					RW_THROW("ContactStrategyCylinderTube (updateContacts): found two edge-edge contacts, but both are lying on the same side of the center line - this is unexpected.");
				}
			}
			std::vector<bool> matched(tmpRes.size(),false);
			for (std::size_t i = 0; i < tmpInfo.size(); i++) {
				for (std::size_t j = 0; j < tmpRes.size(); j++) {
					if (matched[j])
						continue;
					//std::cout << " tmpRes caseInfo match: " << tmpInfo[i].caseInfo << " " << tmpRes[j].caseInfo << std::endl;
					if (tmpRes[j].caseInfo*tmpInfo[i].caseInfo > 0) {
						res.push_back(tmpRes[j]);
						tinfos.push_back(tmpInfo[i]);
						matched[j] = true;
						break;
					}
				}
			}
			if (findNew) {
				for (std::size_t j = 0; j < tmpRes.size(); j++) {
					if (!matched[j]) {
						if (tmpRes[j].contact.getDepth() > 0) {
							//std::cout << " tmpRes no match (adding new with penetration): " << tmpRes[j].caseInfo << std::endl;
							res.push_back(tmpRes[j]);
							TrackInfo newInfo = tmpInfo[0];
							newInfo.userData = NULL;
							newInfo.caseInfo = tmpRes[j].caseInfo;
							tinfos.push_back(newInfo);
						}
					}
				}
			}
		}
		handled = true;
	}
	if (!handled)
		RW_THROW("The given combination is not handled in the strategy yet!");
	RW_ASSERT(tinfos.size() == res.size());

	/*if ((cylRegion == CYLTOPEDGE || cylRegion == CYLBOTTOMEDGE) && (tubeRegion == OUTERSURFACE || tubeRegion == INNERSURFACE))
		cylEdgeTubeSurface(characteristics, res, NO_THRESHOLD, cylRegion, tubeRegion);
	// Case 3
	if (cylRegion == SURFACE && (tubeRegion == TUBETOPEDGE || tubeRegion == TUBEBOTTOMEDGE))
		cylSurfaceTubeEdge(characteristics, res, NO_THRESHOLD, cylRegion, tubeRegion);
	// Case 9
	if ((cylRegion == CYLTOPEDGE || cylRegion == CYLBOTTOMEDGE) && (tubeRegion == TUBETOPEDGE || tubeRegion == TUBEBOTTOMEDGE))
		cylEdgeTubeEdge(characteristics, res, NO_THRESHOLD, cylRegion, tubeRegion);
	// Case 6
	if ((cylRegion == TOPPLANE || cylRegion == BOTTOMPLANE) && (tubeRegion == TUBETOPEDGE || tubeRegion == TUBEBOTTOMEDGE))
		cylPlaneTubeEdge(characteristics, res, NO_THRESHOLD, cylRegion, tubeRegion);*/

	//std::cout << " - update returned " << res.size() << std::endl;
	return res;
}

std::vector<Contact> ContactStrategyCylinderTube::findContactsIntersection(rw::common::Ptr<Cylinder> peg, Transform3D<> pegPose, rw::common::Ptr<Tube> tube, Transform3D<> holePose, double threshold) {
	std::vector<Contact> res;

	//Cylinder* pegP = static_cast<Cylinder*>(peg.get());
	//Tube* holeP = static_cast<Tube*>(tube.get());
	Cylinder* pegP = peg.get();
	Tube* holeP = tube.get();

	for (int dir = -1; dir < 2; dir += 2) {
		Vector3D<> pPoint = holePose.P()+holePose.R()*Vector3D<>(0,0,holeP->getHeight()/2.);
		Vector3D<> pNormal = holePose.R()*Vector3D<>(0,0,1);
		Vector3D<> cNormal = dir*(pegPose.R()*Vector3D<>(0,0,1));
		double d = dot(pPoint-pegPose.P(),pNormal)/dot(cNormal,pNormal);
		Vector3D<> cCircle = pegPose.P()+dir*(pegPose.R()*Vector3D<>(0,0,d));

		Vector3D<> pProj = GeometricUtil::projectPointOnPlane(cCircle, pPoint, pNormal);

		Vector3D<> x = normalize(cross(cNormal,pNormal));
		Vector3D<> y = normalize(cross(pNormal,x));
		double rx = pegP->getRadius();
		double ry = pegP->getRadius()/dot(cNormal,pNormal);

		std::vector<Vector3D<> > closestPoints = GeometricUtil::closestPointsOnEllipseToCircle(pPoint, pNormal, pProj, x, rx, y, ry);

		BOOST_FOREACH(Vector3D<> &vec, closestPoints) {
			Contact c;
			Vector3D<> pb = pPoint+normalize(vec-pPoint)*holeP->getInnerRadius();
			c.setTransform(inverse(pegPose)*holePose);
			c.setPointA(vec);
			c.setPointB(pb);
			c.setNormal(normalize(vec-pPoint));
			c.setDepth();
			res.push_back(c);
		}
	}

	return res;
}

std::vector<Vector3D<float> > ContactStrategyCylinderTube::getProjectedPoints(rw::common::Ptr<Cylinder> peg, Transform3D<> pegPose, rw::common::Ptr<Tube> hole, Transform3D<> holePose) {
	std::vector<Vector3D<float> > vec;

	//Cylinder* pegP = static_cast<Cylinder*>(peg.get());
	//Tube* holeP = static_cast<Tube*>(hole.get());
	Cylinder* pegP = peg.get();
	Tube* holeP = hole.get();

	Vector3D<> cCircle = pegPose.P()-pegPose.R()*Vector3D<>(0,0,pegP->getHeight()/2.);
	Vector3D<> cNormal = pegPose.R()*Vector3D<>(0,0,1);
	Vector3D<> pNormal = holePose.R()*Vector3D<>(0,0,1);
	Vector3D<> pPoint = holePose.P()-holePose.R()*Vector3D<>(0,0,holeP->getHeight()/2.);

	Vector3D<> pProj = GeometricUtil::projectPointOnPlane(cCircle, pPoint, pNormal);

	Vector3D<> x = normalize(cross(cNormal,pNormal));
	Vector3D<> y = normalize(cross(pNormal,x));
	double rx = pegP->getRadius();
	double ry = pegP->getRadius()*dot(cNormal,pNormal);

	for (double i = 0; i < 2*Pi; i += 0.1)
			vec.push_back(cast<float>(rx*x*cos(i)+ry*y*sin(i)+pProj));
	for (double i = 0; i <= 1.; i += 0.1)
			vec.push_back(cast<float>(rx*x*i+pProj));
	for (double i = 0; i <= 1.; i += 0.1)
			vec.push_back(cast<float>(ry*y*i+pProj));

	return vec;
}

std::vector<Vector3D<float> > ContactStrategyCylinderTube::getIntersectionPoints(rw::common::Ptr<Cylinder> peg, Transform3D<> pegPose, rw::common::Ptr<Tube> hole, Transform3D<> holePose) {
	std::vector<Vector3D<float> > vec;

	//Cylinder* pegP = static_cast<Cylinder*>(peg.get());
	//Tube* holeP = static_cast<Tube*>(hole.get());
	Cylinder* pegP = peg.get();
	Tube* holeP = hole.get();

	Vector3D<> pPoint = holePose.P()+holePose.R()*Vector3D<>(0,0,holeP->getHeight()/2.);
	Vector3D<> pNormal = holePose.R()*Vector3D<>(0,0,1);
	Vector3D<> cNormal = pegPose.R()*Vector3D<>(0,0,1);
	double d = dot(pPoint-pegPose.P(),pNormal)/dot(cNormal,pNormal);
	Vector3D<> cCircle = pegPose.P()+pegPose.R()*Vector3D<>(0,0,d);

	Vector3D<> pProj = GeometricUtil::projectPointOnPlane(cCircle, pPoint, pNormal);

	Vector3D<> x = normalize(cross(cNormal,pNormal));
	Vector3D<> y = normalize(cross(pNormal,x));
	double rx = pegP->getRadius();
	double ry = pegP->getRadius()/dot(cNormal,pNormal);

	for (double i = 0; i < 2*Pi; i += 0.1)
			vec.push_back(cast<float>(rx*x*cos(i)+ry*y*sin(i)+pProj));
	for (double i = 0; i <= 1.; i += 0.1)
			vec.push_back(cast<float>(rx*x*i+pProj));
	for (double i = 0; i <= 1.; i += 0.1)
			vec.push_back(cast<float>(ry*y*i+pProj));

	return vec;
}

std::vector<Vector3D<float> > ContactStrategyCylinderTube::getHolePoints(rw::common::Ptr<Tube> hole, Transform3D<> holePose) {
	std::vector<Vector3D<float> > vec;
	//Tube* holeP = static_cast<Tube*>(hole.get());
	Tube* holeP = hole.get();
	Vector3D<> pNormal = holePose.R()*Vector3D<>(0,0,1);
	Vector3D<> pPoint = holePose.P()-holePose.R()*Vector3D<>(0,0,holeP->getHeight()/2.);
	Vector3D<> x = normalize(cross(pNormal,Vector3D<>(1,0,0)));
	Vector3D<> y = normalize(cross(x,pNormal));
	double r = holeP->getInnerRadius();
	for (double i = 0; i < 2*Pi; i += 0.1)
		vec.push_back(cast<float>(r*x*cos(i)+r*y*sin(i)+pPoint));
	return vec;
}
