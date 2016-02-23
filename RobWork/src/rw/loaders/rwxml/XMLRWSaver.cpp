/*
 * XMLRWSaver.cpp
 *
 *  Created on: 10/10/2013
 *      Author: thomas
 */

#include "XMLRWSaver.hpp"

#include <RobWorkConfig.hpp>
#include <rw/common/DOMParser.hpp>

#include <rw/models/SerialDevice.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/TreeDevice.hpp>

#include <ctime>
#include <ios>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;

namespace {
static std::string getTimeString() {
	time_t now = time(0);
	tm* time = localtime(&now);
	std::stringstream datestream;
	int day = time->tm_mday;
	int month = time->tm_mon+1;
	int year = time->tm_year+1900;
	int hour = time->tm_hour;
	int min = time->tm_min;
	int sec = time->tm_sec;
	datestream << ((day < 10)? "0": "") << day << "/";
	datestream << ((month < 10)? "0": "") << month << "-";
	datestream << year << " ";
	datestream << ((hour < 10)? "0": "") << hour << ":";
	datestream << ((min < 10)? "0": "") << min << ":";
	datestream << ((sec < 10)? "0": "") << sec;
	return datestream.str();
}

static std::vector<std::string> getScopes(const std::string &name) {
	std::string restStr = name;
	std::vector<std::string> scope;
	std::size_t pos = restStr.find('.');
	while (pos != std::string::npos) {
		scope.push_back(restStr.substr(0,pos));
		restStr = restStr.substr(pos+1,std::string::npos);
		pos = restStr.find('.');
	}
	return scope;
}

static std::string getScopeFromName(const std::string &name) {
	std::string restStr = name;
	std::string scope = "";
	std::size_t pos = restStr.find('.');
	while (pos != std::string::npos) {
		if (scope == "")
			scope = restStr.substr(0,pos);
		else
			scope = scope + "." + restStr.substr(0,pos);
		restStr = restStr.substr(pos+1,std::string::npos);
		pos = restStr.find('.');
	}
	return scope;
}

static std::string removeScope(const std::string &name, const std::string curScope = "") {
	std::vector<std::string> namePart = getScopes(name);
	std::vector<std::string> scopePart = getScopes(curScope);

	// Just check that curScope matches
	for (std::size_t i = 0; i < curScope.size(); i++) {
		bool ok = false;
		if (namePart.size() >= i-1) {
			if (namePart[i] == scopePart[i]) {
				ok = true;
			}
		}
		if (!ok)
			RW_THROW("XMLRWSaver failed to remove scope " << curScope << " from name " << name);
	}

	// Then remove the scope
	std::string res = name.substr(curScope.length(),std::string::npos);
	return res;
}

static std::string toString(const Vector3D<> &vector) {
	std::stringstream sstr;
	sstr << std::fixed;
	Vector3D<> vec = vector;
	double frac;
	bool xIsZero = (std::modf(vec[0],&frac) == 0.);
	bool yIsZero = (std::modf(vec[1],&frac) == 0.);
	bool zIsZero = (std::modf(vec[2],&frac) == 0.);
	if (xIsZero && yIsZero && zIsZero)
		sstr << std::setprecision(0);
	else
		sstr << std::setprecision(10);
	sstr << vec[0] << " " << vec[1] << " " << vec[2];
	if (vec[0] == -0)
		vec[0] = 0;
	if (vec[1] == -0)
		vec[1] = 0;
	if (vec[2] == -0)
		vec[2] = 0;
	return sstr.str();
}

static std::string toString(const RPY<> &rpy) {
	std::stringstream sstr;
	sstr << std::fixed;
	Vector3D<> vec(rpy[0]*Rad2Deg,rpy[1]*Rad2Deg,rpy[2]*Rad2Deg);
	double frac;
	bool xIsZero = (std::modf(vec[0],&frac) == 0.);
	bool yIsZero = (std::modf(vec[1],&frac) == 0.);
	bool zIsZero = (std::modf(vec[2],&frac) == 0.);
	if (xIsZero && yIsZero && zIsZero)
		sstr << std::setprecision(0);
	else
		sstr << std::setprecision(5);
	if (vec[0] == -0)
		vec[0] = 0;
	if (vec[1] == -0)
		vec[1] = 0;
	if (vec[2] == -0)
		vec[2] = 0;
	sstr << vec[0] << " " << vec[1] << " " << vec[2];
	return sstr.str();
}

static void addFrame(Frame* frame, DOMElem::Ptr element, const State &state, std::string scope = "") {
	if (frame->isDAF())
		RW_THROW("XMLRWSaver can not yet handle DAF frame " << frame->getName() << ".");

	DOMElem::Ptr frameElem = element->addChild("Frame");
	frameElem->addAttribute("name")->setValue(frame->getName());
	frameElem->addAttribute("refframe")->setValue(frame->getParent()->getName());
	Transform3D<> transform = frame->getTransform(state);
	Vector3D<> pos = transform.P();
	RPY<> rpy = RPY<>(transform.R());
	if (toString(pos) != "0 0 0" || toString(rpy) != "0 0 0") {
		frameElem->addChild("Pos")->setValue(toString(pos));
		frameElem->addChild("RPY")->setValue(toString(rpy));
	}
}

static void addDrawables(Frame* frame, WorkCell::Ptr workcell, DOMElem::Ptr element, std::string scope = "") {
	BOOST_FOREACH(Object::Ptr object, workcell->getObjects()) {
		if (object->getBase() == frame) {
			// Visual Models
			BOOST_FOREACH(Model3D::Ptr model, object->getModels()) {
				DOMElem::Ptr drawElem = element->addChild("Drawable");
				drawElem->addAttribute("name")->setValue(model->getName());
				drawElem->addAttribute("refframe")->setValue(frame->getName());
				Transform3D<> transform = model->getTransform();
				if (!(transform == Transform3D<>::identity())) {
					Vector3D<> pos = transform.P();
					drawElem->addChild("Pos")->setValue(toString(pos));
					RPY<> rpy = RPY<>(transform.R());
					drawElem->addChild("RPY")->setValue(toString(rpy));
				}
				model->toGeometryData();
			}
			// Collision Models
			BOOST_FOREACH(Geometry::Ptr geometry, object->getGeometry()) {
				DOMElem::Ptr drawElem = element->addChild("Drawable");
				drawElem->addAttribute("name")->setValue(geometry->getName());
				drawElem->addAttribute("refframe")->setValue(frame->getName());
				Transform3D<> transform = geometry->getTransform();
				Vector3D<> pos = transform.P();
				RPY<> rpy = RPY<>(transform.R());
				if (toString(pos) != "0 0 0" || toString(rpy) != "0 0 0") {
					drawElem->addChild("Pos")->setValue(toString(pos));
					drawElem->addChild("RPY")->setValue(toString(rpy));
				}
				GeometryData::Ptr data = geometry->getGeometryData();
				if (data->getType() == GeometryData::SpherePrim) {
					Sphere* geoData = (data.cast<Sphere>()).get();
				} else if (data->getType() == GeometryData::BoxPrim) {
					Box::Ptr geoData = data.cast<Box>();
					DOMElem::Ptr geoElem = drawElem->addChild("Box");
					geoElem->addAttribute("x")->setValue(geoData->getParameters()[0]);
					geoElem->addAttribute("y")->setValue(geoData->getParameters()[1]);
					geoElem->addAttribute("z")->setValue(geoData->getParameters()[2]);
				} else if (data->getType() == GeometryData::CylinderPrim) {
					Cylinder* geoData = (data.cast<Cylinder>()).get();
					DOMElem::Ptr geoElem = drawElem->addChild("Cylinder");
					geoElem->addAttribute("height")->setValue(geoData->getHeight());
					geoElem->addAttribute("radius")->setValue(geoData->getRadius());
				} else if (data->getType() == GeometryData::PlainTriMesh) {
					TriMesh::Ptr geoData = data->getTriMesh();
					DOMElem::Ptr geoElem = drawElem->addChild("Polytope");
					geoElem->addAttribute("file")->setValue("unknownPlain");
				} else if (data->getType() == GeometryData::IdxTriMesh) {
					TriMesh::Ptr geoData = data->getTriMesh();
					DOMElem::Ptr geoElem = drawElem->addChild("Polytope");
					geoElem->addAttribute("file")->setValue("unknownIdx");
				} else {
					RW_THROW("XMLRWSaver can not yet save geometry of type " << data->getType() << ".");
				}
			}
		}
	}
}

static std::vector<Device::Ptr> orderDevices(const std::vector<Device::Ptr> &devices, std::map<Device::Ptr, Device::Ptr> &deviceToParent) {
	std::vector<Device::Ptr> deviceOrder;
	std::map<Device::Ptr, bool> deviceAdded;
	BOOST_FOREACH(Device::Ptr device, devices) {
		deviceAdded[device] = false;
	}
	BOOST_FOREACH(Device::Ptr device, devices) {
		if (!deviceAdded[device]) {
			std::stack<Device::Ptr> depStack;
			depStack.push(device);
			while (!depStack.empty()) {
				Device::Ptr curDev = depStack.top();
				Device::Ptr parent = deviceToParent[curDev];
				if (parent != NULL && !deviceAdded[parent]) {
					depStack.push(parent);
				} else {
					deviceOrder.push_back(curDev);
					deviceAdded[curDev] = true;
					depStack.pop();
				}
			}
		}
	}
	return deviceOrder;
}

static std::vector<Frame*> orderFrames(const std::vector<Frame*> &frames) {
	std::vector<Frame*> frameOrder;
	std::map<Frame*, bool> frameAdded;
	BOOST_FOREACH(Frame* frame, frames) {
		frameAdded[frame] = false;
	}
	BOOST_FOREACH(Frame* frame, frames) {
		if (!frameAdded[frame]) {
			std::stack<Frame*> depStack;
			depStack.push(frame);
			while (!depStack.empty()) {
				Frame* curFrame = depStack.top();
				Frame* parent = curFrame->getParent();
				bool isInLis = false;
				BOOST_FOREACH(Frame* fr, frames) {
					if (parent->getName() == fr->getName())
						isInLis = true;
				}
				if (parent != NULL && isInLis && !frameAdded[parent]) {
					depStack.push(parent);
				} else {
					frameOrder.push_back(curFrame);
					frameAdded[curFrame] = true;
					depStack.pop();
				}
			}
		}
	}
	return frameOrder;
}

static void addDevice(Device::Ptr device, const std::vector<Frame*> &frames, DOMElem::Ptr element, WorkCell::Ptr wc, const State &state) {
	DOMElem::Ptr deviceElem;
	std::vector<Frame*> frameOrder = orderFrames(frames);
	if (SerialDevice::Ptr dev = device.cast<SerialDevice>()) {
		deviceElem = element->addChild("SerialDevice");
	} else if (ParallelDevice::Ptr dev = device.cast<ParallelDevice>()) {
		deviceElem = element->addChild("ParallelDevice");
	} else if (TreeDevice::Ptr dev = device.cast<TreeDevice>()) {
		deviceElem = element->addChild("TreeDevice");

		// Construct map to know if frame is joint
		std::cout << "TreeDevice: " << dev->getName() << std::endl;
		BOOST_FOREACH(Frame* frame, frameOrder) {
			std::cout << " Adding frame: " << frame->getName() << std::endl;
			addFrame(frame, deviceElem, state, device->getName());
			addDrawables(frame, wc, deviceElem, device->getName());
		}
	} else {
		RW_THROW("The device " << device->getName() << " if of unsupported type. RWXML could not be saved.");
	}
	deviceElem->addAttribute("name")->setValue(device->getName());
}
}

void XMLRWSaver::save(WorkCell::Ptr workcell, const std::string& filename) {
	DOMParser::Ptr parser = DOMParser::make();
	DOMElem::Ptr rootElem = parser->getRootElement();

	const State &state = workcell->getDefaultState();

	// Add Header
	rootElem->addChild("<xmlcomment>")->setValue(" Generated by RobWork " + std::string(RW_VERSION) + " revision " + std::string(RW_REVISION) + " ");
	rootElem->addChild("<xmlcomment>")->setValue(" Date: " + getTimeString() + " ");
	if (workcell->getFilePath() != "")
		rootElem->addChild("<xmlcomment>")->setValue(" Original file path: " + workcell->getFilePath() + " ");
	if (workcell->getFilename() != "")
		rootElem->addChild("<xmlcomment>")->setValue(" Original filename: " + workcell->getFilename() + " ");

	// Add WorkCell element
	DOMElem::Ptr wcElem = rootElem->addChild("WorkCell");
	wcElem->addAttribute("name")->setValue(workcell->getName());

	// To get a meaningful ordering in the WorkCell file, we want to sort the elements such that each
	// element (device, frame, joint etc) depends on previously defined elements.

	// Construct map for the frames in each device
	std::map<Device::Ptr, std::vector<Frame*> > deviceToFrames;
	BOOST_FOREACH(Frame* frame, workcell->getFrames()) {
		std::string scope = getScopeFromName(frame->getName());
		Device::Ptr device;
		if (scope == "")
			device = NULL;
		else {
			device = workcell->findDevice(scope);
			while (device == NULL && scope != "") {
				scope = getScopeFromName(scope);
				device = workcell->findDevice(scope);
			}
		}
		if (frame->getName() != workcell->getWorldFrame()->getName())
			deviceToFrames[device].push_back(frame);
	}

	// Construct map to keep track of if the frames has been added
	std::map<Device::Ptr, std::map<Frame*, bool> > framesAdded;
	BOOST_FOREACH(Device::Ptr device, workcell->getDevices()) {
		BOOST_FOREACH(Frame* frame, deviceToFrames[device]) {
			framesAdded[device][frame] = false;
		}
	}
	BOOST_FOREACH(Frame* frame, deviceToFrames[NULL]) {
		framesAdded[NULL][frame] = false;
	}

	// Find device dependencies
	std::map<Device::Ptr, Device::Ptr> deviceToParent;
	std::map<Device::Ptr, std::vector<Device::Ptr> > deviceToChild;
	BOOST_FOREACH(Device::Ptr device, workcell->getDevices()) {
		deviceToParent[device] = NULL;
		Frame* base = device->getBase();
		Frame* parent = base->getParent();
		while (parent != NULL) {
			std::string scope = getScopeFromName(parent->getName());
			if (scope != "") {
				Device::Ptr found = workcell->findDevice(scope);
				if (found == NULL) {
					RW_THROW("Could not find the device with name " << scope << " inferred from scope name of frame << " << parent->getName() << ".");
				} else {
					deviceToParent[device] = found;
					deviceToChild[found].push_back(device);
				}
				parent = NULL;
			} else {
				parent = parent->getParent();
			}
		}
	}

	// Determine the linear device ordering (in case devices are nested)
	std::vector<Device::Ptr> deviceOrder = orderDevices(workcell->getDevices(), deviceToParent);

	// Add each device and insert the scope-free frames as they are needed.
	BOOST_FOREACH(Device::Ptr device, deviceOrder) {
		Frame* base = device->getBase();
		Device::Ptr parentDev = deviceToParent[device];

		addDrawables(workcell->getWorldFrame(),workcell,wcElem);

		wcElem->addChild("<xmlcomment>")->setValue(" Device: " + device->getName() + " ");

		// Add frames connecting device to world or parent device
		{
			std::vector<Frame*> framesToAdd;
			Frame* parent = base->getParent();
			while (parent != NULL) {
				if (parent->getName().find('.') == std::string::npos && parent != workcell->getWorldFrame()) {
					framesToAdd.push_back(parent);
					parent = parent->getParent();
				} else {
					parent = NULL;
				}
			}
			for (std::size_t i = 0; i < framesToAdd.size(); i++) {
				Frame* frame = framesToAdd[framesToAdd.size()-1-i];
				if (!framesAdded[NULL][frame]) {
					framesAdded[NULL][frame] = true;
					addFrame(frame,wcElem,state);
					addDrawables(frame,workcell,wcElem);
				}
			}
		}

		// Add the device
		addDevice(device,deviceToFrames[device],wcElem,workcell,state);

		// Add the workcell frames referring to this device (but not the ones connecting to child devices)
		// First mark all frames refering to the device
		std::map<Frame*, bool> useFrame;
		BOOST_FOREACH(Frame* frame, deviceToFrames[NULL]) {
			useFrame[frame] = false;
			Frame* parent = frame->getParent();
			std::vector<Frame*> candidates;
			while (parent != NULL) {
				std::string scope = getScopeFromName(parent->getName());
				if (scope == "" && parent != workcell->getWorldFrame()) {
					candidates.push_back(parent);
					parent = parent->getParent();
				} else if (scope != device->getName() || parent == workcell->getWorldFrame()) {
					candidates.clear();
					parent = NULL;
				} else {
					parent = NULL;
				}
			}
			for (std::size_t i = 0; i < candidates.size(); i++) {
				useFrame[candidates[i]] = true;
			}
		}
		// Then unmark the frames in the chain between parent and current device
		BOOST_FOREACH(Device::Ptr child, deviceToChild[device]) {
			Frame* base = child->getBase();
			Frame* parent = base->getParent();
			std::vector<Frame*> candidates;
			while (parent != NULL) {
				std::string scope = getScopeFromName(parent->getName());
				if (scope == "" && parent != workcell->getWorldFrame()) {
					candidates.push_back(parent);
					parent = parent->getParent();
				} else if (scope != device->getName() || parent == workcell->getWorldFrame()) {
					candidates.clear();
					parent = NULL;
				} else {
					parent = NULL;
				}
			}
			for (std::size_t i = 0; i < candidates.size(); i++) {
				useFrame[candidates[i]] = false;
			}
		}
		BOOST_FOREACH(Frame* frame, deviceToFrames[NULL]) {
			if (useFrame[frame]) {
				framesAdded[NULL][frame] = true;
				addFrame(frame,wcElem,state);
				addDrawables(frame,workcell,wcElem);
			}
		}
	}

	wcElem->addChild("<xmlcomment>")->setValue(std::string(" Device independent frames "));
	// Add the remaining free frames that were not referred to by any devices and did not depend on any devices
	BOOST_FOREACH(Frame* frame, deviceToFrames[NULL]) {
		if (!framesAdded[NULL][frame]) {
			std::cout << "frame: " << frame->getName() << std::endl;
			addFrame(frame,wcElem,state);
			addDrawables(frame,workcell,wcElem);
		}
	}

	parser->save(filename);
}
