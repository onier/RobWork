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

#include "URDFLoader.hpp"
#include "URDFParser.hpp"
#include "URDFModel.hpp"

#include <rw/graphics/Model3DFactory.hpp>
#include <rw/graphics/SceneDescriptor.hpp>

#include <rw/loaders/GeometryFactory.hpp>

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/StateStructure.hpp>

#include <rw/math/Transform3D.hpp>

#include <rw/models/Joint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;

namespace {
struct DummySetup {
public:
	DummySetup():
		scene(ownedPtr(new SceneDescriptor()))
	{
		tree = new StateStructure();
		world = tree->getRoot();
		frameMap[world->getName()] = world;
	}
	StateStructure *tree;
	Frame *world;

	std::map<std::string, Frame*> frameMap;
	std::map<std::string, Frame*> toParentMap;
	std::map<std::string, std::vector<Frame*> > toChildMap;

	Device::Ptr device;

	std::map<Frame*, RigidObject::Ptr> objectMap;

	SceneDescriptor::Ptr scene;
};

static void addAllFrames(const URDFModel &model, std::map<std::string, Frame*> &frameMap) {
	// Add frames (urdf: links)
	BOOST_FOREACH(const URDFModel::Link &link, model.links) {
		std::string scopedName = model.name + "." + link.name;
		Frame* frame = new FixedFrame(scopedName, Transform3D<>::identity());
		frameMap[scopedName] = frame;
	}

	// Add frames (urdf: joints)
	BOOST_FOREACH(const URDFModel::Joint &joint, model.joints) {
		std::string scopedName = model.name + "." + joint.name;
		Frame* frame;
		Transform3D<> transform = joint.origin.toTransform();
		switch (joint.type) {
		case URDFModel::Joint::REVOLUTE:
		{
			RevoluteJoint *j = new RevoluteJoint(scopedName, transform);
			j->setBounds(std::pair<Q, Q>(Q(1, joint.limit.lower), Q(1, joint.limit.upper)));
			j->setMaxVelocity(Q(1, joint.limit.velocity));
			j->setMaxAcceleration(Q(1, joint.limit.effort));
			frame = j;
			break;
		}
		case URDFModel::Joint::CONTINUOUS:
		{
			RevoluteJoint* j = new RevoluteJoint(scopedName, transform);
			j->setBounds(std::pair<Q, Q>(Q(1, -2*Pi), Q(1, 2*Pi)));
			j->setMaxVelocity(Q(1, joint.limit.velocity));
			j->setMaxAcceleration(Q(1, joint.limit.effort));
			frame = j;
			break;
		}
		case URDFModel::Joint::PRISMATIC:
		{
			PrismaticJoint *j = new PrismaticJoint(scopedName, transform);
			j->setBounds(std::pair<Q, Q>(Q(1, joint.limit.lower), Q(1, joint.limit.upper)));
			j->setMaxVelocity(Q(1, joint.limit.velocity));
			j->setMaxAcceleration(Q(1, joint.limit.effort));
			frame = j;
			break;
		}
		case URDFModel::Joint::FIXED:
			frame = new FixedFrame(scopedName, joint.origin.toTransform());
			break;
		case URDFModel::Joint::FLOATING:
			RW_THROW("URDFLoader can not yet use joints of type 'floating'.");
			break;
		case URDFModel::Joint::PLANAR:
			RW_THROW("URDFLoader can not yet use joints of type 'planar'.");
			break;
		}
		frameMap[scopedName] = frame;
	}
}

static void setParentsAndChildren(const URDFModel &model, DummySetup &setup) {
	// Build child and parent maps
	BOOST_FOREACH(const URDFModel::Joint &joint, model.joints) {
		std::string scopedName = model.name + "." + joint.name;
		std::string parentName = model.name + "." + joint.parent;
		std::string childName = model.name + "." + joint.child;
		Frame* frame = setup.frameMap[scopedName];
		Frame* parentLink = setup.frameMap[parentName];
		Frame* childLink = setup.frameMap[childName];
		setup.toChildMap[parentName].push_back(frame);
		setup.toChildMap[scopedName].push_back(childLink);
		setup.toParentMap[scopedName] = parentLink;
		setup.toParentMap[childName] = frame;
	}
	bool foundBase = false;
	BOOST_FOREACH(const URDFModel::Link &link, model.links) {
		std::string scopedName = model.name + "." + link.name;
		if (setup.toParentMap.find(scopedName) == setup.toParentMap.end()) {
			Frame* linkFrame = setup.frameMap[scopedName];
			setup.toParentMap[scopedName] = setup.world;
			setup.toChildMap[setup.world->getName()].push_back(linkFrame);
			foundBase = true;
		}
	}
	if (!foundBase) {
		RW_THROW("URDFLoader: No base link found in URDF model.");
	}
}

static void addToStateStructure(Frame* frame, StateStructure* stateStructure, std::map<std::string, std::vector<Frame*> > &toChildMap) {
	const std::vector<Frame*> children = toChildMap[frame->getName()];
	BOOST_FOREACH(Frame* child, children) {
		stateStructure->addFrame(child, frame);
		addToStateStructure(child, stateStructure, toChildMap);
	}
}

static bool isJointFrame(const std::string &name, const URDFModel &model) {
	BOOST_FOREACH(const URDFModel::Joint &joint, model.joints) {
		std::string scopedName = model.name + "." + joint.name;
		if (scopedName == name)
			return true;
	}
	return false;
}

static bool hasJointInTree(const std::string &rootFrame, const URDFModel &model, std::map<std::string, std::vector<Frame*> > &toChildMap) {
	if (isJointFrame(rootFrame,model))
		return true;
	std::vector<Frame*> children = toChildMap[rootFrame];
	BOOST_FOREACH(Frame* frame, children) {
		if (hasJointInTree(frame->getName(),model,toChildMap))
			return true;
	}
	return false;
}

static bool isSerialChain(const std::string &rootFrame, const URDFModel &model, std::map<std::string, std::vector<Frame*> > &toChildMap) {
	std::vector<Frame*> children = toChildMap[rootFrame];
	bool foundJointSubtree = false;
	std::string subtree = "";
	BOOST_FOREACH(Frame* frame, children) {
		if (hasJointInTree(frame->getName(),model,toChildMap)) {
			if (foundJointSubtree) {
				return false;
			} else {
				foundJointSubtree = true;
				subtree = frame->getName();
			}
		}
	}
	if (foundJointSubtree) {
		return isSerialChain(subtree,model,toChildMap);
	}
	return true;
}

static void createDevice(const URDFModel &model, DummySetup &setup) {
	Device::Ptr device = NULL;

	Frame* baseFrame = setup.toChildMap[setup.world->getName()][0];
	bool serial = isSerialChain(baseFrame->getName(),model,setup.toChildMap);

	if (serial) {
		RW_WARN("URDFLoader: detected serial device. This is not yet implemented so creating TreeDevice instead.");
	}

	/*if (serial) {
		// SerialDevice
		std::vector<Frame*> chain;
		// add the rest of the chain
		BOOST_FOREACH(DummyFrame& dframe, dev._frames) {
			chain.push_back(createFrame(dframe, setup));
		}
		// next add the device and model properties to the frames
		addToStateStructure(chain[0]->getName(), setup);
		// lastly add any props
		BOOST_FOREACH(DummyFrame& dframe, dev._frames) {
			addFrameProps(dframe, setup);
			addDevicePropsToFrame(dev, dframe.getName(), setup);
		}
		//State state( tree );
		State state = setup.tree->getDefaultState();
		model = ownedPtr(new SerialDevice(chain.front(), chain.back(), model.name, state));
	} else {*/
		// TreeDevice
		std::vector<Frame*> endEffectors;

		BOOST_FOREACH(const URDFModel::Link &link, model.links) {
			std::string scopedName = model.name + "." + link.name;
			if (setup.toChildMap[scopedName].size() == 0) {
				endEffectors.push_back(setup.frameMap[scopedName]);
			}
		}

		// And last create TreeDevice
		State state = setup.tree->getDefaultState();
		device = ownedPtr(new TreeDevice(baseFrame, endEffectors, model.name, state));
	//}

	setup.device = device;
}

static std::string stripPackagePath(const std::string &in) {
	std::string str = in;
	if (in.length() >= 10) {
		std::string packageStr = in.substr(0, 10);
		if (StringUtil::toLower(packageStr) == "package://")
			return in.substr(10,in.npos);
	}
	return str;
}

static std::string getGeometryString(const URDFModel::Link::Geometry &geometry, const std::string &defaultDir) {
	std::ostringstream val;

	switch (geometry.type) {
	case URDFModel::Link::Geometry::MESH:
		if (!StringUtil::isAbsoluteFileName(geometry.filenameMesh)) {
			val << defaultDir;
		}
		val << stripPackagePath(geometry.filenameMesh);
		break;
	case URDFModel::Link::Geometry::BOX:
		val << "#Box " << geometry.sizeBox[0] << " " << geometry.sizeBox[1] << " " << geometry.sizeBox[2];
		break;
	case URDFModel::Link::Geometry::SPHERE:
		val << "#Sphere " << geometry.radiusCylinderSphere;
		break;
	case URDFModel::Link::Geometry::CYLINDER:
		val << "#Cylinder " << geometry.radiusCylinderSphere << " " << geometry.lengthCylinder << " " << 20;
		break;
	default:
		val << "";
		break;
	}

	return val.str();
}

static void addModelsToObjects(const URDFModel& model, DummySetup &setup) {
	const std::string defaultDir = StringUtil::getDirectoryName(model.filename);
	BOOST_FOREACH(const URDFModel::Link &link, model.links) {
		std::string scopedName = model.name + "." + link.name;
		RigidObject::Ptr object;
		Frame* frame = setup.frameMap[scopedName];
		if (setup.objectMap.find(frame) == setup.objectMap.end())
			setup.objectMap[frame] = ownedPtr(new RigidObject(frame));
		object = setup.objectMap[frame];

		BOOST_FOREACH(const URDFModel::Link::Visual &visual, link.visual) {
			std::string geomString = getGeometryString(visual.geometry, defaultDir);

			Model3D::Ptr model3d = Model3DFactory::getModel(geomString, "name");
			model3d->setName(model.name);
			model3d->setTransform(visual.origin.toTransform());

			Model3D::Material material;

			// Search for material in root element
			bool found = false;
			BOOST_FOREACH(const URDFModel::Material &m, model.materials) {
				if (visual.material.name == m.name) {
					found = true;
					material = Model3D::Material(m.name,m.color.r,m.color.g,m.color.b,m.color.a);
					// Texture is ignored for now
				}
			}
			// If not found use the definition in the Visual element itself
			if (!found) {
				URDFModel::Material m = visual.material;
				material = Model3D::Material(m.name,m.color.r,m.color.g,m.color.b,m.color.a);
				// Texture is ignored for now
			}

			model3d->addMaterial(material);

			object->addModel(model3d);
		}

		BOOST_FOREACH(const URDFModel::Link::Collision &collision, link.collision) {
			std::string geomString = getGeometryString(collision.geometry, defaultDir);

			Geometry::Ptr geom = GeometryFactory::load(geomString, true);
			geom->setName(model.name);
			geom->setTransform(collision.origin.toTransform());
			geom->setFrame(frame);

			object->addGeometry(geom);
		}
	}
}

static WorkCell::Ptr toWorkCell(const URDFModel& model) {
	model.validate();

	// Create setup
	DummySetup setup;

	// Setup frame structure
	addAllFrames(model, setup.frameMap);
	setParentsAndChildren(model, setup);
	addToStateStructure(setup.world, setup.tree, setup.toChildMap);

	// Create a Device for the robot
	createDevice(model, setup);

	// Add graphical models to the frames
	addModelsToObjects(model, setup);

	// Now start creating the workcell
	WorkCell::Ptr wc = ownedPtr(new WorkCell(ownedPtr(setup.tree), model.name, model.filename));
	wc->setSceneDescriptor(setup.scene);

	// Add device to workcell
	wc->addDevice(setup.device);

	// Add all objects to scene
	std::map<Frame*, RigidObject::Ptr>::iterator first = setup.objectMap.begin();
	for (; first != setup.objectMap.end(); ++first) {
		wc->add((*first).second);
	}

	return wc;
}
}

URDFLoader::URDFLoader() {
}

URDFLoader::~URDFLoader() {
}

WorkCell::Ptr URDFLoader::loadWorkCell(const std::string& filename) {
	URDFModel::Ptr model = URDFParser::parseWorkcell(filename);
    return toWorkCell(*model);
}

WorkCell::Ptr URDFLoader::loadWorkCell(const std::string& filename, const std::string& schemaFileName) {
	URDFModel::Ptr model = URDFParser::parseWorkcell(filename, schemaFileName);
	std::cout << "MODEL: " << std::endl;
	std::cout << *model << std::endl;
    return toWorkCell(*model);
}

WorkCell::Ptr URDFLoader::loadWorkCell(std::istream& instream, const std::string& schemaFileName) {
	URDFModel::Ptr model = URDFParser::parseWorkcell(instream, schemaFileName);
    return toWorkCell(*model);
}

WorkCell::Ptr URDFLoader::loadWorkCell(DOMElem::Ptr element) {
	URDFModel::Ptr model = URDFParser::parseWorkcell(element);
    return toWorkCell(*model);
}

WorkCell::Ptr URDFLoader::load(const std::string& filename, const std::string& schemaFileName) {
	URDFLoader loader;
	return loader.loadWorkCell(filename, schemaFileName);
}

WorkCell::Ptr URDFLoader::load(std::istream& instream, const std::string& schemaFileName) {
	URDFLoader loader;
	return loader.loadWorkCell(instream, schemaFileName);
}

WorkCell::Ptr URDFLoader::load(DOMElem::Ptr element) {
	URDFLoader loader;
	return loader.loadWorkCell(element);
}
