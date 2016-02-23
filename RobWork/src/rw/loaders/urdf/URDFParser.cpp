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

#include "URDFParser.hpp"
#include "URDFModel.hpp"

#include <rw/common/DOMParser.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;

URDFParser::URDFParser() {
}

URDFParser::~URDFParser() {
}

URDFModel::Ptr URDFParser::parseWorkcell(const std::string& filename, const std::string& schemaFileName) {
	URDFModel::Ptr model = ownedPtr(new URDFModel());
	model->filename = filename;
	model->schema = schemaFileName;
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load(filename);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    read(model, elementRoot);
    return model;
}

URDFModel::Ptr URDFParser::parseWorkcell(std::istream& instream, const std::string& schemaFileName) {
	URDFModel::Ptr model = ownedPtr(new URDFModel());
	model->schema = schemaFileName;
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load(instream);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    read(model, elementRoot);
    return model;
}

URDFModel::Ptr URDFParser::parseWorkcell(DOMElem::Ptr element) {
	URDFModel::Ptr model = ownedPtr(new URDFModel());
    read(model, element);
    return model;
}

void URDFParser::validateAttributes(DOMElem::Ptr element, const std::vector<Rule> &rules) {
	std::vector<std::size_t> hits(rules.size(),0);
	if (element->hasChild("<xmlattr>")) {
		BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
			std::string name = StringUtil::toLower(attr->getName());

			bool match = false;
			std::size_t i = 0;
			BOOST_FOREACH(const Rule &rule, rules) {
				std::string ruleName = StringUtil::toLower(rule.name);
				if (name == ruleName) {
					match = true;
					hits[i]++;
				}
				i++;
			}
			if (!match) {
				RW_THROW("Could not load URDF: Attribute '" << attr->getName() << "' is not valid for element '" << element->getName() << "'.");
			}
		}
	}
	std::size_t i = 0;
	BOOST_FOREACH(const Rule &rule, rules) {
		std::string max;
		if (rule.timesMax == 0)
			max = "an infinite amount of";
		else
			max = rule.timesMax;
		if (((rules[i].timesMax != 0) && (hits[i] > rules[i].timesMax)) || hits[i] < rules[i].timesMin) {
			RW_THROW("Could not load URDF: Attribute '" << rule.name << "' was found " << hits[i] << " times for element '" << element->getName() << "'. Must be between " << rule.timesMin << " and " << max << " times.");
		}
		i++;
	}
}

void URDFParser::validateNoAttributes(DOMElem::Ptr element) {
	std::vector<Rule> attrRules(0);
	validateAttributes(element, attrRules);
}

void URDFParser::validateElements(DOMElem::Ptr element, const std::vector<Rule> &rules) {
	std::vector<std::size_t> hits(rules.size(),0);
	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "<xmlcomment>") {
			// Ignore
		} else {
			bool match = false;
			std::size_t i = 0;
			BOOST_FOREACH(const Rule &rule, rules) {
				std::string ruleName = StringUtil::toLower(rule.name);
				if (name == ruleName) {
					match = true;
					hits[i]++;
				}
				i++;
			}
			if (!match) {
				RW_THROW("Could not load URDF: Child element '" << child->getName() << "' is not valid for element '" << element->getName() << "'.");
			}
		}
	}
	std::size_t i = 0;
	BOOST_FOREACH(const Rule &rule, rules) {
		std::string max;
		if (rule.timesMax == 0)
			max = "an infinite amount of";
		else
			max = rule.timesMax;
		if (((rules[i].timesMax != 0) && (hits[i] > rules[i].timesMax)) || hits[i] < rules[i].timesMin) {
			RW_THROW("Could not load URDF: Child element '" << rule.name << "' was found " << hits[i] << " times for element '" << element->getName() << "'. Must be between " << rule.timesMin << " and " << max << " times.");
		}
		i++;
	}
}

void URDFParser::validateNoElements(DOMElem::Ptr element) {
	std::vector<Rule> elemRules(0);
	validateElements(element, elemRules);
}

void URDFParser::validateNoAttributesNoElements(DOMElem::Ptr element) {
	std::vector<Rule> attrRules(0);
	validateAttributes(element, attrRules);

	std::vector<Rule> elemRules(0);
	validateElements(element, elemRules);
}

std::string URDFParser::readSingleAttribute(DOMElem::Ptr element, const std::string &name) {
	std::string res;
	std::vector<Rule> attrRules(1);
	attrRules[0] = Rule(name,1,1);
	validateAttributes(element, attrRules);
	validateNoElements(element);
	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()) {
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == name) {
			res = attr->getValue();
		}
	}
	return res;
}

std::vector<std::vector<double> > URDFParser::readDoubleListAttributes(DOMElem::Ptr element, const std::vector<Rule> &attrRules) {
	validateNoElements(element);
	validateAttributes(element, attrRules);

	std::vector<std::vector<double> > list = std::vector<std::vector<double> >(attrRules.size());

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		std::size_t i = 0;
		BOOST_FOREACH(const Rule &rule, attrRules) {
			std::string ruleName = StringUtil::toLower(rule.name);
			if (attrName == ruleName) {
				list[i] = attr->getValueAsDoubleList();
			}
			i++;
		}
	}

	return list;
}

std::vector<double> URDFParser::readDoubleAttributes(DOMElem::Ptr element, const std::vector<Rule> &attrRules) {
	validateNoElements(element);
	validateAttributes(element, attrRules);

	std::vector<double> list = std::vector<double>(attrRules.size(), 0);

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		std::size_t i = 0;
		BOOST_FOREACH(const Rule &rule, attrRules) {
			std::string ruleName = StringUtil::toLower(rule.name);
			if (attrName == ruleName) {
				list[i] = attr->getValueAsDouble();
			}
			i++;
		}
	}

	return list;
}

URDFModel::Origin URDFParser::readOrigin(DOMElem::Ptr element) {
	URDFModel::Origin origin;

	std::vector<Rule> attrRules(2);
	attrRules[0] = Rule("xyz",0,1);
	attrRules[1] = Rule("rpy",0,1);
	std::vector<std::vector<double> > attr = readDoubleListAttributes(element, attrRules);
	std::vector<double> xyz = attr[0];
	std::vector<double> rpy = attr[1];
	std::size_t required = 3;
	if (xyz.size() != 0) {
		if (xyz.size() != required) {
			RW_THROW("Could not load URDF: Attribute 'xyz' in element '" << element->getName() << "' could not be read. Number of doubles must be 3.");
		}
		origin.xyz = Vector3D<>(xyz[0],xyz[1],xyz[2]);
	}
	if (rpy.size() != 0) {
		if (rpy.size() != required) {
			RW_THROW("Could not load URDF: Attribute 'rpy' in element '" << element->getName() << "' could not be read. Number of doubles must be 3.");
		}
		origin.rpy = Vector3D<>(rpy[0],rpy[1],rpy[2]);
	}

	return origin;
}

URDFModel::Material URDFParser::readMaterial(DOMElem::Ptr element) {
	URDFModel::Material material;

	std::vector<Rule> attrRules(1);
	attrRules[0] = Rule("name",1,1);
	validateAttributes(element, attrRules);

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == "name") {
			material.name = attr->getValue();
		}
	}

	std::vector<Rule> elemRules(2);
	elemRules[0] = Rule("color",0,1);
	elemRules[1] = Rule("texture",0,1);
	validateElements(element, elemRules);

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "color") {
			std::vector<Rule> attrRules(1);
			attrRules[0] = Rule("rgba",1,1);
			std::vector<double> attr = readDoubleListAttributes(child, attrRules)[0];
			std::size_t required = 4;
			if (attr.size() != required) {
				RW_THROW("Could not load URDF: Child element '" << child->getName() << "' in element " << element->getName() << " could not be read. Number of doubles in the '" << attrRules[0].name << "' attribute must be " << required << ".");
			}
			URDFModel::Color color;
			color.r = attr[0];
			color.g = attr[1];
			color.b = attr[2];
			color.a = attr[3];
			material.color = color;
		} else if (name == "texture") {
			validateNoAttributesNoElements(child);
			material.texture = child->getValue();
		}
	}

	return material;
}

void URDFParser::read(URDFModel::Ptr model, DOMElem::Ptr element) {
	std::vector<Rule> rules(1);
	rules[0] = Rule("robot",1,1);
	validateElements(element, rules);

	DOMElem::Ptr robotElem;
	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "robot") {
			robotElem = child;
		}
	}
	readRobot(model, robotElem);
}

void URDFParser::readRobot(URDFModel::Ptr model, DOMElem::Ptr element) {
	std::vector<Rule> attrRules(5);
	attrRules[0] = Rule("name",1,1);
	attrRules[1] = Rule("xmlns:controller",0,1);
	attrRules[2] = Rule("xmlns:interface",0,1);
	attrRules[3] = Rule("xmlns:sensor",0,1);
	attrRules[4] = Rule("xmlns:xacro",0,1);
	validateAttributes(element, attrRules);

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == "name") {
			model->name = attr->getValue();
		}
	}

	std::vector<Rule> elemRules(6);
	elemRules[0] = Rule("material");
	elemRules[1] = Rule("link");
	elemRules[2] = Rule("transmission");
	elemRules[3] = Rule("joint");
	elemRules[4] = Rule("gazebo");
	elemRules[5] = Rule("sensor");
	validateElements(element, elemRules);

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "material") {
			model->materials.push_back(readMaterial(child));
		} else if (name == "link") {
			readLink(model, child);
		} else if (name == "transmission") {
			readTransmission(model, child);
		} else if (name == "joint") {
			readJoint(model, child);
		} else if (name == "gazebo") {
			std::cout << "gazebo" << std::endl;
		} else if (name == "sensor") {
			std::cout << "sensor" << std::endl;
		}
	}
}

void URDFParser::readLink(URDFModel::Ptr model, DOMElem::Ptr element) {
	URDFModel::Link link;

	std::vector<Rule> attrRules(2);
	attrRules[0] = Rule("name",1,1);
	attrRules[1] = Rule("type",0,1);
	validateAttributes(element, attrRules);

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == "name") {
			link.name = attr->getValue();
		} else if (attrName == "type") {
			std::string type = StringUtil::toLower(attr->getValue());
			if (type == "laser") {
				link.type = URDFModel::Link::LASER;
			} else {
				RW_THROW("Could not load URDF: Attribute '" << attr->getName() << "' in element '" << element->getName() << "' is invalid. Allowed option for type is 'laser'.");
			}
		}
	}

	std::vector<Rule> elemRules(3);
	elemRules[0] = Rule("inertial",0,1);
	elemRules[1] = Rule("visual");
	elemRules[2] = Rule("collision");
	validateElements(element, elemRules);

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "inertial") {
			link.inertial = readInertial(child);
		} else if (name == "visual") {
			link.visual.push_back(readVisual(child));
		} else if (name == "collision") {
			link.collision.push_back(readCollision(child));
		}
	}

	model->links.push_back(link);
}

URDFModel::Link::Inertial URDFParser::readInertial(DOMElem::Ptr element) {
	validateNoAttributes(element);

	std::vector<Rule> elemRules(3);
	elemRules[0] = Rule("mass",1,1);
	elemRules[1] = Rule("inertia",1,1);
	elemRules[2] = Rule("origin",0,1);
	validateElements(element, elemRules);

	URDFModel::Link::Inertial inertial;

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "mass") {
			std::vector<Rule> attrRules(1);
			attrRules[0] = Rule("value",1,1);
			inertial.mass = readDoubleAttributes(child, attrRules)[0];
		} else if (name == "inertia") {
			std::vector<Rule> attrRules(6);
			attrRules[0] = Rule("ixx",1,1);
			attrRules[1] = Rule("ixy",1,1);
			attrRules[2] = Rule("ixz",1,1);
			attrRules[3] = Rule("iyy",1,1);
			attrRules[4] = Rule("iyz",1,1);
			attrRules[5] = Rule("izz",1,1);
			std::vector<double> attr = readDoubleAttributes(child, attrRules);
			VectorND<6> inertia;
			for (std::size_t i = 0; i < attr.size(); i++) {
				inertia[i] = attr[i];
			}
			inertial.inertia = inertia;
		} else if (name == "origin") {
			inertial.origin = readOrigin(child);
		}
	}

	return inertial;
}

URDFModel::Link::Geometry URDFParser::readGeometry(DOMElem::Ptr element) {
	validateNoAttributes(element);

	std::vector<Rule> elemRules(4);
	elemRules[0] = Rule("box",0,1);
	elemRules[1] = Rule("cylinder",0,1);
	elemRules[2] = Rule("sphere",0,1);
	elemRules[3] = Rule("mesh",0,1);
	validateElements(element, elemRules);

	URDFModel::Link::Geometry geometry;

	bool geometryAdded = false;
	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "box" && !geometryAdded) {
			std::vector<Rule> attrRules(1);
			attrRules[0] = Rule("size",1,1);
			std::vector<double> sizeBox = readDoubleListAttributes(child, attrRules)[0];
			if (sizeBox.size() != 3) {
				RW_THROW("Could not load URDF: Attribute 'size' in element '" << element->getName() << "' could not be read. Number of doubles must be 3.");
			}
			geometry.type = URDFModel::Link::Geometry::BOX;
			geometry.sizeBox = Vector3D<>(sizeBox[0],sizeBox[1],sizeBox[2]);
			geometryAdded = true;
		} else if (name == "box" && geometryAdded) {
			RW_THROW("Could not load URDF: Multiple geometries in element '" << element->getName() << "' not allowed.");
		} else if (name == "cylinder" && !geometryAdded) {
			std::vector<Rule> attrRules(2);
			attrRules[0] = Rule("radius",1,1);
			attrRules[1] = Rule("length",1,1);
			std::vector<double> attr = readDoubleAttributes(child, attrRules);
			geometry.type = URDFModel::Link::Geometry::CYLINDER;
			geometry.radiusCylinderSphere = attr[0];
			geometry.lengthCylinder = attr[1];
			geometryAdded = true;
		} else if (name == "cylinder" && geometryAdded) {
			RW_THROW("Could not load URDF: Multiple geometries in element '" << element->getName() << "' not allowed.");
		} else if (name == "sphere" && !geometryAdded) {
			std::vector<Rule> attrRules(1);
			attrRules[0] = Rule("radius",1,1);
			geometry.radiusCylinderSphere = readDoubleAttributes(child, attrRules)[0];
			geometry.type = URDFModel::Link::Geometry::SPHERE;
			geometryAdded = true;
		} else if (name == "sphere" && geometryAdded) {
			RW_THROW("Could not load URDF: Multiple geometries in element '" << element->getName() << "' not allowed.");
		} else if (name == "mesh" && !geometryAdded) {
			std::vector<Rule> attrRules(2);
			attrRules[0] = Rule("filename",1,1);
			attrRules[1] = Rule("scale",0,1);
			validateAttributes(child, attrRules);
			validateNoElements(child);
			BOOST_FOREACH(DOMElem::Ptr attr, child->getAttributes()) {
				std::string attrName = StringUtil::toLower(attr->getName());
				if (attrName == "filename") {
					geometry.filenameMesh = attr->getValue();
				} else if (attrName == "scale") {
					geometry.scaleMesh = attr->getValueAsDouble();
				}
			}
			geometry.type = URDFModel::Link::Geometry::MESH;
			geometryAdded = true;
		} else if (name == "mesh" && geometryAdded) {
			RW_THROW("Could not load URDF: Multiple geometries in element '" << element->getName() << "' not allowed.");
		}
	}

	return geometry;
}

URDFModel::Link::Visual URDFParser::readVisual(DOMElem::Ptr element) {
	validateNoAttributes(element);

	std::vector<Rule> elemRules(3);
	elemRules[0] = Rule("geometry",1,1);
	elemRules[1] = Rule("material",0,1);
	elemRules[2] = Rule("origin",0,1);
	validateElements(element, elemRules);

	URDFModel::Link::Visual visual;

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "geometry") {
			visual.geometry = readGeometry(child);
		} else if (name == "material") {
			visual.material = readMaterial(child);
		} else if (name == "origin") {
			visual.origin = readOrigin(child);
		}
	}

	return visual;
}

URDFModel::Link::Collision URDFParser::readCollision(DOMElem::Ptr element) {
	validateNoAttributes(element);

	std::vector<Rule> elemRules(2);
	elemRules[0] = Rule("geometry",1,1);
	elemRules[1] = Rule("origin",0,1);
	validateElements(element, elemRules);

	URDFModel::Link::Collision collision;

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "geometry") {
			collision.geometry = readGeometry(child);
		} else if (name == "origin") {
			collision.origin = readOrigin(child);
		}
	}

	return collision;
}

void URDFParser::readTransmission(URDFModel::Ptr model, DOMElem::Ptr element) {
	URDFModel::Transmission transmission;

	std::vector<Rule> attrRules(2);
	attrRules[0] = Rule("name",1,1);
	attrRules[1] = Rule("type",0,1);
	validateAttributes(element, attrRules);

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == "name") {
			transmission.name = attr->getValue();
		} else if (attrName == "type") {
			transmission.type = attr->getValue();
		}
	}

	std::vector<Rule> elemRules(4);
	elemRules[0] = Rule("type",0,1);
	elemRules[1] = Rule("joint",1,1);
	elemRules[2] = Rule("actuator",1,1);
	elemRules[3] = Rule("mechanicalReduction",0,1);
	validateElements(element, elemRules);

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "type") {
			if (transmission.type == "") {
				transmission.type = child->getValue();
			} else if (transmission.type != child->getValue()) {
				RW_THROW("Could not load URDF: Conflicting types set for element '" << element->getName() << "'.");
			}
		} else if (name == "joint") {
			transmission.joint = readSingleAttribute(child, "name");
		} else if (name == "actuator") {
			transmission.actuator = readActuator(child);
		} else if (name == "mechanicalReduction") {
			validateNoAttributesNoElements(child);
			transmission.mechanicalReduction = child->getValueAsDouble();
		}
	}

	model->transmission.push_back(transmission);
}

URDFModel::Transmission::Actuator URDFParser::readActuator(DOMElem::Ptr element) {
	URDFModel::Transmission::Actuator actuator;

	std::vector<Rule> attrRules(1);
	attrRules[0] = Rule("name",1,1);
	validateAttributes(element, attrRules);

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == "name") {
			actuator.name = attr->getValue();
		}
	}

	std::vector<Rule> elemRules(2);
	elemRules[0] = Rule("hardwareInterface",0,1);
	elemRules[1] = Rule("mechanicalReduction",0,1);
	validateElements(element, elemRules);

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "hardwareInterface") {
			validateNoAttributesNoElements(child);
			actuator.hardwareInterface = child->getValue();
		} else if (name == "mechanicalReduction") {
			validateNoAttributesNoElements(child);
			actuator.mechanicalReduction = child->getValueAsDouble();
		}
	}

	return actuator;
}

void URDFParser::readJoint(URDFModel::Ptr model, DOMElem::Ptr element) {
	URDFModel::Joint joint;

	std::vector<Rule> attrRules(2);
	attrRules[0] = Rule("name",1,1);
	attrRules[1] = Rule("type",1,1);
	validateAttributes(element, attrRules);

	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == "name") {
			joint.name = attr->getValue();
		} else if (attrName == "type") {
			std::string type = StringUtil::toLower(attr->getValue());
			if (type == "revolute") {
				joint.type = URDFModel::Joint::REVOLUTE;
			} else if (type == "continuous") {
				joint.type = URDFModel::Joint::CONTINUOUS;
			} else if (type == "prismatic") {
				joint.type = URDFModel::Joint::PRISMATIC;
			} else if (type == "fixed") {
				joint.type = URDFModel::Joint::FIXED;
			} else if (type == "floating") {
				joint.type = URDFModel::Joint::FLOATING;
			} else if (type == "planar") {
				joint.type = URDFModel::Joint::PLANAR;
			} else {
				RW_THROW("Could not load URDF: Attribute '" << attr->getName() << "' in element '" << element->getName() << "' is invalid. Allowed options for type are 'revolute', 'continuous', 'prismatic', 'fixed', 'floating', and 'planar'.");
			}
		}
	}

	std::vector<Rule> elemRules(9);
	elemRules[0] = Rule("origin",0,1);
	elemRules[1] = Rule("parent",1,1);
	elemRules[2] = Rule("child",1,1);
	elemRules[3] = Rule("axis",0,1);
	elemRules[4] = Rule("calibration",0,1);
	elemRules[5] = Rule("dynamics",0,1);
	elemRules[6] = Rule("limit",0,1);
	elemRules[7] = Rule("mimic",0,1);
	elemRules[8] = Rule("safety_controller",0,1);
	validateElements(element, elemRules);

	bool limitTag = false;
	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "origin") {
			joint.origin = readOrigin(child);
		} else if (name == "parent") {
			joint.parent = readSingleAttribute(child, "link");
		} else if (name == "child") {
			joint.child = readSingleAttribute(child, "link");
		} else if (name == "axis") {
			std::vector<Rule> attrRules(1);
			attrRules[0] = Rule("xyz",1,1);
			std::vector<double> vec = readDoubleListAttributes(child, attrRules)[0];
			if (vec.size() != 3) {
				RW_THROW("Could not load URDF: Attribute 'xyz' in element '" << element->getName() << "' could not be read. Number of doubles must be 3.");
			}
			joint.axis = Vector3D<>(vec[0],vec[1],vec[2]);
		} else if (name == "calibration") {
			std::vector<Rule> attrRules(2);
			attrRules[0] = Rule("rising",0,1);
			attrRules[1] = Rule("falling",0,1);
			std::vector<std::vector<double> > attr = readDoubleListAttributes(child, attrRules);
			if (attr[0].size() == 1) {
				joint.calibrationRising = attr[0][0];
			}
			if (attr[1].size() == 1) {
				joint.calibrationRising = attr[1][0];
			}
		} else if (name == "dynamics") {
			std::vector<Rule> attrRules(2);
			attrRules[0] = Rule("damping",0,1);
			attrRules[1] = Rule("friction",0,1);
			std::vector<std::vector<double> > attr = readDoubleListAttributes(child, attrRules);
			URDFModel::Joint::Dynamics dynamics;
			if (attr[0].size() == 1) {
				dynamics.damping = attr[0][0];
			}
			if (attr[1].size() == 1) {
				dynamics.friction = attr[1][0];
			}
			joint.dynamics = dynamics;
		} else if (name == "limit") {
			std::vector<Rule> attrRules(4);
			attrRules[0] = Rule("lower",0,1);
			attrRules[1] = Rule("upper",0,1);
			attrRules[2] = Rule("effort",1,1);
			attrRules[3] = Rule("velocity",1,1);
			std::vector<std::vector<double> > attr = readDoubleListAttributes(child, attrRules);
			URDFModel::Joint::Limit limit;
			if (attr[0].size() == 1)
				limit.lower = attr[0][0];
			if (attr[1].size() == 1)
				limit.upper = attr[1][0];
			if (attr[2].size() == 1)
				limit.effort = attr[2][0];
			if (attr[3].size() == 1)
				limit.velocity = attr[3][0];
			joint.limit = limit;
			limitTag = true;
		} else if (name == "mimic") {
			std::vector<Rule> elemRules(3);
			elemRules[0] = Rule("joint",1,1);
			elemRules[1] = Rule("multiplier",0,1);
			elemRules[2] = Rule("offset",0,1);
			validateNoAttributes(child);
			validateElements(child, attrRules);
			URDFModel::Joint::Mimic mimic;
			BOOST_FOREACH(DOMElem::Ptr mimicElem, child->getChildren()) {
				std::string mimicName = StringUtil::toLower(child->getName());
				if (mimicName == "joint") {
					mimic.joint = mimicElem->getValue();
				} else if (mimicName == "multiplier") {
					mimic.multiplier = mimicElem->getValueAsDouble();
				} else if (mimicName == "multiplier") {
					mimic.offset = mimicElem->getValueAsDouble();
				}
			}
			joint.mimic = mimic;
		} else if (name == "safety_controller") {
			std::vector<Rule> attrRules(4);
			attrRules[0] = Rule("soft_lower_limit",0,1);
			attrRules[1] = Rule("soft_upper_limit",0,1);
			attrRules[2] = Rule("k_position",0,1);
			attrRules[3] = Rule("k_velocity",1,1);
			std::vector<std::vector<double> > attr = readDoubleListAttributes(child, attrRules);
			URDFModel::Joint::Safety_Controller sc;
			if (attr[0].size() == 1)
				sc.soft_lower_limit = attr[0][0];
			if (attr[1].size() == 1)
				sc.soft_upper_limit = attr[1][0];
			if (attr[2].size() == 1)
				sc.k_position = attr[2][0];
			if (attr[3].size() == 1)
				sc.k_velocity = attr[3][0];
			joint.safety_controller = sc;
		}
	}

	if (joint.type == URDFModel::Joint::REVOLUTE || joint.type == URDFModel::Joint::PRISMATIC) {
		if (!limitTag) {
			RW_THROW("Could not load URDF: Element 'limit' in element '" << element->getName() << "' is missing for joint of type 'revolute' or 'prismatic'.");
		}
	}

	model->joints.push_back(joint);
}

void URDFParser::readSensor(URDFModel::Ptr model, DOMElem::Ptr element) {
	URDFModel::Sensor sensor;

	std::vector<Rule> attrRules(2);
	attrRules[0] = Rule("name",1,1);
	attrRules[1] = Rule("type",0,1);
	attrRules[2] = Rule("update_rate",0,1);
	validateAttributes(element, attrRules);

	bool updateRateSet = false;
	BOOST_FOREACH(DOMElem::Ptr attr, element->getAttributes()){
		std::string attrName = StringUtil::toLower(attr->getName());
		if (attrName == "name") {
			sensor.name = attr->getValue();
		} else if (attrName == "type") {
			std::string type = StringUtil::toLower(attr->getValue());
			if (type == "depth") {
				sensor.type = URDFModel::Sensor::DEPTH;
			} else {
				RW_THROW("Could not load URDF: Attribute '" << attr->getName() << "' in element '" << element->getName() << "' is invalid. Allowed option for type is 'depth'.");
			}
		} else if (attrName == "update_rate") {
			sensor.update_rate = attr->getValueAsDouble();
			updateRateSet = true;
		}
	}

	std::vector<Rule> elemRules(3);
	elemRules[0] = Rule("origin",0,1);
	elemRules[1] = Rule("parent",1,1);
	elemRules[2] = Rule("update_rate",0,1);
	validateElements(element, elemRules);

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		std::string name = StringUtil::toLower(child->getName());
		if (name == "origin") {
			sensor.origin = readSensorOrigin(child);
		} else if (name == "parent") {
			sensor.parent = readSingleAttribute(child, "link");
		} else if (name == "update_rate") {
			if (!updateRateSet) {
				sensor.update_rate = child->getValueAsDouble();
			} else if (sensor.update_rate != child->getValueAsDouble()) {
				RW_THROW("Could not load URDF: Conflicting update_rates set for element '" << element->getName() << "'.");
			}
		}
	}

	model->sensors.push_back(sensor);
}

URDFModel::Sensor::Origin URDFParser::readSensorOrigin(DOMElem::Ptr element) {
	URDFModel::Sensor::Origin origin;

	std::vector<Rule> attrRules(4);
	attrRules[0] = Rule("xyz",0,1);
	attrRules[1] = Rule("rpy",0,1);
	attrRules[2] = Rule("camera",0,1);
	attrRules[3] = Rule("ray",0,1);
	std::vector<std::vector<double> > attr = readDoubleListAttributes(element, attrRules);
	std::vector<double> xyz = attr[0];
	std::vector<double> rpy = attr[1];
	std::size_t required = 3;
	if (xyz.size() != 0) {
		if (xyz.size() != required) {
			RW_THROW("Could not load URDF: Attribute 'xyz' in element '" << element->getName() << "' could not be read. Number of doubles must be 3.");
		}
		origin.xyz = Vector3D<>(xyz[0],xyz[1],xyz[2]);
	}
	if (rpy.size() != 0) {
		if (rpy.size() != required) {
			RW_THROW("Could not load URDF: Attribute 'rpy' in element '" << element->getName() << "' could not be read. Number of doubles must be 3.");
		}
		origin.rpy = Vector3D<>(rpy[0],rpy[1],rpy[2]);
	}

	return origin;
}
