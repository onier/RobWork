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

#include "DOMPropertyMapSaver.hpp"
#include "DOMBasisTypes.hpp"

#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/PropertyBase.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;

DOMPropertyMapSaver::DOMPropertyMapSaver() {
}

DOMPropertyMapSaver::~DOMPropertyMapSaver() {
}

void DOMPropertyMapSaver::saveProperty(const rw::common::Ptr<const PropertyBase> property, DOMElem& element) {
	const DOMElem::Ptr name = element.addChild("Name");
	const DOMElem::Ptr description = element.addChild("Description");
	const DOMElem::Ptr value = element.addChild("Value");
	std::cout << "Adding property: " << property->getIdentifier() << std::endl;

	name->setValue(property->getIdentifier());
	description->setValue(property->getDescription());

	const PropertyType& type = property->getType();

	std::cout << "k" << std::endl;
	switch(type.getId()) {
	case PropertyType::Q:
		DOMBasisTypes::createQ(property.scast<const Property<Q> >()->getValue(), value);
		break;
	default:
		//RW_THROW("DOMPropertyMapSaver (saveProperty): Property \"" << property->getIdentifier() << "\" could not be saved (id " << type.getId() << ") !");
		break;
	}
	std::cout << "kk" << std::endl;
/*
	BOOST_FOREACH( DOMElem::Ptr child, element->getChildren() ){
		if (child->isName("id") ) {
			name = child->getValue();
		} else if (child->isName("Description")) {
			description = child->getValue();
		} else if (child->isName("PropertyMap")) {
			return ownedPtr(new Property<PropertyMap>(name, description, DOMPropertyMapLoader::readProperties(child, true)));
		} else if (child->isName("String")) {
			return ownedPtr(new Property<std::string>(name, description, DOMBasisTypes::readString(child)));
		} else if (child->isName("StringList")) {
			return ownedPtr(new Property<std::vector<std::string> >(name, description, DOMBasisTypes::readStringList(child)));
		} else if (child->isName("IntList")) {
			return ownedPtr(new Property<std::vector<int> >(name, description, DOMBasisTypes::readIntList(child)));
		} else if (child->isName("DoubleList")) {
			return ownedPtr(new Property<std::vector<double> >(name, description, DOMBasisTypes::readDoubleList(child)));
		} else if (child->isName("Double")) {
			return ownedPtr(new Property<double>(name, description, DOMBasisTypes::readDouble(child)));
		} else if (child->isName("Float")) {
			return ownedPtr(new Property<float>(name, description, DOMBasisTypes::readFloat(child)));
		} else if (child->isName("Integer")) {
			return ownedPtr(new Property<int>(name, description, DOMBasisTypes::readInt(child)));
		} else if (child->isName("Boolean")) {
			return ownedPtr(new Property<bool>(name, description, DOMBasisTypes::readBool(child)));
		} else if (child->isName("Vector3D")){
			return ownedPtr(new Property<Vector3D<> >(name, description, DOMBasisTypes::readVector3D(child)));
		} else if (child->isName("Vector2D")) {
			return ownedPtr(new Property<Vector2D<> >(name, description, DOMBasisTypes::readVector2D(child)));
		} else if (child->isName("Q")) {
			return ownedPtr(new Property<Q>(name, description, DOMBasisTypes::readQ(child)));
		} else if (child->isName("Transform3D")) {
			return ownedPtr(new Property<Transform3D<> >(name, description, DOMBasisTypes::readTransform3D(child)));
		} else if (child->isName("Rotation3D")) {
			return ownedPtr(new Property<Rotation3D<> >(name, description, DOMBasisTypes::readRotation3D(child)));
		} else if (child->isName("EAA")) {
			return ownedPtr(new Property<EAA<> >(name, description, DOMBasisTypes::readEAA(child)));
		} else if (child->isName("RPY")) {
			return ownedPtr(new Property<RPY<> >(name, description, DOMBasisTypes::readRPY(child)));
		} else if (child->isName("Quaternion")) {
			return ownedPtr(new Property<Quaternion<> >(name, description, DOMBasisTypes::readQuaternion(child)));
		} else if (child->isName("Rotation2D")) {
			return ownedPtr(new Property<Rotation2D<> >(name, description, DOMBasisTypes::readRotation2D(child)));
		} else if (child->isName("VelocityScrew6D")) {
			return ownedPtr(new Property<VelocityScrew6D<> >(name, description, DOMBasisTypes::readVelocityScrew6D(child)));
		} else if (child->isName("QPath")){
			DOMPathLoader loader(child);
			return ownedPtr(new Property<QPath>(name, description,*loader.getQPath()));
		} else if (child->isName("T3DPath")){
			DOMPathLoader loader(child);
			return ownedPtr(new Property<Transform3DPath >(name, description, *loader.getTransform3DPath()));
		} else {
			RW_THROW("Parse Error: data value not reqognized in property with id \""<< name << "\"!");
		}
	}*/
}

void DOMPropertyMapSaver::saveProperties(const PropertyMap& map, DOMElem& element) {
	BOOST_FOREACH(const rw::common::Ptr<const PropertyBase> property, map.getProperties()) {
		const DOMElem::Ptr child = element.addChild("Property");
		saveProperty(property, *child);
		break;
	}
}

void DOMPropertyMapSaver::save(const PropertyMap& map, const std::string& filename, const std::string& schemaFileName) {
	std::cout << "1" << std::endl;
    const DOMParser::Ptr parser = DOMParser::make();
	std::cout << "2" << std::endl;
	const DOMElem::Ptr doc = parser->getRootElement();
	std::cout << "3" << std::endl;
	const DOMElem::Ptr root = doc->addChild("PropertyMap");
	std::cout << "4" << std::endl;
	try {
	saveProperties(map,*root);
	} catch (Exception& e) {
		std::cout << "exception: " << e.what() << std::endl;
	} catch (...) {
		std::cout << "exception" << std::endl;
	}
	std::cout << "5: " << filename << std::endl;
    std::ofstream stream(filename.c_str());
	std::cout << "52" << std::endl;
	parser->save(filename);
	std::cout << "6" << std::endl;
}

void DOMPropertyMapSaver::write(const PropertyMap& map, std::ostream& outstream, const std::string& schemaFileName) {
    const DOMParser::Ptr parser = DOMParser::make();
	const DOMElem::Ptr doc = parser->getRootElement();
	const DOMElem::Ptr root = doc->addChild("PropertyMap");
	saveProperties(map,*root);
	parser->save(outstream);
}
