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

#ifndef RW_LOADERS_URDFPARSER_HPP_
#define RW_LOADERS_URDFPARSER_HPP_

/**
 * @file URDFParser.hpp
 *
 * \copydoc rw::loaders::URDFParser
 */

#include "URDFModel.hpp"

#include <rw/common/DOMElem.hpp>

namespace rw {
namespace loaders {
//! @addtogroup loaders

//! @{
/**
 * @brief Parses a WorkCell from the URDF file format into a dummy workcell representation.
 *
 * For further information on the URDF format, see http://wiki.ros.org/urdf
 */
class URDFParser {
public:
	/**
	 * @brief Parse a URDF file into a dummy workcell representation.
	 * @param filename [in] the URDF file to parse.
	 * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
	 * @return dummy representation of a workcell.
	 */
	static URDFModel::Ptr parseWorkcell(const std::string& filename, const std::string& schemaFileName = "");

	/**
	 * @brief Parse a URDF stream into a dummy workcell representation.
     * @param instream [in] The input stream to read from.
	 * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
	 * @return dummy representation of a workcell.
	 */
	static URDFModel::Ptr parseWorkcell(std::istream& instream, const std::string& schemaFileName = "");

	/**
	 * @brief Parse a URDF DOMElem into a dummy workcell representation, assuming format is a valid URDF description.
     * @param element [in] DOMElement representing the WorkCell.
	 * @return dummy representation of a workcell.
	 */
	static URDFModel::Ptr parseWorkcell(rw::common::DOMElem::Ptr element);

private:
	struct Rule {
		Rule(): name(""), timesMin(0), timesMax(0) {};
		Rule(std::string id): name(id), timesMin(0), timesMax(0) {};
		Rule(std::string id, std::size_t min): name(id), timesMin(min), timesMax(0) {};
		Rule(std::string id, std::size_t min, std::size_t max): name(id), timesMin(min), timesMax(max) {};
		std::string name;
		std::size_t timesMin;
		std::size_t timesMax; // 0 is infinite
	};

	// Validation functions
	static void validateAttributes(rw::common::DOMElem::Ptr element, const std::vector<Rule> &rules);
	static void validateNoAttributes(rw::common::DOMElem::Ptr element);
	static void validateElements(rw::common::DOMElem::Ptr element, const std::vector<Rule> &rules);
	static void validateNoElements(rw::common::DOMElem::Ptr element);
	static void validateNoAttributesNoElements(rw::common::DOMElem::Ptr element);

	// Generic Element reader functions
	static std::string readSingleAttribute(rw::common::DOMElem::Ptr element, const std::string &name);
	static std::vector<double> readDoubleAttributes(rw::common::DOMElem::Ptr element, const std::vector<Rule> &attrRules);
	static std::vector<std::vector<double> > readDoubleListAttributes(rw::common::DOMElem::Ptr element, const std::vector<Rule> &attrRules);

	// Top-level reader functions
	static void read(URDFModel::Ptr model, rw::common::DOMElem::Ptr element);
	static void readRobot(URDFModel::Ptr model, rw::common::DOMElem::Ptr element);

	// Origin
	static URDFModel::Origin readOrigin(rw::common::DOMElem::Ptr element);

	// Materials
	static URDFModel::Material readMaterial(rw::common::DOMElem::Ptr element);

	// Links
	static void readLink(URDFModel::Ptr model, rw::common::DOMElem::Ptr element);
	static URDFModel::Link::Inertial readInertial(rw::common::DOMElem::Ptr element);
	static URDFModel::Link::Geometry readGeometry(rw::common::DOMElem::Ptr element);
	static URDFModel::Link::Visual readVisual(rw::common::DOMElem::Ptr element);
	static URDFModel::Link::Collision readCollision(rw::common::DOMElem::Ptr element);

	// Transmission
	static void readTransmission(URDFModel::Ptr model, rw::common::DOMElem::Ptr element);
	static URDFModel::Transmission::Actuator readActuator(rw::common::DOMElem::Ptr element);

	// Joint
	static void readJoint(URDFModel::Ptr model, rw::common::DOMElem::Ptr element);

	// Gazebo

	// Sensors
	static void readSensor(URDFModel::Ptr model, rw::common::DOMElem::Ptr element);
	static URDFModel::Sensor::Origin readSensorOrigin(rw::common::DOMElem::Ptr element);

	URDFParser();
	virtual ~URDFParser();
};
//! @}
} /* namespace loaders */
} /* namespace rw */
#endif /* RW_LOADERS_URDFPARSER_HPP_ */
