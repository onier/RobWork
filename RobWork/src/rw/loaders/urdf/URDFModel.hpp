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

#ifndef RW_LOADERS_URDFMODEL_HPP_
#define RW_LOADERS_URDFMODEL_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/VectorND.hpp>

#include <boost/foreach.hpp>

#include <string>
#include <vector>
#include <iostream>

/**
 * @file URDFModel.hpp
 *
 * \copydoc rw::loaders::URDFModel
 */

namespace rw {
namespace loaders {
//! @addtogroup loaders

//! @{
/**
 * @brief Structure for holding data from the URDF file format.
 *
 * For further information on the URDF format, see http://wiki.ros.org/urdf
 */
class URDFModel {
public:
    //! @brief smart pointer type to this class
    typedef rw::common::Ptr<URDFModel> Ptr;

    struct Color {
		double r,g,b,a;

		friend std::ostream& operator<<(std::ostream& out, const Color& color) {
			out << "{r,g,b,a}={" << color.r << "," << color.g << "," << color.b << "," << color.a << "}";
			return out;
		}
    };

    struct Material {
    	std::string name;
    	Color color;
		std::string texture;

		friend std::ostream& operator<<(std::ostream& out, const Material& material) {
			out << "name:" << material.name;
			out << " color:" << material.color;
			out << " texture:" << material.texture;
			return out;
		}
    };

	struct Origin {
		rw::math::Vector3D<> xyz;
		rw::math::Vector3D<> rpy;

		rw::math::Transform3D<> toTransform() const;

		friend std::ostream& operator<<(std::ostream& out, const Origin& origin) {
			return out << "(" << origin.xyz << "," << origin.rpy << ")";
		}
	};

	struct Link {
		Link(): type(DEFAULT) {};

		enum Type {DEFAULT, LASER};

		struct Inertial {
			Inertial(): mass(0) {};

			Origin origin;
			double mass;
			rw::math::VectorND<6> inertia; // above-diagonal elements {ixx, ixy, ixz, iyy, iyz, izz}

			friend std::ostream& operator<<(std::ostream& out, const Inertial& inertial) {
				out << "   Origin: " << inertial.origin << std::endl;
				out << "   Mass: " << inertial.mass << std::endl;
				out << "   Inertia {ixx, ixy, ixz, iyy, iyz, izz}: " << inertial.inertia << std::endl;
				return out;
			}
		};

		struct Geometry {
			Geometry(): type(NONE), radiusCylinderSphere(0), lengthCylinder(0), scaleMesh(1.0) {};

			enum Type {NONE, BOX, CYLINDER, SPHERE, MESH};

			Type type;
			rw::math::Vector3D<> sizeBox;
			double radiusCylinderSphere;
			double lengthCylinder;
			std::string filenameMesh;
			double scaleMesh;

			friend std::ostream& operator<<(std::ostream& out, const Geometry& geo) {
				out << "    Type: ";
				switch(geo.type) {
				case NONE:
					out << "None";
					break;
				case BOX:
					out << "Box";
					break;
				case CYLINDER:
					out << "Cylinder";
					break;
				case SPHERE:
					out << "Sphere";
					break;
				case MESH:
					out << "Mesh";
					break;
				}
				out << std::endl;
				out << "    Size (box): " << geo.sizeBox << std::endl;
				out << "    Radius (cylinder/sphere): " << geo.radiusCylinderSphere << std::endl;
				out << "    Length (cylinder): " << geo.lengthCylinder << std::endl;
				out << "    Filename (mesh): " << geo.filenameMesh << std::endl;
				out << "    Scale (mesh): " << geo.scaleMesh << std::endl;
				return out;
			}
		};

		struct Visual {
			Origin origin;
			Geometry geometry;
			Material material;

			friend std::ostream& operator<<(std::ostream& out, const Visual& visual) {
				out << "   Origin: " << visual.origin << std::endl;
				out << "   Geometry" << std::endl;
				out << visual.geometry;
				out << "   Material: " << visual.material << std::endl;
				return out;
			}
		};

		struct Collision {
			Origin origin;
			Geometry geometry;

			friend std::ostream& operator<<(std::ostream& out, const Collision& col) {
				out << "   Origin: " << col.origin << std::endl;
				out << "   Geometry" << std::endl;
				out << col.geometry;
				return out;
			}
		};

		std::string name;
		Type type;
		Inertial inertial;
		std::vector<Visual> visual;
		std::vector<Collision> collision;

		friend std::ostream& operator<<(std::ostream& out, const Link& link) {
			out << " Link: " << link.name << std::endl;
			switch (link.type) {
			case DEFAULT:
				out << "  Type DEFAULT" << std::endl;
				break;
			case LASER:
				out << "  Type LASER" << std::endl;
				break;
			}
			out << "  Inertial" << std::endl;
			out << link.inertial;
			if (link.visual.size() == 0)
				out << "   No visual information for link" << std::endl;
			BOOST_FOREACH(const Visual &vis, link.visual) {
				out << vis;
			}
			if (link.collision.size() == 0)
				out << "   No collision information for link" << std::endl;
			BOOST_FOREACH(const Collision &col, link.collision) {
				out << col;
			}
			return out;
		}
	};

	struct Transmission {
		Transmission(): mechanicalReduction(0) {};

		struct Actuator {
			Actuator(): mechanicalReduction(0) {};
			std::string name;
			std::string hardwareInterface;
			double mechanicalReduction;

			friend std::ostream& operator<<(std::ostream& out, const Actuator& actuator) {
				out << "   Name: " << actuator.name << std::endl;
				out << "   hardwareInterface: " << actuator.hardwareInterface << std::endl;
				out << "   mechanicalReduction: " << actuator.mechanicalReduction << std::endl;
				return out;
			}
		};

		std::string name;
		std::string type;
		std::string joint;
		Actuator actuator;
		double mechanicalReduction;

		friend std::ostream& operator<<(std::ostream& out, const Transmission& transmission) {
			out << " Transmission: " << transmission.name << std::endl;
			out << "  Type: " << transmission.type << std::endl;
			out << "  Joint: " << transmission.joint << std::endl;
			out << "  Actuator" << std::endl;
			out << transmission.actuator;
			out << "  mechanicalReduction: " << transmission.mechanicalReduction << std::endl;
			return out;
		}
	};

	struct Joint {
		Joint(): type(REVOLUTE), axis(rw::math::Vector3D<>::x()), calibrationRising(0), calibrationFalling(0) {};

		enum Type {REVOLUTE, CONTINUOUS, PRISMATIC, FIXED, FLOATING, PLANAR};

		struct Dynamics {
			double damping;
			double friction;
		};

		struct Limit {
			double lower;
			double upper;
			double effort;
			double velocity;
		};

		struct Mimic {
			std::string joint;
			double multiplier;
			double offset;
		};

		struct Safety_Controller {
			double soft_lower_limit;
			double soft_upper_limit;
			double k_position;
			double k_velocity;
		};

		std::string name;
		Type type;
		Origin origin;
		std::string parent;
		std::string child;
		rw::math::Vector3D<> axis;
		double calibrationRising;
		double calibrationFalling;
		Dynamics dynamics;
		Limit limit;
		Mimic mimic;
		Safety_Controller safety_controller;

		friend std::ostream& operator<<(std::ostream& out, const Joint& joint) {
			out << " Joint: " << joint.name << std::endl;
			out << "  ! stream operator not implemented yet !" << std::endl;
			return out;
		}
	};

	struct Gazebo {
		friend std::ostream& operator<<(std::ostream& out, const Gazebo& gazebo) {
			out << " Gazebo tag" << std::endl;
			out << "  ! stream operator not implemented yet !" << std::endl;
			return out;
		}
	};

	struct Sensor {
		Sensor(): type(NONE), update_rate(1.0) {};

		enum Type {NONE, DEPTH};

		struct Origin {
			rw::math::Vector3D<> xyz;
			rw::math::Vector3D<> rpy;

			struct Image {
				Image(): width(0), height(0), hfov(0), near(0), far(0) {};
				std::size_t width;
				std::size_t height;
				std::string format;
				double hfov;
				double near;
				double far;
			};

			struct Ray {
				Ray(): samples(1), resolution(1), min_angle(0), max_angle(0) {};
				std::size_t samples;
				double resolution;
				double min_angle;
				double max_angle;
			};

			Image image;
			Ray horizontal;
			Ray vertical;
		};

		std::string name;
		Type type;
		double update_rate;
		std::string parent;
		Origin origin;

		friend std::ostream& operator<<(std::ostream& out, const Sensor& sensor) {
			out << " Sensor: " << sensor.name << std::endl;
			out << "  ! stream operator not implemented yet !" << std::endl;
			return out;
		}
	};

	URDFModel();
	virtual ~URDFModel();

    std::string name;
    std::string filename;
    std::string schema;
    std::vector<Material> materials;
    std::vector<Link> links;
    std::vector<Transmission> transmission;
    std::vector<Joint> joints;
    std::vector<Gazebo> gazebo;
    std::vector<Sensor> sensors;

    void validate() const;

	friend std::ostream& operator<<(std::ostream& out, const URDFModel& model) {
		out << "URDFModel: " << model.name << std::endl;
		out << " Filename: " << model.filename << std::endl;
		out << " Schema: " << model.schema << std::endl;
		if (model.materials.size() == 0)
			out << "No materials found" << std::endl;
		BOOST_FOREACH(const URDFModel::Material &material, model.materials) {
			out << " Material: " << material << std::endl;
		}
		if (model.links.size() == 0)
			out << "No links found" << std::endl;
		BOOST_FOREACH(const URDFModel::Link &link, model.links) {
			out << link;
		}
		if (model.transmission.size() == 0)
			out << "No transmissions found" << std::endl;
		BOOST_FOREACH(const URDFModel::Transmission &transmission, model.transmission) {
			out << transmission;
		}
		if (model.joints.size() == 0)
			out << "No joints found" << std::endl;
		BOOST_FOREACH(const URDFModel::Joint &joint, model.joints) {
			out << joint;
		}
		if (model.gazebo.size() == 0)
			out << "No gazebo tags found" << std::endl;
		BOOST_FOREACH(const URDFModel::Gazebo &gazebo, model.gazebo) {
			out << gazebo;
		}
		if (model.sensors.size() == 0)
			out << "No sensors found" << std::endl;
		BOOST_FOREACH(const URDFModel::Sensor &sensor, model.sensors) {
			out << sensor;
		}
		return out;
	}
};
//! @}
} /* namespace loaders */
} /* namespace rw */
#endif /* RW_LOADERS_URDFMODEL_HPP_ */
