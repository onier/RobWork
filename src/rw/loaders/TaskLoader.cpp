/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "TaskLoader.hpp"

#include <iostream>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/task/Trajectory.hpp>
#include <rw/task/Task.hpp>

#include <rw/loaders/xml/XML.hpp>

typedef boost::property_tree::ptree PTree;

using namespace std;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::task;
using namespace boost::property_tree;

namespace
{
    string quote(const string& str) { return StringUtil::quote(str); }

    typedef PTree::const_iterator CI;

    ToolSpeed readToolSpeed(const PTree& tree)
    {
        const double speed = tree.get<double>("ToolSpeed.N");
        if (tree.get_child_optional("ToolSpeed.Angular")) {
            return ToolSpeed(ToolSpeed::Angular, speed);
        } else if (tree.get_child_optional("ToolSpeed.Positional")) {
            return ToolSpeed(ToolSpeed::Positional, speed);
        } else {
            RW_THROW(
                "No tool speed specified. <Angular> or <Positional> expected.");
            return readToolSpeed(tree);
        }
    }

    PropertyMap readOptionalPropertyMap(const PTree& tree)
    {
        PropertyMap properties;
        const CI p = tree.find("PropertyMap");
        if (p != tree.end()) {
            XML::readPropertyMap(p->second, properties);
        }
        return properties;
    }

    string readOptionalName(const PTree& tree)
    {
        return tree.get<string>("Name", "");
    }

    Entity readEntity(const PTree& tree)
    {
        return Entity(readOptionalName(tree), readOptionalPropertyMap(tree));
    }

    Frame* readFrame(const PTree& tree, const WorkCell& workcell)
    {
        const string frame_name = tree.get<string>("Frame");
        Frame* frame = workcell.findFrame(frame_name);
        if (!frame)
            RW_THROW(
                "Reference frame "
                << quote(frame_name)
                << " not found in workcell "
                << workcell);
        return frame;
    }

    Link::Constraint readLinkConstraint(const PTree& tree, const WorkCell& workcell)
    {
        if (tree.find("LinearToolConstraint") != tree.end()) {
            const PTree& child = tree.get_child("LinearToolConstraint");
            return LinearToolConstraint(readToolSpeed(child));
        }

        else if (tree.find("CircularToolConstraint") != tree.end()) {
            const PTree& child = tree.get_child("CircularToolConstraint");

            return CircularToolConstraint(
                readToolSpeed(child),
                XML::readVector3D(child),
                readFrame(child, workcell));

        } else if (tree.find("LinearJointConstraint") != tree.end()) {
            return LinearJointConstraint();
        }

        else {
            RW_THROW("No link constraint found.");

            // To avoid a compiler warning.
            return LinearJointConstraint();
        }
    }

    Link readLink(const PTree& tree, const WorkCell& workcell)
    {
        return Link(
            readEntity(tree),
            readLinkConstraint(tree, workcell));
    }

    ToolLocation readToolLocation(const PTree& tree, const WorkCell& workcell)
    {
        return ToolLocation(
            XML::readTransform3D(tree.get_child("Transform3D")),
            readFrame(tree, workcell));
    }

    Target readTarget(
        const PTree& tree, const WorkCell& workcell, const Device& device)
    {
        const Entity entity = readEntity(tree);

        if (tree.find("Tool") != tree.end()) {
            return Target(
                entity,
                readToolLocation(tree.get_child("Tool"), workcell));
        } else if (tree.find("Joint") != tree.end()) {
            const Q q = XML::readQ(tree.get_child("Joint"));

            // We sanity check that q has the right dimension.
            if (q.size() != device.getDOF())
                RW_THROW(
                    "Configuration "
                    << q
                    << " for device "
                    << device
                    << " should have been of dimension "
                    << (int)device.getDOF());

            return Target(entity, q);
        } else {
            RW_THROW("No target specified. <Tool> or <Joint> expected.");

            // To avoid a compiler warning.
            return readTarget(tree, workcell, device);
        }
    }

    Frame* readTCP(const PTree& tree, Device* device, const WorkCell& workcell)
    {
        boost::optional<string> tcp_name = tree.get_optional<string>("TCP");
        if (!tcp_name) {
            return device->getEnd();
        } else {
            Frame* tcp = workcell.findFrame(*tcp_name);

            if (!tcp)
                RW_THROW(
                    "TCP frame "
                    << quote(*tcp_name)
                    << " not found in workcell "
                    << workcell);

            return tcp;
        }
    }

    Device* readDevice(const PTree& tree, const WorkCell& workcell)
    {
        const string device_name = tree.get<string>("Device");
        Device* device = workcell.findDevice(device_name);
        if (!device)
            RW_THROW(
                "No device named "
                << quote(device_name)
                << " in workcell "
                << workcell);
        return device;
    }

    Trajectory readTrajectory(const PTree& tree, WorkCell* workcell)
    {
        const Entity entity = readEntity(tree);
        Device* device = readDevice(tree, *workcell);
        Frame* tcp = readTCP(tree, device, *workcell);

        std::vector<Trajectory::value_type> elements;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Target") {
                elements.push_back(
                    readTarget(p->second, *workcell, *device));
            } else if (p->first == "Link") {
                elements.push_back(
                    readLink(p->second, *workcell));
            } else if (p->first != "Device" || p->first != "TCP") {
                // Nothing to do.
            } else {
                RW_THROW(
                    "Unexpected XML tag "
                    << quote(p->first)
                    << ". <Target>, <Link>, <Device>, or <TCP> expected.");
            }
        }

        return Trajectory(entity, workcell, device, tcp, elements);
    }

    AttachFrame readAttachFrame(const PTree& tree, const WorkCell& workcell)
    {
        const Entity entity = readEntity(tree);

        const string item_name = tree.get<string>("Item");
        const string tcp_name = tree.get<string>("TCP");

        MovableFrame* item =
            dynamic_cast<MovableFrame*>(
                workcell.findFrame(item_name));

        if (!item) {
            RW_THROW(
                "No movable frame named "
                << quote(item_name)
                << " in workcell "
                << workcell);
        }

        Frame* tcp = workcell.findFrame(tcp_name);
        if (!tcp)
            RW_THROW(
                "No TCP frame named "
                << quote(tcp_name)
                << " in workcell "
                << workcell);

        return AttachFrame(entity, item, tcp);
    }

    WorkCellPtr getWorkCellOptionally(
        const PTree& tree, WorkCellPtr optional_workcell)
    {
        if (optional_workcell)
            return optional_workcell;
        else {
            const string workcell_name = tree.get<string>("WorkCell");
            return WorkCellLoader::load(workcell_name);
        }
    }

    Task readTask(const PTree& tree, WorkCellPtr optional_workcell)
    {
        const Entity entity = readEntity(tree);

        WorkCellPtr workcell =
            getWorkCellOptionally(tree, optional_workcell);

        std::vector<Task::value_type> actions;

        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Trajectory") {
                actions.push_back(
                    readTrajectory(p->second, workcell.get()));
            } else if (p->first == "AttachFrame") {
                actions.push_back(
                    readAttachFrame(p->second, *workcell));
            }
        }

        return Task(entity, workcell, actions);
    }
}

Task TaskLoader::load(const string& file, WorkCellPtr optional_workcell)
{
    try {
        PTree tree;
        read_xml(file, tree);

        // XML::printTree(tree);

        return readTask(tree.get_child("Task"), optional_workcell);

    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }

    // To avoid a compiler warning.
    RW_ASSERT(!"Impossible");
    return TaskLoader::load(file, optional_workcell);
}
