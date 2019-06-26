/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#include "AnalyticGeometryPlugin.hpp"
#include "QuadraticTestObjects.hpp"

#include <rw/geometry/GeometryData.hpp>
#include <rw/geometry/analytic/quadratics/QuadraticBREP.hpp>

using rw::common::Extension;
using rw::geometry::GeometryData;
using namespace rwlibs::geometry;

RW_ADD_PLUGIN(AnalyticGeometryPlugin)

AnalyticGeometryPlugin::AnalyticGeometryPlugin():
    Plugin("AnalyticGeometryPlugin", "AnalyticGeometryPlugin", "1.0")
{
}

AnalyticGeometryPlugin::~AnalyticGeometryPlugin()
{
}

std::vector<Extension::Descriptor> AnalyticGeometryPlugin::getExtensionDescriptors()
{
    std::vector<Extension::Descriptor> exts;

    exts.push_back(Extension::Descriptor("QuadraticTestObjectA", "rw.loaders.GeometryFactory"));
    exts.back().getProperties().set<std::string>("type", "QuadraticTestObjectA");
    exts.push_back(Extension::Descriptor("QuadraticTestObjectB", "rw.loaders.GeometryFactory"));
    exts.back().getProperties().set<std::string>("type", "QuadraticTestObjectB");

    return exts;
}

Extension::Ptr AnalyticGeometryPlugin::makeExtension(const std::string& id)
{
    if (id == "QuadraticTestObjectA") {
        const Extension::Ptr extension = ownedPtr(new Extension(
                "QuadraticTestObjectA","rw.loaders.GeometryFactory", this,
                QuadraticTestObjects::objectA().cast<GeometryData>()));
        extension->getProperties().set<std::string>("type", "QuadraticTestObjectA");
        return extension;
    } else if (id == "QuadraticTestObjectB") {
        const Extension::Ptr extension = ownedPtr(new Extension(
                "QuadraticTestObjectB","rw.loaders.GeometryFactory",
                this, QuadraticTestObjects::objectB().cast<GeometryData>()));
        extension->getProperties().set<std::string>("type", "QuadraticTestObjectB");
        return extension;
    }

    return NULL;
}
