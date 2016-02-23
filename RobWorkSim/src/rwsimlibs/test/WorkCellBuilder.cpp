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

#include "WorkCellBuilder.hpp"

#include <rw/geometry/Box.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/graphics/Model3D.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rwsimlibs::test;

#define PAH_DARKGRAY Vector3D<float>(98.f,98.f,98.f)/255.f
#define PAH_LIGHTGRAY Vector3D<float>(149.f,149.f,149.f)/255.f
#define PAH_BLUE Vector3D<float>(64.f,116.f,142.f)/255.f
#define PAH_ORANGE Vector3D<float>(227.f,104.f,12.f)/255.f

WorkCellBuilder::ColorScheme::ColorScheme():
	fixedBodies(Vector3D<float>(0.5f,0.5f,0.5f)),
	kinematicBodies(fixedBodies),
	dynamicBodies(fixedBodies)
{
}

WorkCellBuilder::ColorScheme::ColorScheme(const Vector3D<float>& color):
	fixedBodies(color), kinematicBodies(color), dynamicBodies(color)
{
}

WorkCellBuilder::ColorScheme::ColorScheme(const Vector3D<float>& fixedBodies, const Vector3D<float>& kinematicBodies, const Vector3D<float>& dynamicBodies):
	fixedBodies(fixedBodies), kinematicBodies(kinematicBodies), dynamicBodies(dynamicBodies)
{
}

WorkCellBuilder::PaHColors::PaHColors():
	ColorScheme(PAH_BLUE, PAH_LIGHTGRAY, PAH_ORANGE)
{
}

WorkCellBuilder::WorkCellBuilder(const ColorScheme& colors):
	_colors(colors)
{
}

WorkCellBuilder::~WorkCellBuilder() {
}

void WorkCellBuilder::addFloor(WorkCell::Ptr wc, const std::string& name, bool trimesh) const {
	addPlane(wc,Vector3D<>::z(),0,name, trimesh);
}

void WorkCellBuilder::addPlane(WorkCell::Ptr wc, const rw::math::Vector3D<>& n, double d, const std::string& name, bool trimesh) const {
	FixedFrame* const frame = new FixedFrame(name,Transform3D<>::identity());
	wc->addFrame(frame,wc->getWorldFrame());

	const Plane::Ptr geoData = ownedPtr(new Plane(n,d));
	Geometry::Ptr geo;
	if (trimesh)
		geo = ownedPtr(new Geometry(geoData->createMesh(0,5), name));
	else
		geo = ownedPtr(new Geometry(geoData, name));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material(name+"Material",_colors.fixedBodies[0],_colors.fixedBodies[1],_colors.fixedBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D(name));
	model->addTriMesh(material,*geoData->createMesh(0,5));
	robject->addModel(model);

	wc->add(robject);
}

void WorkCellBuilder::addBox(WorkCell::Ptr wc, double x, double y, double z, double density, const std::string& name, bool trimesh) const {
	MovableFrame* const frame = new MovableFrame(name);
	wc->addFrame(frame,wc->getWorldFrame());

	GeometryData::Ptr geoData = ownedPtr(new Box(x, y, z));
	if (trimesh)
		geoData = geoData->getTriMesh(true);
	Geometry::Ptr geo = ownedPtr(new Geometry(geoData, "Box"));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material("BoxMaterial",_colors.dynamicBodies[0],_colors.dynamicBodies[1],_colors.dynamicBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D("Box"));
	model->addTriMesh(material,*geoData->getTriMesh());
	robject->addModel(model);

	wc->add(robject);
}
