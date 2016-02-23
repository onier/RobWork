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

#include "NURBSSurface.hpp"
#include <RobWorkConfig.hpp>
#include <opennurbs/src/opennurbs.h>

using namespace rw::common;
using namespace rw::geometry;

namespace rwlibs {
namespace nurbs {

GeometryData::GeometryType NURBSSurface::getType() const {
	return GeometryData::UserType;
}

Ptr<TriMesh> NURBSSurface::getTriMesh(bool forceCopy) {
	return NULL;
}

NURBSSurface::NURBSSurface() {
}

NURBSSurface::NURBSSurface(rw::geometry::Box box) {
	ON_3dPoint p1(0,0,0);
	ON_3dPoint p2(0.1,0.1,0.1);
	ON_BoundingBox bbox(p1,p2);
	ON_Box onbox(bbox);
	/*ON_NurbsSurface* nurbs = new ON_NurbsSurface();
	ON_NurbsSurface nurbs2;
	box.GetNurbForm(nurbs2);
	nurbs->CopyFrom(&nurbs2);
	_patches.push_back(ownedPtr(new NURBSPatch(nurbs)));*/
}

NURBSSurface::NURBSSurface(rw::geometry::Cylinder cylinder) {
	ON_Circle circle;
	ON_Cylinder cyl(circle,cylinder.getHeight());
	ON_Brep* cyl_brep = ON_BrepCylinder(cyl,true,true);
	for (unsigned int i = 0; cyl_brep->Face(i) != NULL; i++) {
		const ON_Surface* surface = cyl_brep->Face(i)->SurfaceOf();
		/*ON_NurbsSurface* nurbs = new ON_NurbsSurface();
		surface->GetNurbForm(*nurbs,0.0001);*/
		const ON_NurbsSurface* nurbs = ON_NurbsSurface::Cast(surface);
		if (nurbs != NULL) {
			ON_NurbsSurface* nurbsNew = new ON_NurbsSurface();
			nurbsNew->CopyFrom(nurbs);
			_patches.push_back(ownedPtr(new NURBSPatch(nurbsNew)));
		}
		const ON_RevSurface* revSurface = ON_RevSurface::Cast(surface);
		if (revSurface != NULL) {
			ON_NurbsSurface* nurbsNew = new ON_NurbsSurface();
			revSurface->GetNurbForm(*nurbsNew);
			nurbsNew->CopyFrom(nurbs);
			_patches.push_back(ownedPtr(new NURBSPatch(nurbsNew)));
		}
	}
}

NURBSSurface::~NURBSSurface() {
}

void NURBSSurface::addPatch(NURBSPatch::Ptr patch) {
	_patches.push_back(patch);
}

std::vector<NURBSPatch::Ptr> NURBSSurface::getPatches() {
	return _patches;
}

void NURBSSurface::save3DM(const std::string & file) const {
	ONX_Model model;
	ON_3dmObjectAttributes attribs;
	// some notes
	time_t now;
	time(&now);
	ON_wString notes = "Saved in RobWork revision ";
	notes = notes + RW_REVISION;
	notes = notes + "\r\n";
	notes = notes + "Date and time: ";
	notes = notes + ctime(&now);

	model.m_properties.m_Notes.m_notes = notes;
	model.m_properties.m_Notes.m_bVisible = (model.m_properties.m_Notes.m_notes.Length() > 0);

	// set revision history information
	model.m_properties.m_RevisionHistory.NewRevision();

	// set application information
	ON_wString appName = "RobWork ";
	appName = appName + RW_VERSION;
	model.m_properties.m_Application.m_application_name = appName;
	model.m_properties.m_Application.m_application_URL = "http://robwork.dk";
	model.m_properties.m_Application.m_application_details = "Framework for simulation and control of robot systems.";

	// file settings (units, tolerances, views, ...)
	model.m_settings.m_ModelUnitsAndTolerances.m_unit_system = ON::meters;
	model.m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance = 0.001;
	model.m_settings.m_ModelUnitsAndTolerances.m_angle_tolerance = ON_PI/180.0; // radians
	model.m_settings.m_ModelUnitsAndTolerances.m_relative_tolerance = 0.01; // 1%

	// layer table
	{
		// OPTIONAL - define some layers
		ON_Layer layer[1];

		layer[0].SetLayerName("Default");
		layer[0].SetVisible(true);
		layer[0].SetLocked(false);
		layer[0].SetLayerIndex(0);
		layer[0].SetColor( ON_Color(0,0,0) );

		model.m_layer_table.Append(layer[0]);
	}

	for (std::size_t i = 0; i < _patches.size(); i++) {
		ONX_Model_Object& mo = model.m_object_table.AppendNew();
		mo.m_object = _patches[i]->getRaw();
		mo.m_bDeleteObject = false;
		mo.m_attributes.m_layer_index = 0;
		std::stringstream str;
		str << "NURBS Patch " << i;
		mo.m_attributes.m_name = str.str().c_str();
	}

	FILE* fp = ON::OpenFile( file.c_str(), "wb" );
	ON_BinaryFile archive( ON::write3dm, fp );
	model.Polish();
	// writes model to archive
	std::string comment = "NURBSPatch::save3DM(...) ";
	comment = comment + ctime(&now);
	model.Write( archive,
			ON_BinaryArchive::CurrentArchiveVersion(),
			comment.c_str(),
			NULL );
}

NURBSSurface::Ptr NURBSSurface::load3DM(const std::string &file) {
	ONX_Model model;
	FILE* archive_fp = ON::OpenFile( file.c_str(), "rb");
	ON_BinaryFile archive( ON::read3dm, archive_fp );
	model.Read( archive );
	ON::CloseFile( archive_fp );
	int objNo = model.m_object_table.Count();
	NURBSSurface::Ptr surface = ownedPtr(new NURBSSurface());
	for (int i = 0; i < objNo; i++) {
		const ON_NurbsSurface* obj = ON_NurbsSurface::Cast(model.m_object_table.At(i)->m_object);
		ON_NurbsSurface* newObj = new ON_NurbsSurface();
		newObj->CopyFrom(obj);
		NURBSPatch::Ptr patch = ownedPtr(new NURBSPatch(newObj));
		surface->addPatch(patch);
	}
	return surface;
}

} /* namespace nurbs */
} /* namespace rwlibs */
