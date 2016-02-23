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

#include "NURBSPatch.hpp"
#include <RobWorkConfig.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;

namespace rwlibs {
namespace nurbs {

GeometryData::GeometryType NURBSPatch::getType() const {
	return GeometryData::UserType;
}

rw::common::Ptr<rw::geometry::TriMesh> NURBSPatch::getTriMesh(bool forceCopy) {
	return NULL;
}

NURBSPatch::Ptr NURBSPatch::make(
		unsigned int orderU,
		unsigned int orderV,
		std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
		std::vector<double> knotsU,
		std::vector<double> knotsV
)
{
	validate(orderU,orderV,controlPoints,knotsU,knotsV);
	return ownedPtr(new NURBSPatch(orderU,orderV,controlPoints,knotsU,knotsV));
}

NURBSPatch::Ptr NURBSPatch::make(
		unsigned int orderU,
		unsigned int orderV,
		std::vector<std::vector<rw::math::Vector3D<> > > controlPoints,
		std::vector<double> knotsU,
		std::vector<double> knotsV,
		std::vector<std::vector<double> > weights
)
{
	validate(orderU,orderV,controlPoints,knotsU,knotsV);
	if (weights.size() != controlPoints.size())
			RW_THROW("Not same number of weights in U parameter as number of control points.");
	if (weights[0].size() != controlPoints[0].size())
			RW_THROW("Not same number of weights in V parameter as number of control points.");
	return ownedPtr(new NURBSPatch(orderU,orderV,controlPoints,knotsU,knotsV,weights));
}

ON_NurbsSurface* NURBSPatch::getRaw() {
	return _surface;
}

void NURBSPatch::validate(
		unsigned int orderU,
		unsigned int orderV,
		const std::vector<std::vector<rw::math::Vector3D<> > > &controlPoints,
		std::vector<double> &knotsU,
		std::vector<double> &knotsV
) {
	if (orderU < 2 || orderV < 2)
		RW_THROW("Order must be at least 2 in both parameters U and V.");
	std::size_t sizeU = controlPoints.size();
	if (sizeU < orderU+1) {
		std::stringstream str;
		str << "Number of control points in parameter U must be at least " << (orderU+1) << "." << " It was of size " << sizeU << "." << std::endl;
		RW_THROW(str.str());
	}
	std::vector<std::vector<Vector3D<> > >::const_iterator it = controlPoints.begin();
	std::size_t sizeV = (*it).size();
	if (sizeV < orderV+1) {
		std::stringstream str;
		str << "Number of control points in parameter V must be at least " << (orderV+1) << "." << " It was of size " << sizeV << "." << std::endl;
		RW_THROW(str.str());
	}
	for (it++; it < controlPoints.end(); it++) {
		if ((*it).size() != sizeV)
			RW_THROW("Number of control points in parameter V is not equal for each row in parameter U.");
	}
	if (knotsU.size() == orderU+sizeU) {
		knotsU.erase(knotsU.begin());
		knotsU.erase(knotsU.end());
	}
	if (knotsV.size() == orderV+sizeV) {
		knotsU.erase(knotsV.begin());
		knotsU.erase(knotsV.end());
	}
	if (knotsU.size() != orderU+sizeU-2) {
		std::stringstream str;
		str << "Number of knots in parameter U is expected to be " << (orderU+sizeU-2) << " or " << (orderU+sizeU) << " (if superfluous duplicates included at beginning and end). It was " << knotsU.size() << "." << std::endl;
		RW_THROW(str.str());
	}
	if (knotsV.size() != orderV+sizeV-2) {
		std::stringstream str;
		str << "Number of knots in parameter V is expected to be " << (orderV+sizeV-2) << " or " << (orderV+sizeV) << " (if superfluous duplicates included at beginning and end).. It was " << knotsV.size() << "." << std::endl;
		RW_THROW(str.str());
	}

	std::vector<double>::const_iterator knotItU = knotsU.begin();
	double valU = *knotItU;
	unsigned int repU = 1;
	for (knotItU++; knotItU < knotsU.end(); knotItU++) {
		if (*knotItU == valU) {
			repU++;
			if (repU > orderU-1) {
				std::stringstream str;
				str << "The multiplicity of a knot in parameter U was higher than the degree " << (orderU-1) << "." << std::endl;
				RW_THROW(str.str());
			}
		} else if (*knotItU < valU)
			RW_THROW("Knots must be non-decreasing. Problem occurred in parameter U.");
		else {
			repU = 1;
			valU = *knotItU;
		}
	}

	std::vector<double>::const_iterator knotItV = knotsV.begin();
	double valV = *knotItV;
	unsigned int repV = 1;
	for (knotItV++; knotItV < knotsV.end(); knotItV++) {
		if (*knotItV == valV) {
			repV++;
			if (repV > orderV-1) {
				std::stringstream str;
				str << "The multiplicity of a knot in parameter V was higher than the degree " << (orderV-1) << "." << std::endl;
				RW_THROW(str.str());
			}
		} else if (*knotItV < valV)
			RW_THROW("Knots must be non-decreasing. Problem occurred in parameter V.");
		else {
			repV = 1;
			valV = *knotItV;
		}
	}
}

NURBSPatch::NURBSPatch(ON_NurbsSurface* surface):
	_surface(surface)
{
}

NURBSPatch::NURBSPatch(
		unsigned int orderU,
		unsigned int orderV,
		std::vector<std::vector<Vector3D<> > > controlPoints,
		std::vector<double> knotsU,
		std::vector<double> knotsV
):
	_surface(ON_NurbsSurface::New(3,true,orderU,orderV,controlPoints.size(),controlPoints[0].size()))
{
	for (std::size_t i = 0; i < controlPoints.size(); i++) {
		for (std::size_t j = 0; j < controlPoints[i].size(); j++) {
			ON_3dPoint point;
			point.x = controlPoints[i][j][0];
			point.y = controlPoints[i][j][1];
			point.z = controlPoints[i][j][2];
			_surface->SetCV(i,j,point);
		}
	}
	for (std::size_t i = 0; i < knotsU.size(); i++)
		_surface->SetKnot(0,i,knotsU[i]);
	for (std::size_t i = 0; i < knotsV.size(); i++)
		_surface->SetKnot(1,i,knotsV[i]);
}

NURBSPatch::NURBSPatch(
		unsigned int orderU,
		unsigned int orderV,
		std::vector<std::vector<Vector3D<> > > controlPoints,
		std::vector<double> knotsU,
		std::vector<double> knotsV,
		std::vector<std::vector<double> > weights
):
	_surface(ON_NurbsSurface::New(3,false,orderU,orderV,controlPoints.size(),controlPoints[0].size()))
{
}

NURBSPatch::~NURBSPatch() {
	delete _surface;
}

std::pair<unsigned int,unsigned int> NURBSPatch::getDegree() const {
	std::pair<unsigned int,unsigned int> pair;
	pair.first = _surface->Degree(0);
	pair.second = _surface->Degree(1);
	return pair;
}

std::pair<unsigned int,unsigned int> NURBSPatch::getOrder() const {
	std::pair<unsigned int,unsigned int> pair;
	pair.first = _surface->Order(0);
	pair.second = _surface->Order(1);
	return pair;
}

std::vector<double> NURBSPatch::getKnotsU() const {
	const double* knots = _surface->Knot(0);
	int knotCount = _surface->KnotCount(0);
	std::vector<double> res;
	for (int i = 0; i < knotCount; i++) {
		res.push_back(knots[i]);
	}
	return res;
}

std::vector<double> NURBSPatch::getKnotsV() const {
	const double* knots = _surface->Knot(1);
	int knotCount = _surface->KnotCount(1);
	std::vector<double> res;
	for (int i = 0; i < knotCount; i++) {
		res.push_back(knots[i]);
	}
	return res;
}

std::vector<std::vector<Vector3D<> > > NURBSPatch::getControlPoints() const {
	std::vector<std::vector<Vector3D<> > > outer;
	int cntU = _surface->CVCount(0);
	int cntV = _surface->CVCount(1);
	for (int i = 0; i < cntU; i++) {
		std::vector<Vector3D<> > inner;
		for (int j = 0; j < cntV; j++) {
			const double* val = _surface->CV(i,j);
			Vector3D<> vec(val[0],val[1],val[2]);
			inner.push_back(vec);
		}
		outer.push_back(inner);
	}
	return outer;
}

std::vector<std::vector<double> > NURBSPatch::getWeights() const {
	std::vector<std::vector<double> > outer;
	int cntU = _surface->CVCount(0);
	int cntV = _surface->CVCount(1);
	for (int i = 0; i < cntU; i++) {
		std::vector<double> inner;
		for (int j = 0; j < cntV; j++) {
			inner.push_back(_surface->Weight(i,j));
		}
		outer.push_back(inner);
	}
	return outer;
}

void NURBSPatch::save3DM(const std::string & file) const {
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
		layer[1].SetLayerIndex(0);
		layer[0].SetColor( ON_Color(0,0,0) );

		model.m_layer_table.Append(layer[0]);
	}

	ONX_Model_Object& mo = model.m_object_table.AppendNew();
	mo.m_object = _surface;
	mo.m_bDeleteObject = false;
	mo.m_attributes.m_layer_index = 0;
	mo.m_attributes.m_name = "NURBS Patch";

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

NURBSPatch::Ptr NURBSPatch::load3DM(const std::string &file) {
	 ONX_Model model;
	 FILE* archive_fp = ON::OpenFile( file.c_str(), "rb");
	 ON_BinaryFile archive( ON::read3dm, archive_fp );
	 model.Read( archive );
	 ON::CloseFile( archive_fp );
	 const ON_NurbsSurface* obj = ON_NurbsSurface::Cast(model.m_object_table.At(0)->m_object);
	 ON_NurbsSurface* newObj = new ON_NurbsSurface();
	 newObj->CopyFrom(obj);
	 return ownedPtr(new NURBSPatch(newObj));
}

} /* namespace nurbs */
} /* namespace rwlibs */
