#include "ProximityModel.hpp"

#include "ProximityStrategy.hpp"

using namespace rw::proximity;

ProximityModel::~ProximityModel()
{
};

bool ProximityModel::addGeometry(const rw::geometry::Geometry& geom){
	return owner->addGeometry(this, geom);
}

bool ProximityModel::removeGeometry(const std::string& geom){
	return owner->removeGeometry(this, geom);
}

std::vector<std::string> ProximityModel::getGeometryIDs(){
	return owner->getGeometryIDs(this);
}
