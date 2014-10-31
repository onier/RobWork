#include "DeformableObject.hpp"


using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::models;

DeformableObject::DeformableObject(rw::kinematics::Frame* baseframe, int nr_of_nodes):
	Object(baseframe),_rstate(1, rw::common::ownedPtr( new DeformableObjectCache(nr_of_nodes)).cast<StateCache>() )
{
	add(_rstate);
}

DeformableObject::~DeformableObject(){

}

rw::math::Vector3D<float>& DeformableObject::getNode(int id, rw::kinematics::State& state) const
{
	return _rstate.getStateCache<DeformableObjectCache>(state)->_nodes[id];
}

const rw::math::Vector3D<float>& DeformableObject::getNode(int id, const rw::kinematics::State& state) const{
	return _rstate.getStateCache<DeformableObjectCache>(state)->_nodes[id];
}

const std::vector<rw::geometry::IndexedTriangle<> >& DeformableObject::getFaces() const {
	return _faces;
}

void DeformableObject::addFace(unsigned int node1, unsigned int node2, unsigned int node3){
	_faces.push_back(rw::geometry::IndexedTriangle<>(node1,node2,node3) );
}

rw::geometry::IndexedTriMesh<>::Ptr DeformableObject::getMesh(rw::kinematics::State& cstate){
	return NULL;
}


 const std::vector<rw::geometry::Geometry::Ptr>& DeformableObject::getGeometry(const rw::kinematics::State& state) const{
	 return _geoms;
 }

 const std::vector<rw::graphics::Model3D::Ptr>& DeformableObject::getModels() const{
	 return _models;
 }

 double DeformableObject::getMass(rw::kinematics::State& state) const{
	 return 1.0;
 }

 rw::math::Vector3D<> DeformableObject::getCOM(rw::kinematics::State& state) const{
	 return rw::math::Vector3D<>(0,0,0);
 }

 rw::math::InertiaMatrix<> DeformableObject::getInertia(rw::kinematics::State& state) const{
	 return rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1);
 }
