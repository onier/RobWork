/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/ContactDataMap.hpp>
#include <rwsim/dynamics/MaterialDataMap.hpp>
#include "RWPEBody.hpp"
#include "RWPEFrictionModel.hpp"
#include "RWPEMaterialMap.hpp"

#include "RWPEContact.hpp"
#include "RWPERestitutionModel.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEMaterialMap::RWPEMaterialMap(const ContactDataMap &contactDataMap, const MaterialDataMap &materialDataMap) {
	// Construct list of material names (friction)
	const int maxMatId = materialDataMap.getMaxMatID();
	_idToMat.resize(maxMatId);
	for (int i = 0; i < maxMatId; i++) {
		_idToMat[i] = materialDataMap.getMaterialName(i);
	}
	// Construct list of object type names (collisions)
	const int maxTypeId = contactDataMap.getMaxID();
	_idToType.resize(maxTypeId);
	for (int i = 0; i < maxTypeId; i++) {
		_idToType[i] = contactDataMap.getObjectTypeName(i);
	}
	// Construct friction models for all pairs of materials
	_frictionModels.resize(maxMatId+1,std::vector<const RWPEFrictionModel*>(maxMatId+1,NULL));
	for (int i = 0; i < maxMatId; i++) {
		for (int j = i; j < maxMatId; j++) {
			std::string modelId;
			PropertyMap parameters;
			std::vector<FrictionData> datas = materialDataMap.getFrictionDatas(i,j);
			bool found = false;
			if (datas.size() == 0) {
				const FrictionData& defData = materialDataMap.getDefaultFriction(Coulomb);
				datas.push_back(defData);
				datas.back().typeName = "Coulomb";
			}
			BOOST_FOREACH(const FrictionData& data, datas) {
				modelId = data.typeName;
				if (RWPEFrictionModel::Factory::hasModel(modelId)) {
					BOOST_FOREACH(const FrictionParam& pars, data.parameters) {
						if (pars.second.size() == 1)
							parameters.set(pars.first,pars.second[0]);
					}
					found = true;
					break;
				}
			}
			if (!found) {
				RW_THROW("RWPEMaterialMap (RWPEMaterialMap): Could not find correct friction data for \"" << _idToMat[i] << "\" and \"" << _idToMat[j] << "\".");
			}
			const RWPEFrictionModel* model = RWPEFrictionModel::Factory::makeModel(modelId, parameters);
			_frictionModels[i][j] = model;
			if (i != j)
				_frictionModels[j][i] = _frictionModels[i][j];
		}
	}
	// Construct restitution models for all pairs of object types
	_restitutionModels.resize(maxTypeId+1,std::vector<const RWPERestitutionModel*>(maxTypeId+1,NULL));
	for (int i = 0; i < maxTypeId; i++) {
		for (int j = i; j < maxTypeId; j++) {
			const std::string modelId = "Newton";
			PropertyMap parameters;
			parameters.set("cr",contactDataMap.getNewtonData(i,j).cr);
			parameters.set("LinearThreshold",0.00001); // 1/100 mm per second (as ODE)
			parameters.set("AngularThreshold",1*Deg2Rad); // 1 degrees per second
			const RWPERestitutionModel* model = RWPERestitutionModel::Factory::makeModel(modelId, parameters);
			_restitutionModels[i][j] = model;
			if (i != j)
				_restitutionModels[j][i] = _restitutionModels[i][j];
		}
	}
}

RWPEMaterialMap::~RWPEMaterialMap() {
	for (std::size_t i = 0; i < _frictionModels.size(); i++) {
		for (std::size_t j = i; j < _frictionModels[i].size(); j++) {
			delete _frictionModels[i][j];
		}
	}
	_frictionModels.clear();
	for (std::size_t i = 0; i < _restitutionModels.size(); i++) {
		for (std::size_t j = i; j < _restitutionModels[i].size(); j++) {
			delete _restitutionModels[i][j];
		}
	}
	_restitutionModels.clear();
}

const RWPEFrictionModel& RWPEMaterialMap::getFrictionModel(const RWPEBody &bodyA, const RWPEBody &bodyB) const {
	const std::string typeA = bodyA.get()->getInfo().material;
	const std::string typeB = bodyB.get()->getInfo().material;
	bool foundA = false;
	bool foundB = false;
	std::size_t idA;
	std::size_t idB;
	for (std::size_t i = 0; i < _idToMat.size(); i++) {
		if (!foundA) {
			if (_idToMat[i] == typeA) {
				foundA = true;
				idA = i;
			}
		}
		if (!foundB) {
			if (_idToMat[i] == typeB) {
				foundB = true;
				idB = i;
			}
		}
		if (foundA && foundB)
			break;
	}
	if (!foundA || !foundB)
		RW_THROW("RWPEMaterialMap (getFrictionModel): could not find a friction model for the given bodies \"" << bodyA.get()->getName() << "\" and \"" << bodyB.get()->getName() << "\".");
	return *(_frictionModels[idA][idB]);
}

const RWPERestitutionModel& RWPEMaterialMap::getRestitutionModel(const RWPEBody &bodyA, const RWPEBody &bodyB) const {
	const std::string typeA = bodyA.get()->getInfo().objectType;
	const std::string typeB = bodyB.get()->getInfo().objectType;
	bool foundA = false;
	bool foundB = false;
	std::size_t idA;
	std::size_t idB;
	for (std::size_t i = 0; i < _idToType.size(); i++) {
		if (!foundA) {
			if (_idToType[i] == typeA) {
				foundA = true;
				idA = i;
			}
		}
		if (!foundB) {
			if (_idToType[i] == typeB) {
				foundB = true;
				idB = i;
			}
		}
		if (foundA && foundB)
			break;
	}
	if (!foundA || !foundB)
		RW_THROW("RWPEMaterialMap (getRestitutionModel): could not find a restitution model for the given bodies \"" << bodyA.get()->getName() << "\" and \"" << bodyB.get()->getName() << "\".");
	return *(_restitutionModels[idA][idB]);
}

const RWPERestitutionModel& RWPEMaterialMap::getRestitutionModel(const RWPEContact &contact) const {
	return getRestitutionModel(*contact.getParent(),*contact.getChild());
}