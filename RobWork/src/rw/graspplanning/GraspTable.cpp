
#include "GraspTable.hpp"

#include <fstream>
#include <iostream>

using namespace rw::graspplanning;

GraspTable::GraspTable(const std::string& handName,  const std::string& objectId):
    _handName(handName),_objectId(objectId)
{

}

void GraspTable::addGrasp(GraspData& data)
{
	std::cout << "Adding grasp";
    _graspData.push_back(data);
}

GraspTable* GraspTable::load(const std::string& filename){
    return new GraspTable("","");
}

void GraspTable::save(const std::string& filename){
    std::ofstream fstr(filename.c_str());
    std::cout << "saving grasp stuff" << std::endl;

    fstr << "hand: " << _handName << "\n";
    std::cout << "hand: " << _handName << "\n";
    fstr << "object: " << _objectId << "\n";
    std::cout << "object: " << _objectId << "\n";
    fstr << "TableSize: " << _graspData.size() << "\n";
    std::cout << "TableSize: " << _graspData.size() << "\n";
    if(_graspData.size()==0){
    	fstr.close();
    	return;
    }
    fstr << "HandDOF: " << _graspData[0].pq.size() << "\n";
    std::cout << "HandDOF: " << _graspData[0].pq.size() << "\n";
    fstr << "NrOfQualityMeasures: " << _graspData[0].quality.size() << "\n";
    std::cout << "NrOfQualityMeasures: " << _graspData[0].quality.size() << "\n";
    // each data entry is printed on a line
    for(size_t i=0;i<_graspData.size();i++){
        GraspData &data = _graspData[i];

        // first we print the approach vector
        fstr << data.approach[0] << ";" << data.approach[1] << ";" << data.approach[2] << ";";
        //std::cout << data.approach[0] << ";" << data.approach[1] << ";" << data.approach[2] << ";";
        // then we print the quality
        for(size_t j=0;j<data.quality.size();j++){
            fstr << data.quality[j] << ";";
            //std::cout << data.quality[j] << ";";
        }

        // then the preshape configuration
        for(size_t j=0;j<data.pq.size();j++){
            fstr << data.pq[j] << ";";
            //std::cout << data.pq[j] << ";";
        }

        // hand pose
        for(size_t j=0;j<6;j++){
            fstr << data.hp.get(j) << ";";
            std::cout << data.hp.get(j) << ";";
        }

        // object pose
        for(size_t j=0;j<6;j++){
            fstr << data.op.get(j) << ";";
            std::cout <<data.op.get(j) << ";";
        }

        // then we print the contact configuration
        fstr << data.cq.size() << ";";
        std::cout << data.cq.size() << ";";
        for(size_t j=0;j<data.cq.size();j++){
            fstr << data.cq[j] << ";";
            std::cout << data.cq[j] << ";";
        }

        // now comes the actual contacts
        fstr << data.grasp.contacts.size() << ";";
        for(size_t j=0;j<data.grasp.contacts.size();j++){
            fstr << data.grasp.contacts[j].p[0] << ";";
            fstr << data.grasp.contacts[j].p[1] << ";";
            fstr << data.grasp.contacts[j].p[2] << ";";
            fstr << data.grasp.contacts[j].n[0] << ";";
            fstr << data.grasp.contacts[j].n[1] << ";";
            fstr << data.grasp.contacts[j].n[2] << ";";
        }

        // we end the data entry with a endline
        fstr << std::endl;
        std::cout << std::endl;
    }

    fstr.close();
}


