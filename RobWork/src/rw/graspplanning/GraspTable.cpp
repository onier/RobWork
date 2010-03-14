
#include "GraspTable.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Pose6D.hpp>

#include <fstream>
#include <iostream>
#include <cstdio>
using namespace rw::math;

using namespace rw::graspplanning;

GraspTable::GraspTable(const std::string& handName,  const std::string& objectId):
    _handName(handName),_objectId(objectId)
{

}

void GraspTable::addGrasp(GraspData& data)
{
//	std::cout << "Adding grasp";
    _graspData.push_back(data);
}

GraspTable* GraspTable::load(const std::string& filename){
	const int linesize = 1000;
	char line[linesize],chunk[100],handname[100],objectId[100];
	handname[0] = 0; // make sure that if data has noname it will have no name
	objectId[0] = 0;
	std::cout << filename << std::endl;
	std::ifstream istr( filename.c_str() );
	int res;
	if( !istr.is_open() )
		RW_THROW("Could not open file: "<< filename);

	unsigned int tablesize, handdof, nrquality;
	istr.getline(line, linesize);
	sscanf (line,"%s %s",chunk, handname);
	istr.getline(line, linesize);
	sscanf (line,"%s %s",chunk, objectId);
	istr.getline(line, linesize);
	sscanf (line,"%s %i",chunk,&tablesize);
    // fstr << "TableSize: " << _graspData.size() << "\n";

    GraspTable *gtable = new GraspTable(handname,objectId);

    std::cout << "hand: " << handname << "\n";
    std::cout << "object: " << objectId << "\n";
    std::cout << "TableSize: " << tablesize << "\n";

	if(tablesize==0)
		return gtable;

	istr.getline(line, linesize);
	sscanf (line,"%s %i",chunk,&handdof);
	istr.getline(line, linesize);
	sscanf (line,"%s %i",chunk,&nrquality);
    std::cout << "HandDOF: " << handdof << "\n";
    std::cout << "NrOfQualityMeasures: " << nrquality << "\n";


    char tmpC;

    for(size_t i=0;i<tablesize;i++){
    	GraspData data;
    	float a[3];
    	float pq[handdof];
    	float qual[nrquality];
    	float hp[6], op[6];
    	istr >> a[0] >> tmpC >> a[1] >> tmpC >> a[2] >> tmpC;
    	data.approach = Vector3D<>(a[0],a[1],a[2]);
    	//std::cout << "1";
    	data.quality = Q(nrquality);
    	for(size_t j=0;j<nrquality;j++){
    		istr >> qual[j]>> tmpC;
    		data.quality[j] = qual[j];
    	}
    	//std::cout << "2";
    	data.pq = Q(handdof);
    	for(size_t j=0;j<handdof;j++){
    		istr >> pq[j]>> tmpC;
    		data.pq[j] = pq[j];
    	}
    	//std::cout << "3";
    	istr >> hp[0] >> tmpC >> hp[1] >> tmpC >> hp[2] >> tmpC
			 >> hp[3] >> tmpC >> hp[4] >> tmpC >> hp[5] >> tmpC;
    	data.hp = Pose6D<>(hp[0],hp[1],hp[2],hp[3],hp[4],hp[5]);

    	//std::cout << "4";
    	istr >> op[0] >> tmpC >> op[1] >> tmpC >> op[2] >> tmpC
			 >> op[3] >> tmpC >> op[4] >> tmpC >> op[5] >> tmpC;
    	data.op = Pose6D<>(op[0],op[1],op[2],op[3],op[4],op[5]);
    	//std::cout << "5";
    	size_t cqsize;
    	istr >> cqsize >> tmpC;
    	//std::cout << "6";
    	data.cq = Q(cqsize);
    	float cq[cqsize];
    	for(size_t j=0;j<cqsize;j++){
    		istr >> cq[j] >> tmpC;
    		data.cq[j] = cq[j];
    	}
    	//std::cout << "7";
    	size_t consize;
    	istr >> consize >> tmpC;
    	//std::cout << "8";
    	data.grasp = Grasp3D(consize);
    	float cs[3];
    	for(size_t j=0;j<consize;j++){
    		for(size_t m=0;m<3;m++){
    			istr >> cs[m] >> tmpC;
    			data.grasp.contacts[j].p[m] = cs[m];
    		}
    		for(size_t m=0;m<3;m++){
    			istr >> cs[m] >> tmpC;
    			data.grasp.contacts[j].n[m] = cs[m];
    		}
    	}
    	//std::cout << "9" << std::endl;
    	// read tactile data
        // and now the tactile data
    	size_t nrsensors;
    	istr >> nrsensors >> tmpC;
    	//std::cout << "Nr sensors: " << nrsensors << std::endl;
    	data._tactiledata.resize(nrsensors);
        for(size_t j=0;j<nrsensors;j++){
        	// write dimensions of pad
        	size_t xdim, ydim;
        	istr >> xdim >> tmpC;
        	istr >> ydim >> tmpC;
        	rw::sensor::TactileArray::ValueMatrix mat(xdim,ydim);
        	for(size_t x=0;x<xdim;x++)
        		for(size_t y=0;y<ydim;y++){
        			float val;
        			istr >> val >> tmpC;
        			mat(x,y) = val;
        		}
        	//std::cout << xdim << ";" << ydim << ";"<<
        	data._tactiledata[j] = mat;
        }

    	gtable->addGrasp(data);

    }

	return gtable;
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

        // and now the tactile data
        fstr << data._tactiledata.size() << ";";

        for(size_t j=0;j<data._tactiledata.size();j++){
        	// write dimensions of pad
        	fstr << data._tactiledata[j].size1() << ";";
        	fstr << data._tactiledata[j].size2() << ";";
        	for(size_t x=0;x<data._tactiledata[j].size1();x++)
        		for(size_t y=0;y<data._tactiledata[j].size2();y++)
        			fstr << data._tactiledata[j](x,y) << ";";

        }

        // we end the data entry with a endline
        fstr << std::endl;
        std::cout << std::endl;
    }

    fstr.close();
}


