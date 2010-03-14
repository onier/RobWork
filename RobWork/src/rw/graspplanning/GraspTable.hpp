/*
 * GraspTable.hpp
 *
 *  Created on: 22-09-2009
 *      Author: jimali
 */

#ifndef GRASPTABLE_HPP_
#define GRASPTABLE_HPP_

#include <rw/math.hpp>
#include <rw/common.hpp>
#include "Grasp3D.hpp"
#include <rw/sensor/TactileArray.hpp>


namespace rw {
namespace graspplanning {

/**
 * @brief A table of grasp configurations that has been generated using a robot hand,
 * a number of preshapes, and some grasp policy.
 *
 *
 *
 *
 */
class GraspTable {
public:
    struct GraspData {
    	GraspData(): hp(0,0,0,0,0,0), op(0,0,0,0,0,0)
    	{}
        rw::math::Vector3D<> approach; // approach relative to object
        rw::math::Q cq; // contact configuration
        rw::math::Q pq; // preshape configuration
        rw::math::Pose6D<> hp; // hand pose
        rw::math::Pose6D<> op; // object pose
        Grasp3D grasp;
        rw::math::Q quality;
        std::vector<rw::sensor::TactileArray::ValueMatrix> _tactiledata;
    };

public:
    GraspTable(const std::string& handName, const std::string& objectId);

    virtual ~GraspTable(){};

    void addGrasp(GraspData& data);

    size_t size(){ return _graspData.size();};

    static GraspTable* load(const std::string& filename);

    void save(const std::string& filename);

    std::vector<GraspData>& getData(){return _graspData;};

    /**
     *
     * @return
     */
    //std::vector<GraspData> findGrasps(GraspValidateFilter* filter);
    //std::vector<GraspData> findGrasps(rw::math::Q& minQ, GraspValidateFilter* filter = NULL);
    //std::vector<GraspData> findGrasps(const rw::math::Vector3D<>& approach, double maxAngle,GraspValidateFilter* filter = NULL);
    //std::vector<GraspData> findGrasps(rw::math::Q& minQ, const rw::math::Vector3D<>& approach, double maxAngle,GraspValidateFilter* filter = NULL);

private:
    rw::common::PropertyMap _properties;

    std::vector<GraspData> _graspData;

    std::string _handName;
    std::string _objectId;
};

}
}

#endif /* GRASPTABLE_HPP_ */
