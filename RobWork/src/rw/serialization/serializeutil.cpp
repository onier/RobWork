#include "serializeutil.hpp"

#include <rw/math/Q.hpp>
#include "InputArchive.hpp"
#include "OutputArchive.hpp"

namespace {

	template <class ARR>
	std::vector<double> toStdVector(const ARR& tmp, int size){
		std::vector<double> qvec(size);
        for(int i=0;i<size;i++){
        	qvec[i] = tmp[i];
        }
        return qvec;
	}

	template <class ARR>
	ARR fromStdVector(std::vector<double>& data, ARR& tmp){
        for(size_t i=0;i<data.size();i++){
        	tmp[i] = data[i];
        }
        return tmp;
	}

}

namespace serialization {

    void write(const rw::math::Q& tmp, OutputArchive& oar, const std::string& id){
    	oar.write( toStdVector(tmp, tmp.size()), id );
    }
    void write(const rw::math::Vector2D<>& tmp, OutputArchive& oar, const std::string& id){
    	oar.write( toStdVector(tmp, 2), id );
    }
    void write(const rw::math::Vector3D<>& tmp, OutputArchive& oar, const std::string& id){
    	oar.write( toStdVector(tmp, 3), id );
    }
    void write(const rw::math::Rotation2D<>& tmp, OutputArchive& oar, const std::string& id){
    	std::vector<double> data;
    	data.push_back(tmp(0,0));
    	data.push_back(tmp(0,1));
    	data.push_back(tmp(1,0));
    	data.push_back(tmp(1,1));
    	oar.write(data,id);
    }

    void read(rw::math::Q& tmp, InputArchive& iar, const std::string& id){
    	std::vector<double> arr;
    	iar.read(arr, id);
    	tmp = rw::math::Q(arr.size());
    	fromStdVector(arr, tmp);
    }
    void read(rw::math::Vector2D<>& tmp, InputArchive& iar, const std::string& id){
    	std::vector<double> arr;
    	iar.read(arr, id);
    	fromStdVector(arr, tmp);
    }
    void read(rw::math::Vector3D<>& tmp, InputArchive& iar, const std::string& id){
    	std::vector<double> arr;
    	iar.read(arr, id);
    	fromStdVector(arr, tmp);
    }

    void read(rw::math::Rotation2D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Rotation3D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Rotation3D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Transform2D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Transform2D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Transform3D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Transform3D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Pose2D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Pose2D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Pose3D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Pose3D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::VelocityScrew6D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::VelocityScrew6D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Quaternion<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Quaternion<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::EAA<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::EAA<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::RPY<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::RPY<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Jacobian& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Jacobian& tmp, InputArchive& iar, const std::string& id);




}
