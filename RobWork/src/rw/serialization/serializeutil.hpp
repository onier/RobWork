#ifndef SERIALIZEUTIL_HPP
#define SERIALIZEUTIL_HPP

#include <rw/math.hpp>
#include "InputArchive.hpp"
#include "OutputArchive.hpp"

namespace serialization {

    void write(const rw::math::Q& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Q& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Vector2D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Vector2D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Vector3D<>& tmp, OutputArchive& oar, const std::string& id);
    void read(rw::math::Vector3D<>& tmp, InputArchive& iar, const std::string& id);

    void write(const rw::math::Rotation2D<>& tmp, OutputArchive& oar, const std::string& id);
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

#endif
