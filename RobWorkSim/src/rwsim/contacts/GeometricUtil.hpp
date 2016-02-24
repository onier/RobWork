/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_CONTACTS_GEOMETRICUTIL_HPP_
#define RWSIM_CONTACTS_GEOMETRICUTIL_HPP_

/**
 * @file GeometricUtil.hpp
 *
 * \copydoc rwsim::contacts::GeometricUtil
 */

#include <rw/math/Vector3D.hpp>

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Utility functions for generic geometric operations.
 */
class GeometricUtil {
public:
	typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > PointPair;

	/**
	 * @brief Get a vector perpendicular to the given vector.
	 * @param normal [in] the vector to find perpendicular vector for.
	 * @return perpendicular vector.
	 */
	static rw::math::Vector3D<> getPerpendicular(const rw::math::Vector3D<> &normal);

	/**
	 * @brief Project a point on a plane.
	 * @param point [in] the point to project.
	 * @param planePoint [in] some point on the plane.
	 * @param planeNormal [in] the planes normal vector.
	 * @return the projected point.
	 */
	static rw::math::Vector3D<> projectPointOnPlane(const rw::math::Vector3D<> &point, const rw::math::Vector3D<> &planePoint, const rw::math::Vector3D<> &planeNormal);

	/**
	 * @brief Project a line on a plane.
	 * @param point [in] point on the line.
	 * @param dir [in] the direction of the line.
	 * @param planePoint [in] some point on the plane.
	 * @param planeNormal [in] the normal vector of the plane.
	 * @return a pair of vectors, where the first is a point and the second a direction of the line.
	 */
	static PointPair projectLineOnPlane(const rw::math::Vector3D<> &point, const rw::math::Vector3D<> &dir, const rw::math::Vector3D<> &planePoint, const rw::math::Vector3D<> &planeNormal);

	/**
	 * @brief Project a point on a line.
	 * @param point [in] the point to project.
	 * @param linePoint [in] some point on the line.
	 * @param lineDir [in] the directional vector of the line.
	 * @return the projected point on the line.
	 */
	static rw::math::Vector3D<> projectPointOnLine(const rw::math::Vector3D<> &point, const rw::math::Vector3D<> &linePoint, const rw::math::Vector3D<> &lineDir);

	/**
	 * @brief Get the distance between a point and a line.
	 * @param point [in] the point.
	 * @param linePoint [in] a point on the line.
	 * @param lineDir [in] the direction of the line.
	 * @return the distance.
	 */
	static double pointLineDistance(const rw::math::Vector3D<> &point, const rw::math::Vector3D<> &linePoint, const rw::math::Vector3D<> &lineDir);

	/**
	 * @brief Find the intersection between a line and a plane.
	 * @param planePoint [in] a point on the plane.
	 * @param planeNormal [in] the normal of the plane.
	 * @param linePoint [in] a point on the line.
	 * @param lineDir [in] the direction of the line.
	 * @return the intersection point.
	 */
	static rw::math::Vector3D<> linePlaneIntersection(const rw::math::Vector3D<> &planePoint, const rw::math::Vector3D<> &planeNormal, const rw::math::Vector3D<> &linePoint, const rw::math::Vector3D<> &lineDir);

	/**
	 * @brief Find the intersection line between two planes.
	 * @param planeAPoint [in] a point in one plane.
	 * @param planeANormal [in] the normal of the first plane.
	 * @param planeBPoint [in] a point in another plane.
	 * @param planeBNormal [in] the normal of the second plane.
	 * @return a pair of vectors, where the first is a point and the second a direction of the line where the planes intersect.
	 */
	static std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > planePlaneIntersection(const rw::math::Vector3D<> &planeAPoint, const rw::math::Vector3D<> &planeANormal, const rw::math::Vector3D<> &planeBPoint, const rw::math::Vector3D<> &planeBNormal);

	/**
	 * @brief Find the point on a circle that lies closest to some plane.
	 * @param center [in] the center point of the circle.
	 * @param radius [in] radius of the circle.
	 * @param circleNormal [in] the normal of the circle (normal to the plane in which the circle lies).
	 * @param planeNormal [in] the normal of the plane to find closest point to.
	 * @return the closest point on the circle.
	 */
	static rw::math::Vector3D<> closestPointOnCircleToPlane(const rw::math::Vector3D<> &center, double radius, const rw::math::Vector3D<> &circleNormal, const rw::math::Vector3D<> &planeNormal);

	/**
	 * @brief Find the point on a circle that lies closest to some point.
	 * @param center [in] the center point of the circle.
	 * @param radius [in] radius of the circle.
	 * @param circleNormal [in] the normal of the circle (normal to the plane in which the circle lies).
	 * @param point [in] the point.
	 * @return the point on the circle that is closest to the given point.
	 */
	static rw::math::Vector3D<> closestPointOnCircleToPoint(const rw::math::Vector3D<> &center, double radius, const rw::math::Vector3D<> &circleNormal, const rw::math::Vector3D<> point);

	/**
	 * @brief Find the points on a circle that lies closest to some line.
	 *
	 * Note that there can be one or two local minimas.
	 * If two solutions are returned, the first will be the global minimum.
	 *
	 * @param center [in] the center point of the circle.
	 * @param radius [in] radius of the circle.
	 * @param circleNormal [in] the normal of the circle (normal to the plane in which the circle lies).
	 * @param linePoint [in] a point on the line.
	 * @param lineDir [in] the direction of the line.
	 * @return a list of one or two pairs of points, where the first is the point on the circle and the second is the point on the line.
	 */
	static std::vector<PointPair> closestPointsOnCircleToLine(const rw::math::Vector3D<> &center, double radius, const rw::math::Vector3D<> &circleNormal, const rw::math::Vector3D<> linePoint, const rw::math::Vector3D<> &lineDir);

	/**
	 * @brief Find the closest points on an ellipse to some circle.
	 * @param centerCircle [in] the center of the circle.
	 * @param normal [in] the normal of the circle.
	 * @param centerEllipse [in] the center of the ellipse.
	 * @param dir1 [in] the first principal direction of the ellipse.
	 * @param r1 [in] the radius in the first principal direction.
	 * @param dir2 [in] the second principal direction of the ellipse.
	 * @param r2 [in] the radius in the second principal direction.
	 * @return a list of the closest points.
	 */
	static std::vector<rw::math::Vector3D<> > closestPointsOnEllipseToCircle(const rw::math::Vector3D<> &centerCircle, const rw::math::Vector3D<> &normal, const rw::math::Vector3D<> &centerEllipse, const rw::math::Vector3D<> &dir1, double r1, const rw::math::Vector3D<> &dir2, double r2);
	static std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > closestPointOnCircleToCircle(const rw::math::Vector3D<> &cCircle1, const rw::math::Vector3D<> &n1, double r1, const rw::math::Vector3D<> &cCircle2, const rw::math::Vector3D<> &n2, double r2);
	static std::vector<std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > > closestPointsOnCircleToCircle(const rw::math::Vector3D<> &cCircle1, const rw::math::Vector3D<> &n1, double r1, const rw::math::Vector3D<> &cCircle2, const rw::math::Vector3D<> &n2, double r2);

	static PointPair extremalPointsOnEllipseToPoint(const rw::math::Vector3D<> &point, const rw::math::Vector3D<> &centerEllipse, const rw::math::Vector3D<> &dir1, double r1, const rw::math::Vector3D<> &dir2, double r2);

	/**
	 * @brief Find the intersection points between a circle and ellipse.
	 * @param normal [in] normal to the plane where both circle and ellipse lies.
	 * @param centerCircle [in] center point of the circle.
	 * @param rc [in] radius of the circle.
	 * @param centerEllipse [in] center point of the ellipse.
	 * @param dir1 [in] the first principal direction of the ellipse.
	 * @param r1 [in] the radius in the first principal direction.
	 * @param dir2 [in] the second principal direction of the ellipse.
	 * @param r2 [in] the radius in the second principal direction.
	 * @return a list of points with length 0 to 4.
	 */
	static std::vector<rw::math::Vector3D<> > intersectCircleEllipse(const rw::math::Vector3D<> &normal, const rw::math::Vector3D<> &centerCircle, double rc, const rw::math::Vector3D<> &centerEllipse, const rw::math::Vector3D<> &dir1, double r1, const rw::math::Vector3D<> &dir2, double r2);

private:
	struct FuncCircleCircleDist;
	struct FuncdTrig;
	struct FuncdPol;
	struct Bracketmethod;
	struct Dbrent;

	GeometricUtil();
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_GEOMETRICUTIL_HPP_ */
