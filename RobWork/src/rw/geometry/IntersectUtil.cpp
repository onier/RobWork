/*
 * IntersectUtil.cpp
 *
 *  Created on: 24/04/2010
 *      Author: jimali
 */
#include "IntersectUtil.hpp"

using namespace rw::geometry;
using namespace rw::math;

rw::math::Vector3D<> IntersectUtil::closestPt(const Vector3D<>& point, const Line& line)
{
	return closestPtPointLine(point, line.p1(), line.p2());
}


rw::math::Vector3D<> IntersectUtil::closestPtPointRay(const Vector3D<>& point, const Vector3D<>& ray)
{
	
}

IntersectUtil::PointPair IntersectUtil::closestPts(const Line& lineA, const Line& lineB){

}

IntersectUtil::PointPair IntersectUtil::closestPts(const Line& line, const Triangle<>& tri){

}

rw::math::Vector3D<> IntersectUtil::closestPtPointLine(const rw::math::Vector3D<>& point,
										    const rw::math::Vector3D<>& p1,
											const rw::math::Vector3D<>& p2)
{
	Vector3D<> ab = p2-p1;
	// project c onto ab, computing parameterizated pos
	double t = dot(point - p1,ab)/dot(ab,ab);
	// if outside segment, clamp t
	t = Math::clamp(t,0,1);
	// compute projected position
	return p1 + t*ab;
}


IntersectUtil::PointPair IntersectUtil::closestPtsRayTraingle(
		const rw::math::Vector3D<>& ray,
		const Triangle<>& tri)
{
	// for each segment in the triangle we do a closest point calculation

}

bool IntersectUtil::intersetPtRayPlane(
		const rw::math::Vector3D<> &p1,
		const rw::math::Vector3D<> &p2,
		const Plane& p, Vector3D<> &dst)
{
	Vector3D<> ab = p2-p1;
	// project c onto ab, computing parameterizated pos
	double t = (p.d() - dot(p.normal() - p1, p1)) / dot(p.normal(),ab);

	// if abs(t) is very large then the ray is parallel to plane
	if(10000000<fabs(t))
		return false;
	// compute projected position
	dst = p1 + t*ab;
	return true;
}

bool IntersectUtil::intersetPtRayPlane(
						const rw::math::Vector3D<>& rayp1,
						const rw::math::Vector3D<>& rayp2,
						const rw::math::Vector3D<>& p1,
						const rw::math::Vector3D<>& p2,
						const rw::math::Vector3D<>& p3,
						rw::math::Vector3D<>& dst)
{
	Plane p(p1, p2, p3);
	return IntersectUtil::intersetPtRayPlane(rayp1, rayp2, p, dst);
}
