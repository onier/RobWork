/*
 * CircleModel.hpp
 *
 *  Created on: 10-03-2009
 *      Author: jimali
 */

#ifndef CIRCLEMODEL_HPP_
#define CIRCLEMODEL_HPP_

#include <rw/math/Vector3D.hpp>

#include <vector>

/**
 * @brief a model of a circle, represented as a normal, a center and a radius
 */
class CircleModel {
public:

	/**
	 * @brief constructor
	 */
	CircleModel(){};

	/**
	 * @brief contructor
	 * @param c [in] center
	 * @param n [in] normal
	 * @param radi [in] radius
	 * @return
	 */
	CircleModel(const rw::math::Vector3D<>& c, const rw::math::Vector3D<>& n, double radi);

	/**
	 * @brief constructor from 3 unique points on circle
	 * @param p1 [in] point on circle
	 * @param p2 [in] point on circle
	 * @param p3 [in] point on circle
	 */
	CircleModel(rw::math::Vector3D<>& p1, rw::math::Vector3D<>& p2, rw::math::Vector3D<>& p3);

	/**
	 * @brief refit the circle from a number of points. Use a least square fit.
	 * @param points [in] the vector of points that the circle is fitted to
	 */
	void refit(const std::vector<rw::math::Vector3D<> >& points);

	/**
	 * @brief tests if two circles are "equal" which is within some epsilon of
	 * each other
	 * @param circ [in] the other circle
	 * @param epsilon [in] how close they need to be.
	 * @return
	 */
	bool isClose(const CircleModel& circ, double epsilon) const;

	/**
	 * @brief tests if a point \p is within a distance \b epsilon of
	 * this circle.
	 * @param p [in] point in 3D
	 * @param epsilon [in] max distance that \b p can be from this circle.
	 * @return true if p is within \b epsilon of this circle
	 */
	bool isClose(const rw::math::Vector3D<>& p, double epsilon) const;

    /**
     * @brief Ouputs CircleModel to stream
     * @param os [in/out] stream to use
     * @param model [in] a CircleModel
     * @return the resulting stream
     */
    friend std::ostream& operator<<(std::ostream& os, const CircleModel& model){
        return os << "Circle ["<< model._center << ","<< model._n << "," << model._r << "]";
    }

	/**
	 * @brief create a circle from a number of points. Use a least square fit.
	 * @param points [in] the vector of points that the circle is fitted to
	 */
	static CircleModel fitTo(const std::vector<rw::math::Vector3D<> >& points);

public:
	rw::math::Vector3D<> _center,_n;
	double _r;
};


#endif /* CIRCLEMODEL_HPP_ */
