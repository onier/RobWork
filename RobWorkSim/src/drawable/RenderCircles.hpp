#ifndef RENDERCIRCLES_HPP_
#define RENDERCIRCLES_HPP_

#include <list>
#include <vector>

#include <util/CircleModel.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/drawable/Render.hpp>
#include <rw/math/Vector3D.hpp>

/**
 * @brief Render a set of circles
 */
class RenderCircles: public rwlibs::drawable::Render
{
public:
	/**
	 * @brief constructor
	 * @param angleResolution [in] the resolution of the circle line segments
	 * in degree. The circle is approximated using line segments.
	 */
	RenderCircles(float angleResolution=10);

	/**
	 * @brief destructor
	 */
	virtual ~RenderCircles();

	/**
	 * @brief adds circle to the circles that are allready drawn
	 * @param circle [in] circle to draw
	 */
	void addCircle(const CircleModel& circle);

	/**
	 * @brief adds circles to the circles that are allready drawn
	 */
	void addCircles(const std::vector<CircleModel>& circles);

	void setCircles(const std::vector<CircleModel>& circles);

	void setColor(double r, double g, double b);

	void clear();

	virtual void draw(DrawType type, double alpha) const;
private:
	float _stepSize;
	std::vector<CircleModel> _circles;
	float _color[3];
};


#endif /*RenderGhost_HPP_*/
