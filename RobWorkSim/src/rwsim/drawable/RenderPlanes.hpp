#ifndef RWSIM_DRAWABLE_RENDERPLANES_HPP_
#define RWSIM_DRAWABLE_RENDERPLANES_HPP_

//! @file RenderPlanes.hpp

#include <list>
#include <vector>

#include <rw/kinematics/State.hpp>
#include <rw/math/Vector3D.hpp>

#include <rwlibs/drawable/Render.hpp>

#include <rwsim/util/PlaneModel.hpp>

namespace rwsim {
namespace drawable {
	//! @addtogroup drawable @{

	/**
	 * @brief renderer for rendering planes
	 */
	class RenderPlanes: public rwlibs::drawable::Render
	{
	public:
		/**
		 * @brief constructor
		 */
		RenderPlanes(float planesize=1.0);

		/**
		 * @brief destructor
		 */
		virtual ~RenderPlanes();

		/**
		 * @brief adds planes to the renerer
		 */
		void addPlanes(const std::vector<rwsim::util::PlaneModel >& planes);

		/**
		 * @brief clear the list of planes
		 */
		void clear();

		/**
		 * @brief set the color used for the model
		 * @param r [in] red color value
		 * @param g [in] green color value
		 * @param b [in] blue color value
		 */
		void setColor(double r, double g, double b);

		//! Render::draw
		virtual void draw(DrawType type, double alpha) const;
	private:
		std::vector<util::PlaneModel> _planes;
		float _planesize;
		float _color[3];
	};
	//! @}
}
}

#endif /*RenderGhost_HPP_*/
