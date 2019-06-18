#ifndef SIMPLEGLVIEWER_H_
#define SIMPLEGLVIEWER_H_

#include <rw/graphics/SceneViewer.hpp>

namespace rw { namespace models { class WorkCell; } }
namespace rwlibs { namespace opengl { class SceneOpenGL; } }

class EventListener;
class Menu;

#include <map>

/**
 * @brief simple viewer for rendering robwork workcell
 */
class SimpleGLViewer: public rw::graphics::SceneViewer {
    public:
    	//! @brief smart pointer type to this class
    	typedef rw::common::Ptr<SimpleGLViewer> Ptr;

        //! @brief Constructor.
		SimpleGLViewer();

    	//! @brief Destructor.
        virtual ~SimpleGLViewer();

        //! @copydoc rw::graphics::SceneViewer::getScene
        virtual rw::graphics::SceneGraph::Ptr getScene();

        //! @copydoc rw::graphics::SceneViewer::getLogo
        virtual const std::string& getLogo() const;

        //! @copydoc rw::graphics::SceneViewer::setLogo
        virtual void setLogo(const std::string& string);

        //! @copydoc rw::graphics::SceneViewer::getPropertyMap
        virtual rw::common::PropertyMap& getPropertyMap();

        //! @copydoc rw::graphics::SceneViewer::updateView
        virtual void updateView();

        //! @copydoc rw::graphics::SceneViewer::updateState
        virtual void updateState(const rw::kinematics::State& state);

        //! @copydoc rw::graphics::SceneViewer::setWorldNode
        virtual void setWorldNode(rw::graphics::GroupNode::Ptr wnode);

        //! @copydoc rw::graphics::SceneViewer::getWorldNode
        virtual rw::graphics::GroupNode::Ptr getWorldNode();

        //! @copydoc rw::graphics::SceneViewer::saveBufferToFile(const std::string&, const int, const int, const int)
        // changed stdfilename to filename due to doxygen error
        virtual void saveBufferToFile(const std::string& filename,
                                      const int fillR, const int fillG, const int fillB);

        //! @copydoc rw::graphics::SceneViewer::getViewCamera
        virtual rw::graphics::SceneCamera::Ptr getViewCamera();

        //! @copydoc rw::graphics::SceneViewer::getViewCenter
        virtual rw::math::Vector3D<> getViewCenter();

        //! @copydoc rw::graphics::SceneViewer::pickDrawable(SceneGraph::RenderInfo&, int, int)
        virtual rw::graphics::DrawableNode::Ptr pickDrawable(int x, int y);

        //! @copydoc rw::graphics::SceneViewer::pickDrawable(SceneGraph::RenderInfo&, int, int)
        virtual rw::graphics::DrawableNode::Ptr pickDrawable(rw::graphics::SceneGraph::RenderInfo& info, int x, int y);

        //! @copydoc rw::graphics::SceneViewer::createView
        virtual View::Ptr createView(const std::string& name, bool enableBackground=false);

        //! @copydoc rw::graphics::SceneViewer::getMainView
        virtual View::Ptr getMainView();

        //! @copydoc rw::graphics::SceneViewer::destroyView
        virtual void destroyView(View::Ptr view);

        //! @copydoc rw::graphics::SceneViewer::selectView
        virtual void selectView(View::Ptr view);

        //! @copydoc rw::graphics::SceneViewer::getCurrentView
        virtual View::Ptr getCurrentView();

        //! @copydoc rw::graphics::SceneViewer::getViews
        virtual std::vector<View::Ptr> getViews();

        //! @copydoc rw::graphics::SceneViewer::renderView
        virtual void renderView(View::Ptr view);

        //! @copydoc rw::graphics::SceneViewer::zoom
        virtual void zoom(double amount);

        //! @copydoc rw::graphics::SceneViewer::autoZoom
        virtual void autoZoom();

        void init(int argc, char** argv);

        void addMenu(Menu *menu);

        void setKeyListener(EventListener *listener);

        void setWorkcell(rw::common::Ptr<rw::models::WorkCell> workcell);

        const rw::kinematics::State& getState();

        void resize(int width, int height);

        void setPosition(int x, int y);

        bool start();

        bool stop();

    private:
        void addSubMenus(Menu *menu);
        void initGlut(int,int,int,int);
        void initLights();
        void initMenu();

    public:
        struct InternalData;
        InternalData* const _data;

    private:
        rw::common::Ptr<rwlibs::opengl::SceneOpenGL> _scene;
        rw::graphics::GroupNode::Ptr _worldNode;
        rw::common::Ptr<rw::kinematics::State> _state;

        rw::common::Ptr<rw::models::WorkCell> _wc;

        std::map<int,Menu*> _menuMap;
};


#endif /*SIMPLEGLVIEWER_H_*/
