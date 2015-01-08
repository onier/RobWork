#include "SamplePlugin.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;

using namespace rws;



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

}

SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
}

void SamplePlugin::open(WorkCell* workcell)
{


}

void SamplePlugin::close() {
}

void SamplePlugin::btnPressed() {

	static double cnt = 0;
	using namespace rw::models;
	RW_WARN("Butten pressed!!");


    QObject *obj = sender();
    if(obj==_btn0){

        log().info() << "Button 0\n";
        State state = getState();
        double scale = 3.14/_dobj->getNrNodes(state);
        for(int i=0;i<_dobj->getNrNodes(state);i++){
        	_dobj->getNode(i,state)[2] = std::sin( i*scale+cnt ) * 0.1;
        }
        getRobWorkStudio()->setState(state);
        cnt +=0.1;
    } else if(obj==_btn1){
        std::cout << "Adding deformable object" << std::endl;

        rw::geometry::Geometry::Ptr geom = rw::geometry::Geometry::makeGrid(10,10,1,1);
        Frame* wframe = getRobWorkStudio()->getWorkCell()->getWorldFrame();
        // create deformable object
        _dobj = ownedPtr( new DeformableObject(wframe, 100 ) );
        getRobWorkStudio()->getWorkCell()->add(_dobj);
        State state = getRobWorkStudio()->getWorkCell()->getDefaultState();
        // make 10x10 grid
        for(int y=0;y<10;y++)
        	for(int x=0;x<10;x++){
        		_dobj->setNode(y*10+x, Vector3D<float>(x*0.1,y*0.1, 0), state );
        	}
        // add faces
        for(int y=0;y<9;y++)
        	for(int x=0;x<9;x++){
        		_dobj->addFace(y*10+(x+1), y*10+x, (y+1)*10+x);
        		_dobj->addFace((y+1)*10+x, (y+1)*10+(x+1), y*10+x+1);
        	}

        getRobWorkStudio()->setState(state);
        getRobWorkStudio()->getWorkCellScene()->updateSceneGraph(state);
    } else if(obj==_spinBox){
        log().info() << "spin value:" << _spinBox->value() << "\n";
    }


}

void SamplePlugin::stateChangedListener(const State& state) {

}

#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1")
#else
Q_EXPORT_PLUGIN(SamplePlugin);
#endif
