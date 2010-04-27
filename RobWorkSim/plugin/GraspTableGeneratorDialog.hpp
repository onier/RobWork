/*
 * GraspTableGeneratorDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef GraspTableGeneratorDialog_HPP_
#define GraspTableGeneratorDialog_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include "ui_GraspTableGeneratorPlugin.h"

#include <rw/kinematics/State.hpp>

#include <dynamics/RigidBody.hpp>
#include <dynamics/DynamicWorkcell.hpp>
#include <simulator/ThreadSimulator.hpp>
#include <control/PDController.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <rw/graspplanning/GraspTable.hpp>
#include <util/MovingAverage.hpp>

#include <boost/numeric/ublas/matrix.hpp>

#include <rw/proximity/CollisionDetector.hpp>
#include <dynamics/RigidDevice.hpp>

#include <sensors/BodyContactSensor.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include <util/RestingPoseGenerator.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>

#include "ThreadSafeStack.hpp"
        struct RestingConfig {
            RestingConfig(const rw::kinematics::State& state, const std::string& str):
                _state(state),_desc(str){}
            RestingConfig(){};
            rw::kinematics::State _state;
            std::string _desc;
        };
/**
 * @brief a grphical interface for calculating resting configurations of
 * rigid bodies using rigid body physics simulation.
 *
 *
 */
class GraspTableGeneratorDialog : public rws::RobWorkStudioPlugin, private Ui::GraspTableGeneratorPlugin
    {
        Q_OBJECT
		Q_INTERFACES( rws::RobWorkStudioPlugin )

    public:
        typedef std::vector<boost::numeric::ublas::matrix<float> > TactileSensorData;

        GraspTableGeneratorDialog();

        /**
         * @copydoc RobWorkStudioPlugin::open
         */
        void open(rw::models::WorkCell* workcell);

        /**
         * @copydoc RobWorkStudioPlugin::close
         */
        void close();

        /**
         * @copydoc RobWorkStudioPlugin::initialize
         */
        void initialize();

        /**
         * @brief for state changes of RWS
         */
        void stateChangedListener(const rw::kinematics::State& state);

        /**
         * @brief we listen for events regarding opening and closing of dynamic
         * workcell
         */
        void genericEventListener(const std::string& event);


        const rw::kinematics::State& getState(){ return _state; };

        std::vector<dynamics::RigidBody*>& getBodies(){ return _bodies; };

        std::vector<rw::kinematics::State>& getStartPoses(){return _startPoses;};

        std::vector<rw::kinematics::State>& getRestingPoses(){return _resultPoses;};

        void setSaveDir(const std::string& str){
            _savePath->setText(str.c_str());
        }

        void startAuto();

        void stepCallBack(int i, const rw::kinematics::State& state);


    signals:
        /**
         * @brief The state of one simulation thread can be actively monitored
         * through this signal.
         * @param state
         */
        void stateChanged(const rw::kinematics::State& state);

        /**
         * @brief An event that is fired when a resting pose has been calculated.
         */
        void restingPoseEvent(const RestingConfig& restcfg);

    private slots:
        void btnPressed();
        void changedEvent();

    private:
    	void initializeStart();
        void updateStatus();
        void cleanup();

        /**
         * @brief calculates a random configuration of
         * all bodies
         * @param state
         */
        void calcRandomCfg(rw::kinematics::State& state);

        /**
         * @brief Calculate random configuration for \b bodies
         * @param bodies
         * @param state
         */
        void calcRandomCfg(std::vector<dynamics::RigidBody*> &bodies,
                           rw::kinematics::State& state);

        /**
         * @brief calculates a collision free random configuration of
         * all bodies
         * @param state
         */
        void calcColFreeRandomCfg(rw::kinematics::State& state);

        bool isSimulationFinished( SimulatorPtr sim, const rw::kinematics::State& state );

        bool saveRestingState( int simidx, SimulatorPtr sim , const rw::kinematics::State& state );


    private:
        rw::common::PropertyMap _settings;

        struct CallBackFunctor {
            CallBackFunctor(int i,GraspTableGeneratorDialog *parent):_i(i),_parent(parent){}

            void stepCallBack(const rw::kinematics::State& state){
                _parent->stepCallBack(_i, state);
            }

            int _i;
            GraspTableGeneratorDialog *_parent;

        };

        std::vector<RestingPoseGenerator*> _generators;

        Ui::GraspTableGeneratorPlugin _ui;
        rw::kinematics::State _defstate;
        rw::kinematics::State _state;
        QTimer *_timer;
        std::vector<rw::common::Ptr<ThreadSimulator> > _simulators;
        std::vector<rw::kinematics::State> _initStates;
        std::vector<double> _simStartTimes;
        int _nrOfTests;
        double _totalSimTime;
        std::vector<dynamics::RigidBody*> _bodies;

        long _startTime;

        std::vector<rw::kinematics::State> _startPoses;
        std::vector<rw::kinematics::State> _resultPoses;

        rw::kinematics::FrameMap<dynamics::RigidBody*> _frameToBody;
        rw::common::Ptr<dynamics::DynamicWorkcell> _dwc;

        rw::proximity::CollisionDetector *_colDect;
        double _lastTime,_lastBelowThresUpdate;
        MovingAverage _avgSimTime;
        MovingAverage _avgTime;

        std::vector<PDControllerPtr> _controllers;
        std::vector<rw::math::Q> _preshapes;
        std::vector<rw::math::Q> _targetQ;
        dynamics::RigidBody *_body;
        RigidDevice *_hand;
        rw::kinematics::MovableFrame *_handBase,*_object;

        BodyContactSensorPtr _bodySensor;

        bool _exitHard;

        bool _graspNotStable;

        std::vector<bool> _fingersInContact;

        std::vector<std::vector< rw::math::Q > > _handconfigs;
        std::vector<std::vector< TactileSensorData > > _tactiledatas;

        std::vector<rw::common::Ptr<CallBackFunctor> > _functors;
        std::vector<double> _nextTimeUpdate;
        int _nrOfTestsOld;

        ThreadSafeStack<RestingConfig> _restingConfigs;

        //QSampler *_handQSampler;
        std::vector<int> _currentPreshapeIDX;
        rw::math::Q _target,_preshape;
        rw::math::Transform3D<> _objTransform;

        rw::graspplanning::GraspTable *_gtable;
        int _nrOfGraspsInGroup, _lastTableBackupCnt;
        int _tactileDataOnAllCnt;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
