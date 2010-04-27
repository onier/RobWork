/*
 * TactileSensorDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef GRASPRESTINGPOSEDIALOG_HPP_
#define GRASPRESTINGPOSEDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include "ui_TactileSensorDialog.h"

#include <rw/kinematics/State.hpp>

#include <dynamics/RigidBody.hpp>
#include <dynamics/DynamicWorkcell.hpp>
#include <simulator/ThreadSimulator.hpp>
#include <sensors/TactileArraySensor.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/math.hpp>
#include <util/MovingAverage.hpp>

#include <rw/proximity/CollisionDetector.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>

        struct Moment {
        	rw::math::Vector2D<> center;
        	rw::math::Vector2D<> first,second;
        };


/**
 * @brief a grphical interface for calculating resting configurations of
 * rigid bodies using rigid body physics simulation.
 *
 *
 */
class TactileSensorDialog : public QDialog, public Ui::TactileSensorDialog
    {
        Q_OBJECT

    public:
        TactileSensorDialog(
                          dynamics::DynamicWorkcell *dwc,
                          QWidget *parent = 0);

        void drawTactileInput();

        void detectFeatures();

    signals:


    public slots:
        void zoomIn();
        void zoomOut();
        void rotateLeft();
        void rotateRight();
        void wheelEvent(QWheelEvent* event);

    private slots:
        void btnPressed();
        void changedEvent();
        void updateState();


    private:
        void initTactileInput();
        void detectCenterMass();
        void findMoments();
    private:
        Ui::TactileSensorDialog _ui;

        dynamics::DynamicWorkcell *_dwc;
        std::vector<TactileArraySensor*> _tsensors;
        std::vector<boost::numeric::ublas::matrix<float> > _values;
        QGraphicsScene *_scene;

        std::vector< std::vector<QGraphicsRectItem*> > _rectItems;
        std::vector<QGraphicsEllipseItem*> _centerItems;
        std::vector<QGraphicsLineItem*> _momentItems;
        std::vector<QGraphicsLineItem*> _momentSecItems;
        std::vector<std::pair<int,int> > _dims;


        //typedef std::pair<rw::math::Vector2D<>,rw::math::Vector2D<> > Moment;


        std::vector< rw::math::Vector2D<> > _centers;
        std::vector< Moment > _moments;
        int _nrOfPadsH;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
