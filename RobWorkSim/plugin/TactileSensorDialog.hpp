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

    private:
        Ui::TactileSensorDialog _ui;

        dynamics::DynamicWorkcell *_dwc;
        std::vector<TactileArraySensor*> _tsensors;
        std::vector<boost::numeric::ublas::matrix<float> > _values;
        QGraphicsScene *_scene;

        std::vector< std::vector<QGraphicsRectItem*> > _rectItems;
        std::vector<QGraphicsEllipseItem*> _centerItems;
        int _w;
        int _h;
        std::vector< rw::math::Vector2D<> > _centers;
        int _nrOfPadsH;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
