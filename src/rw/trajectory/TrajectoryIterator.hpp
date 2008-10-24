/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed Actionrmation about these packages.
 *********************************************************************/

#ifndef RW_TRAJECTORY_TRAJECTORYITERATOR_HPP
#define RW_TRAJECTORY_TRAJECTORYITERATOR_HPP

/**
 * @file TrajectoryIterator.hpp
 */

#include "Trajectory.hpp"

#include <vector>

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Bi-directional iterator for running efficiently through a trajectory
     */
    template <class T>
    class TrajectoryIterator
    {
    public:
        /**
         * @brief Constructs iterator for \b trajectory
         *
         * @param trajectory [in] Trajectory to iterate through
         * @param dt [in] Default stepsize used for ++ and -- operators
         */
        TrajectoryIterator(const Trajectory<T>* trajectory, double dt = 1)
        {
            _trajectory = trajectory;
            _dt = dt;
            _time = 0;
            _currentSegment = trajectory->_segments.begin();
        }

        /**
         * @brief Returns the current position (time) of the iterator
         * @return The current time.
         */
        double getTime() { return _time; }

        /**
         * @brief This function can be used to decrease the iterator position.
         * The position can be decreased no longer than to time equals 0.
         *
         * @param dt [in] a double that describes how much to decrease the
         * iterator position
         */
        void operator -=(double dt)
        {
            if (_time - dt < 0)
                _time = 0;
            else
                _time -= dt;
            while (_time < _currentSegment->t1 )
                _currentSegment--;
        }

        /**
         * @brief This function can be used to increase the iterator position.
         * The position can be increased no longer than the length of the
         * complete trajectory.
         *
         * @param dt [in] a double that describes how much to increase the
         * iterator position
         */
        void operator +=(double dt)
        {
            if (_time + dt > _trajectory->duration())
                _time = _trajectory->duration();
            else
                _time += dt;
            while (_time > _currentSegment->t2 )
                _currentSegment++;
        }

        /**
         * @brief Operator overloading ++ for increasing the position of the iterator.
         *
         * The increment is equal to the \b dt specified in the constructor.
         * @return Reference to the TrajectoryIterator
         */
        TrajectoryIterator& operator++()
        {
            (*this) += _dt;
            return *this;
        }

        /**
         * @brief Operator overloading -- for decreasing the position of the iterator.
         *
         * The decrement is equal to the \b dt specified in the constructor.
         * @return Reference to the TrajectoryIterator
         */
        TrajectoryIterator& operator--()
        {
            (*this) -= _dt;
            return *this;
        }

        /**
         * @brief Test if the end of the trajectory is reached.
         *
         * @return true if the iterator has reached the end of the trajectory false
         * otherwise.
         */
        bool isEnd() { return _time >= _trajectory->duration(); }

        /**
         * @brief Test if the beginning of the trajectory is reached.
         *
         * @return true if the iterator has reached the beginning of the trajectory
         * false otherwise.
         */
        bool isBegin() { return _time <= 0; }

        /**
         * @brief Extracts a point at the current position in the trajectory.
         *
         * @return the point at the current position in the trajectory.
         */
        T operator*() const { return x(); }

        /**
         * @brief Extracts a point at the current position in the trajectory.
         *
         * @return the point at the current position in the trajectory.
         */
        T x() const {
            return _trajectory->getX(*_currentSegment, _time);
        }

        /**
         * @brief Extracts a point of the derivative of the trajectory
         * at the current position in the trajectory
         *
         * @return the derived point at the current position in the trajectory.
         */
        T dx() const {
            return _trajectory->getDX(*_currentSegment, _time);
        }

        /**
         * @brief Extracts a point of the double derivative of the trajectory at the
         * current position in the trajectory
         *
         * @return the double derived point at the current position in the trajectory.
         */
        T ddx() const {
            return _trajectory->getDDX(*_currentSegment, _time);
        }

    private:
        typename Trajectory<T>::SegmentList::const_iterator _currentSegment;
        const Trajectory<T>* _trajectory;
        double _time;
        double _dt;
    };

    /** @} */

}} // end namespaces

#endif // end include guard
