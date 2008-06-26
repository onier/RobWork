/*********************************************************************
 * RobWork Version 0.2
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
 * for detailed information about these packages.
 *********************************************************************/

#include "QConstraint.hpp"
#include "StateConstraint.hpp"

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::pathplanning;

namespace
{
    class FromStateConstraint : public QConstraint
    {
    public:
        FromStateConstraint(
            StateConstraintPtr detector,
            DevicePtr device,
            const State& state)
            :
            _detector(detector),
            _device(device),
            _state(state)
        {
            RW_ASSERT(device);
            RW_ASSERT(detector);
        }

        bool inCollision(const Q& q) const
        {
            State state = _state;
            _device->setQ(q, state);
            return _detector->inCollision(state);
        }

    private:
        StateConstraintPtr _detector;
        DevicePtr _device;
        State _state;
    };

    class FromConstraints : public QConstraint
    {
    public:
        FromConstraints(
            const std::vector<QConstraintPtr>& constraints) :
            _constraints(constraints)
        {}

        bool inCollision(const Q& q) const
        {
            BOOST_FOREACH(const QConstraintPtr& sc, _constraints) {
                if (sc->inCollision(q))
                    return true;
            }
            return false;
        }

    private:
        std::vector<QConstraintPtr> _constraints;
    };

    class NormalizedConstraint : public QConstraint
    {
    public:
        NormalizedConstraint(
            QConstraintPtr constraint,
            const QNormalizer& normalizer)
            :
            _constraint(constraint),
            _normalizer(normalizer)
        {}

        bool inCollision(const Q& raw_q) const
        {
            Q q = raw_q;
            _normalizer.setFromNormalized(q);
            return _constraint->inCollision(q);
        }

    private:
        QConstraintPtr _constraint;
        QNormalizer _normalizer;
    };

    class FixedConstraint : public QConstraint
    {
    public:
        FixedConstraint(bool value) : _value(value) {}

        bool inCollision(const Q&) const { return _value; }

    private:
        bool _value;
    };

    typedef std::auto_ptr<QConstraint> T;
}

std::auto_ptr<QConstraint> QConstraint::makeFixed(bool value)
{
    return T(new FixedConstraint(value));
}

T QConstraint::make(
    StateConstraintPtr detector,
    DevicePtr device,
    const State& state)
{
    return T(
        new FromStateConstraint(
            detector,
            device,
            state));
}

T QConstraint::make(
    CollisionDetectorPtr detector,
    DevicePtr device,
    const State& state)
{
    return make(
        StateConstraint::make(detector),
        device,
        state);
}

T QConstraint::makeMerged(
    const std::vector<QConstraintPtr>& constraints)
{
    return T(new FromConstraints(constraints));
}

std::auto_ptr<QConstraint> QConstraint::makeMerged(
    const QConstraintPtr& ca,
    const QConstraintPtr& cb)
{
    std::vector<QConstraintPtr> cs;
    cs.push_back(ca);
    cs.push_back(cb);
    return makeMerged(cs);
}

T QConstraint::makeNormalized(
    const QConstraintPtr& constraint,
    const QNormalizer& normalizer)
{
    return T(new NormalizedConstraint(constraint, normalizer));
}

T QConstraint::makeNormalized(
    const QConstraintPtr& constraint,
    const std::pair<Q, Q>& bounds)
{
    return makeNormalized(constraint, QNormalizer(bounds));
}

T QConstraint::makeNormalized(
    const QConstraintPtr& constraint,
    const Device& device)
{
    return makeNormalized(constraint, QNormalizer(device.getBounds()));
}
