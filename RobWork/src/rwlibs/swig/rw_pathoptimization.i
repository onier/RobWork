%module rw_pathoptimization

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>

using namespace rwlibs::swig;
using rw::math::Metric;
using rw::trajectory::Path;
%}

%import <rwlibs/swig/rw.i>

%pragma(java) jniclassimports=%{
import org.robwork.rw.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.rw.*;
%}

class PathLengthOptimizer
{
public:

    %extend {

        PathLengthOptimizer(rw::common::Ptr<CollisionDetector> cd,
                            rw::common::Ptr<Device> dev,
                            const State &state)
        {
            rw::pathplanning::PlannerConstraint constraint =
                    rw::pathplanning::PlannerConstraint::make(cd.get(), dev, state);
            return new PathLengthOptimizer(constraint, rw::math::MetricFactory::makeEuclidean< rw::math::Q>());
        }

        PathLengthOptimizer(rw::common::Ptr<CollisionDetector> cd,
                            rw::common::Ptr<Device> dev,
                            rw::common::Ptr<Metric<rw::math::Q> > metric,
                            const State &state)
        {
            rw::pathplanning::PlannerConstraint constraint =
                    rw::pathplanning::PlannerConstraint::make(cd.get(), dev, state);
            return new PathLengthOptimizer(constraint, metric );
        }

        PathLengthOptimizer(rw::common::Ptr<PlannerConstraint> constraint,
                            rw::common::Ptr<Metric<rw::math::Q> > metric)
        {
            return new PathLengthOptimizer(*constraint, metric);
        }

        rw::common::Ptr<Path<rw::math::Q> > pathPruning(rw::common::Ptr<Path<rw::math::Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::pathPruning(*path);
            return rw::common::ownedPtr( new PathQ(res) );
        }
/*
        rw::common::Ptr<Path<rw::math::Q> > shortCut(rw::common::Ptr<Path<rw::math::Q> > path,
                                       size_t cnt,
                                       double time,
                                       double subDivideLength);
*/
        rw::common::Ptr<Path<rw::math::Q> > shortCut(rw::common::Ptr<Path<rw::math::Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::shortCut(*path);
            return rw::common::ownedPtr( new PathQ(res) );
        }

        rw::common::Ptr<Path<rw::math::Q> > partialShortCut(rw::common::Ptr<Path<rw::math::Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::partialShortCut(*path);
            return rw::common::ownedPtr( new PathQ(res) );
        }
/*
        rw::common::Ptr<Path<rw::math::Q> > partialShortCut(rw::common::Ptr<Path<rw::math::Q> > path,
                                              size_t cnt,
                                              double time,
                                              double subDivideLength);
                                              */
    }
    PropertyMap& getPropertyMap();

};