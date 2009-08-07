/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_ROBWORK_HPP
#define RW_ROBWORK_HPP

/**
 * @file robwork.hpp
 */

/*
  This file includes all header files of robwork.
*/

#include "./common/Cache.hpp"
#include "./common/ConcatVectorIterator.hpp"
//#include "./common/ConvertUtil.hpp"
#include "./common/Exception.hpp"
#include "./common/IOUtil.hpp"
#include "./common/Log.hpp"
#include "./common/LogBufferedChar.hpp"
#include "./common/LogBufferedMsg.hpp"
#include "./common/LogStreamWriter.hpp"
#include "./common/LogWriter.hpp"
#include "./common/macros.hpp"
#include "./common/Message.hpp"
#include "./common/os.hpp"
#include "./common/Property.hpp"
#include "./common/PropertyBase.hpp"
#include "./common/PropertyMap.hpp"
#include "./common/PropertyType.hpp"
#include "./common/Ptr.hpp"
#include "./common/StringUtil.hpp"
#include "./common/Timer.hpp"
#include "./common/TimerUtil.hpp"
#include "./common/VectorIterator.hpp"
#include "./geometry/Face.hpp"
#include "./geometry/FaceArrayFactory.hpp"
#include "./geometry/Geometry.hpp"
#include "./geometry/GeometryBox.hpp"
#include "./geometry/GeometryCylinder.hpp"
#include "./geometry/GeometryFactory.hpp"
#include "./geometry/GeometrySTL.hpp"
#include "./invkin/CCDSolver.hpp"
#include "./invkin/ClosedFormIK.hpp"
#include "./invkin/IKMetaSolver.hpp"
#include "./invkin/IterativeIK.hpp"
//#include "./invkin/IterativeIKSetup.hpp"
#include "./invkin/IterativeMultiIK.hpp"
#include "./invkin/ParallelIKSolver.hpp"
#include "./invkin/PieperSolver.hpp"
#include "./invkin/ResolvedRateSolver.hpp"
#include "./invkin/SimpleMultiSolver.hpp"
#include "./invkin/SimpleSolver.hpp"
#include "./kinematics/FixedFrame.hpp"
#include "./kinematics/FKRange.hpp"
#include "./kinematics/FKTable.hpp"
#include "./kinematics/Frame.hpp"
#include "./kinematics/FrameProperty.hpp"
#include "./kinematics/FramePropertyImpl.hpp"
#include "./kinematics/FrameType.hpp"
#include "./kinematics/Kinematics.hpp"
#include "./kinematics/MovableFrame.hpp"
#include "./kinematics/QState.hpp"
#include "./kinematics/State.hpp"
#include "./kinematics/StateData.hpp"
#include "./kinematics/StateSetup.hpp"
#include "./kinematics/StateStructure.hpp"
#include "./kinematics/TreeState.hpp"
#include "./loaders/colsetup/CollisionSetupLoader.hpp"
#include "./loaders/image/PGMLoader.hpp"
//#include "./loaders/image/PNGLoader.hpp"
#include "./loaders/path/PathLoader.hpp"
#include "./loaders/rwxml/DependencyGraph.hpp"
#include "./loaders/rwxml/MultipleFileIterator.hpp"
#include "./loaders/rwxml/XMLParserUtil.hpp"
#include "./loaders/rwxml/XMLRWLoader.hpp"
#include "./loaders/rwxml/XMLRWParser.hpp"
#include "./loaders/rwxml/XMLRWPreParser.hpp"
//#include "./loaders/TaskLoader.hpp"
#include "./loaders/tul/Tag.hpp"
#include "./loaders/tul/TULLoader.hpp"
#include "./loaders/WorkCellLoader.hpp"
#include "./loaders/rwxml/XML.hpp"
#include "./loaders/rwxml/XMLErrorHandler.hpp"
#include "./loaders/rwxml/XMLParser.hpp"
#include "./math/Constants.hpp"
#include "./math/EAA.hpp"
#include "./math/InertiaMatrix.hpp"
#include "./math/Jacobian.hpp"
#include "./math/Line2D.hpp"
#include "./math/LinearAlgebra.hpp"
#include "./math/Math.hpp"
#include "./math/Metric.hpp"
#include "./math/MetricFactory.hpp"
#include "./math/MetricUtil.hpp"
#include "./math/PerspectiveTransform2D.hpp"
#include "./math/PerspectiveTransform3D.hpp"
#include "./math/Pose6D.hpp"
#include "./math/Q.hpp"
#include "./math/Quaternion.hpp"
#include "./math/Rotation2D.hpp"
#include "./math/Rotation3D.hpp"
#include "./math/Rotation3DVector.hpp"
#include "./math/RPY.hpp"
#include "./math/Transform2D.hpp"
#include "./math/Transform3D.hpp"
#include "./math/Vector2D.hpp"
#include "./math/Vector3D.hpp"
#include "./math/VelocityScrew6D.hpp"
#include "./models/Accessor.hpp"
//#include "./models/BasicDevice.hpp"
//#include "./models/BasicDeviceJacobian.hpp"
#include "./models/Cartesian6DOFDevice.hpp"
#include "./models/CollisionModelInfo.hpp"
#include "./models/CompositeDevice.hpp"
//#include "./models/Conveyor.hpp"
//#include "./models/ConveyorBelt.hpp"
//#include "./models/ConveyorItem.hpp"
//#include "./models/ConveyorSegment.hpp"
#include "./models/Device.hpp"
//#include "./models/DeviceJacobian.hpp"
#include "./models/DrawableModelInfo.hpp"
#include "./models/VirtualJoint.hpp"
#include "./models/JacobianUtil.hpp"
#include "./models/Joint.hpp"
#include "./models/JointDevice.hpp"
#include "./models/MobileDevice.hpp"
#include "./models/Models.hpp"
#include "./models/ParallelDevice.hpp"
#include "./models/ParallelLeg.hpp"
//#include "./models/PassivePrismaticFrame.hpp"
#include "./models/DependentRevoluteJoint.hpp"
#include "./models/DependentPrismaticJoint.hpp"
#include "./models/PrismaticJoint.hpp"
#include "./models/RevoluteJoint.hpp"
#include "./models/RigidBodyInfo.hpp"
#include "./models/SerialDevice.hpp"
#include "./models/TimeMetricUtil.hpp"
#include "./models/TreeDevice.hpp"
#include "./models/WorkCell.hpp"
#include "./pathplanning/PathPlanner.hpp"
#include "./pathplanning/PlannerConstraint.hpp"
#include "./pathplanning/PlannerUtil.hpp"
#include "./pathplanning/QConstraint.hpp"
#include "./pathplanning/QEdgeConstraint.hpp"
#include "./pathplanning/QIKSampler.hpp"
#include "./pathplanning/QNormalizer.hpp"
#include "./pathplanning/QSampler.hpp"
#include "./pathplanning/QToQPlanner.hpp"
#include "./pathplanning/QToQSamplerPlanner.hpp"
#include "./pathplanning/QToTPlanner.hpp"
#include "./pathplanning/StateConstraint.hpp"
#include "./pathplanning/StopCriteria.hpp"
#include "./proximity/CollisionDetector.hpp"
#include "./proximity/CollisionSetup.hpp"
#include "./proximity/CollisionStrategy.hpp"
#include "./proximity/CollisionToleranceStrategy.hpp"
#include "./proximity/DistanceCalculator.hpp"
#include "./proximity/DistanceStrategy.hpp"
#include "./proximity/DistanceToleranceStrategy.hpp"
#include "./proximity/ProximityCommon.hpp"
#include "./proximity/ProximityStrategy.hpp"
#include "./sensor/Camera.hpp"
#include "./sensor/CameraListener.hpp"
#include "./sensor/Image.hpp"
#include "./sensor/Sensor.hpp"
#include "./trajectory/Blend.hpp"
#include "./trajectory/CircularInterpolator.hpp"
#include "./trajectory/CubicSplineFactory.hpp"
#include "./trajectory/CubicSplineInterpolator.hpp"
#include "./trajectory/Interpolator.hpp"
#include "./trajectory/InterpolatorUtil.hpp"
#include "./trajectory/LinearInterpolator.hpp"
#include "./trajectory/LloydHaywardBlend.hpp"
#include "./trajectory/ParabolicBlend.hpp"
#include "./trajectory/Path.hpp"
#include "./trajectory/Timed.hpp"
#include "./trajectory/TimedUtil.hpp"
#include "./trajectory/Trajectory.hpp"
#include "./trajectory/TrajectoryFactory.hpp"
#include "./trajectory/TrajectoryIterator.hpp"

#endif // end include guard
