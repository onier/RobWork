#ifndef DYNAMICS_RIGIDBODY_HPP_
#define DYNAMICS_RIGIDBODY_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/InertiaMatrix.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rw/kinematics/Kinematics.hpp>

#include "Body.hpp"

namespace dynamics {

    /**
     * @brief A body is an abstraction over some physical object in the scene.
     * The body class is an abstract class that allows interaction by adding gravity,
     * forces and impulses. The integration scheme used to update the velocity and position
     * of the body is defined by the class that inherit the body interface.
     */
    class RigidBody : public Body
    {
    public:
/*
        RigidBody(
            const BodyInfo& info,
            BodyIntegrator &integrator,
            rw::kinematics::MovableFrame& frame,
            const std::vector<rw::kinematics::Frame*>& frames,
            rw::kinematics::State& state
            );
*/
        RigidBody(
            const BodyInfo& info,
            rw::kinematics::MovableFrame& frame,
            const std::vector<rw::geometry::Geometry*>& geoms,
            rw::kinematics::State& state
            );

        virtual ~RigidBody(){};

    public: // functions that need to be implemented by specialized class

        /**
         * @copydoc Body::saveState
         */
        virtual void saveState(double h, rw::kinematics::State& state);

        /**
         * @copydoc Body::rollBack
         */
        virtual void rollBack(rw::kinematics::State& state);

        /**
         * @copydoc Body::updateVelocity
         */
        virtual void updateVelocity(double h, rw::kinematics::State& state);

        /**
         * @copydoc Body::updatePosition
         */
        virtual void updatePosition(double h, rw::kinematics::State& state);

        /**
         * @copydoc Body::updateImpulse
         */
        virtual void updateImpulse();

        /**
         * @copydoc Body::getPointVelW
         */
        rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p);

        /**
         * @copydoc Body::getEffectiveMassW
         */
        rw::math::InertiaMatrix<> getEffectiveMassW(const rw::math::Vector3D<>& wPc);

        /**
         * @copydoc Body::getMaterial
         */
        const std::string& getMaterial(){
            return _materialID;
        }

        /**
         * @copydoc Body::resetState
         */
        void resetState(rw::kinematics::State &state);

        /**
         * @copydoc Body::reset
         */
        virtual void reset(){
           rw::math::Vector3D<> zeroVec = rw::math::Vector3D<>(0.0,0.0,0.0);
           _force = zeroVec;
           _torque = zeroVec;
        }


    public:


        /**
         * @brief return the parent frame
         */
    	rw::kinematics::Frame& getParent() const {
    		return _parent;
    	};

    	int getBodyType() const {
    	    return _bodyType;
    	};

        /**
         * @brief Sets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual void setForce(const rw::math::Vector3D<>& f){
            _force = f;
        }

        /**
         * @brief Sets the force described in world frame acting on
         * the center mass of this body.
         */
        virtual void setForceW(const rw::math::Vector3D<>& f){
            _force = _pTw.R() * f;
        }

        /**
         * @brief Gets the force described in world frame acting on
         * the center mass of this body.
         */
        virtual rw::math::Vector3D<> getForceW(){
            return _wTp.R() * _force;
        }

        /**
         * @brief Gets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual const rw::math::Vector3D<>& getForce(){
            return _force;
        }

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addForce(const rw::math::Vector3D<>& force){
            _force += force;
        }

        /**
         * @brief Adds a force described in world frame to the
         * center of mass of this body
         */
        virtual void addForceW(const rw::math::Vector3D<>& force){
            _force += _pTw.R() * force;
        }

        /**
         * @brief Adds a force described in parent frame to this body
         * which is working on a specific position pos that is described relative to
         * this body.
         */
        virtual void addForceToPos(const rw::math::Vector3D<>& force,
                                   const rw::math::Vector3D<>& pos){
            // calculate the center force contribution
            _force += force;

            // calculate the torque contribution
            _torque += cross( pos, force );
        }

        /**
         * @brief Adds a force described in world frame to this body
         * which is worked on a specific position pos that is described
         * relative to world
         */
        virtual void addForceWToPosW(const rw::math::Vector3D<>& force,
                                     const rw::math::Vector3D<>& pos);

        /**
         * @brief Adds a impulse described in parent frame to this body
         * which is working on a specific position pos that is described relative to
         * this body.
         */
        virtual void addImpulseToPos(const rw::math::Vector3D<>& impulse,
                                     const rw::math::Vector3D<>& pos){
            // calculate the center force contribution
            _linImpulse += impulse;

            // calculate the torque contribution
            _angImpulse += cross( pos, impulse );
        }

        /**
         * @brief Adds a impulse described in world frame to this body
         * which is worked on a specific position pos that is described
         * relative to world
         */
        virtual void addImpulseWToPosW(const rw::math::Vector3D<>& impulse,
                                       const rw::math::Vector3D<>& pos);

        /**
         * @brief adds gravitation to the body where the gravitation is
         * described in body frame
         */
        virtual void addGravitation(const rw::math::Vector3D<>& grav){
            _force += grav * _mass;
        };

        /**
         * @brief adds gravitation to the body where the gravitation is
         * described in world frame
         */
        virtual void addGravitationW(const rw::math::Vector3D<>& grav){
            _force += (_pTw.R() * grav) * _mass;
        };

        /**
         * @brief sets gravitation to the body where the gravitation is
         * described in body frame
         */
        virtual void setGravitation(const rw::math::Vector3D<>& grav){
            _force = grav * _mass;
        };

        /**
         * @brief sets gravitation to the body where the gravitation is
         * described in world frame
         */
        virtual void setGravitationW(const rw::math::Vector3D<>& grav){
            _force = (_pTw.R() * grav) * _mass;
        };

        /**
         * @brief set the torque of this body with torque t, where t is
         * described in body frame
         */
        virtual void setTorque(const rw::math::Vector3D<>& t){
            _torque = t;
        }

        /**
         * @brief set the torque of this body with torque t, where t is
         * described in world frame
         */
        virtual void setTorqueW(const rw::math::Vector3D<>& t){
            _torque = _pTw.R() * t;
        }

        /**
         * @brief returns torque described in body frame
         */
        virtual const rw::math::Vector3D<>& getTorque(){
            return _torque;
        }

        /**
         * @brief returns torque described in world frame
         */
        virtual rw::math::Vector3D<> getTorqueW(){
            return _wTp.R() * _torque;
        }

        /**
         * @brief returns the transform from parent to body
         */
        virtual const rw::math::Transform3D<>& getPTBody(){
            return _pTb;
        }

        virtual void setPTBody(const rw::math::Transform3D<>& pTb, rw::kinematics::State& state){
            _pTb = pTb;
            _mframe.setTransform( _pTb , state );
        }

        /**
         * @brief returns the transform from world to body
         */
        virtual const rw::math::Transform3D<>& getWTBody(){
            return _wTb;
        }

        /**
         * @brief return the linear velocity described in parent frame
         */
        virtual rw::math::Vector3D<> getLinVel(){
            return _linVel;
        }

        /**
         * @brief return the linear velocity described in world frame
         */
        virtual rw::math::Vector3D<> getLinVelW(){
            return _wTp.R() * _linVel;
        }

        virtual void setLinVel(const rw::math::Vector3D<> &lvel){
            _linVel = lvel;
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        virtual const rw::math::Vector3D<>& getAngVel(){
            return _angVel;
        }

        /**
         * @brief returns the angular velocity described in world frame
         */
        virtual rw::math::Vector3D<> getAngVelW(){
            return _wTp.R() * _angVel;
        }

        virtual void setAngVel(const rw::math::Vector3D<> &avel){
            _angVel = avel;
        }

        /**
         * @brief calculates the relative velocity in parent frame of a point p on the body
         * described in parent frame.
         */
        rw::math::Vector3D<> getPointVel(const rw::math::Vector3D<>& p){
            return _linVel + cross(_angVel, p);
        }

        /**
         * @brief returns the mass of this body
         */
        inline double getMass() const {
            return _mass;
        }

        /**
         * @brief returns the inverse of the mass of this body
         */
        inline double getMassInv() const {
            return _massInv;
        }

        /**
         * @brief returns the body inertia matrix
         */
        const rw::math::InertiaMatrix<>& getBodyInertia() const {
            return _Ibody;
        };

        /**
         * @brief return the inverse of the body inertia matrix
         */
        const rw::math::InertiaMatrix<>& getBodyInertiaInv() const {
            return _IbodyInv;
        };

        /**
         * @brief returns the inverse of the inertia tensor described in
         * parent frame.
         */
        const rw::math::InertiaMatrix<>& getInertiaTensorInv() const {
            return _ITensorInv; ///pTb.R() * (IBodyInv * inverse(pTb.R()));
        }

        /**
         * @brief returns the inverse of the inertia tensor described in
         * world frame.
         */
        rw::math::InertiaMatrix<> getInertiaTensorInvW() const {
            return _wTb.R() * ( _IbodyInv * inverse( _wTb.R() ) );
        }

        /**
         * @brief returns the inverse of the inertia tensor described in
         * parent frame.
         */
        const rw::math::InertiaMatrix<>& getInertiaTensor() const {
            return _ITensor; //pTb.R() * (_IBody * inverse(pTb.R()));
        }


        rw::kinematics::MovableFrame& getMovableFrame(){
        	return _mframe;
        }

        void calcAuxVarialbles(rw::kinematics::State& state);

        /**
         *
         * @note Total energy = 1/2 * mv^2 + 1/2 * Iw^2 + mgz
         */
        double calcEnergy();

    protected:
        // const variables
        const double _mass,_massInv;

        rw::kinematics::MovableFrame &_mframe;
        rw::kinematics::Frame &_parent;

        std::string _materialID;

        int _bodyType;

        rw::math::InertiaMatrix<> _Ibody,
                                  _IbodyInv;

        // state variables
        rw::math::Transform3D<> _pTb; // position and orientation

        rw::math::InertiaMatrix<> _ITensorInv,_ITensor; // inverse inertia tensor in parent frame

        rw::math::Vector3D<> _force, _forceRB, // accumulated force in parent frame
                             _torque, _torqueRB; // accumulated torque in parent frame

        rw::math::Vector3D<> _linImpulse, // linear impulse in parent frame
                             _angImpulse; // angular impulse in parent frame

        rw::math::Vector3D<> _linVel, _linVelRB, // linear velocity in parent frame
                             _angVel, _angVelRB; // angular velocity in parent frame

        rw::math::Transform3D<> _wTb, // world to body
                                _bTw,// body to world
                                _pTw, // parent to world
                                _wTp; // world to parent

        // aux variables


    };
} // namespace dynamics
#endif /*DYNAMICS_RIGIDBODY_HPP_*/
