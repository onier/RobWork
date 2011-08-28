#ifndef FTCOMPENSATION_HPP
#define FTCOMPENSATION_HPP

// Boost
#include <boost/date_time/posix_time/posix_time.hpp>

// RW
#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models/Device.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

namespace rwhw {
   /**
    * @brief Compensator for the NetFT device based on a calibration.    * 
    * 
    * This class reads in a current joint configuration,
    * translates this to a F/T tool configuration and outputs
    * a status flag based on a threshold value which tells whether
    * a collision has occurred.
    * 
    * Example usage:
    * 
    * \code
    * FTCompensation ftc(robotdevice, state, calibfile);
    * while(RUNNING) {
    *    ftc.update(ft, q, dq, ddq); // Updates the collision flag based on current measurements
    *    if(ftc.getCollision())
    *       ACTION
    * }
    * \endcode
    */
   class FTCompensation {
      public:
         /**
          * @brief F/T vector type
          */
         typedef std::pair<Vector3D<>, Vector3D<> > Wrench3D;
         
         /**
          * @brief F/T calibration
          */
         struct FTCalib {
            /**
             * Calibration identifier label
             */
            std::string label;
            
            /**
             * Calibrated mass
             */
            double m;
            
            /**
             * Calibrated tool roll
             */
            double a;
            
            /**
             * Calibrated bias
             */
            Wrench3D bias;
         };
         
         /**
          * @brief Constructor for an input device, a workcell state which is copied and a calibration file.
          * 
          * @param dev the robot device
          * @param state the workcell state used for forward kinematics
          * @param calibfile file path for a F/T calibration
          * @param thres the collision threshold
          */
         FTCompensation(Device::Ptr dev,
                        State state,
                        const std::string& calibfile = "ftcalib.xml",
                        const Wrench3D& thres = Wrench3D(Vector3D<>(5, 5, 5), Vector3D<>(0.5, 0.5, 0.5)));
         
         virtual ~FTCompensation() {}
         
         /**
          * @brief Updates the collision flag based on current joint configuration and F/T vector.
          * Joint velocity and acceleration are estimated based on input and an internal timer.
          * 
          * The use of this function is in general discouraged since higher-order differentials destabilize.
          * 
          * @param ft current wrench
          * @param q current joint configuration
          */
         inline void update(const Wrench3D& ft, const Q& q) {
            // Get elapsed time since last call
            const boost::posix_time::ptime currentT = boost::posix_time::microsec_clock::universal_time();

            // Update clock
            const boost::posix_time::time_duration diffT = currentT - _prevT;
            const double dt = double(diffT.total_microseconds()) * 0.000001;            
            _prevT = currentT;
            
            // Estimate velocity by backward difference
            const Q dq = (q - _qP) / dt;
            
            // Update previous position
            _qP = q;
            
            update(ft, q, dq, dt);            
         }
         
         /**
          * @brief Updates the collision flag based on current joint configuration, velocity and F/T vector.
          * Joint velocity and acceleration are estimated based on input and an internal timer.
          * 
          * @param ft current wrench
          * @param q current joint configuration
          * @param dq current joint velocity
          */
         inline void update(const Wrench3D& ft, const Q& q, const Q& dq) {
            // Get elapsed time since last call
            const boost::posix_time::ptime currentT = boost::posix_time::microsec_clock::universal_time();

            // Update clock
            const boost::posix_time::time_duration diffT = currentT - _prevT;
            const double dt = double(diffT.total_microseconds()) * 0.000001;            
            _prevT = currentT;
            
            update(ft, q, dq, dt);
         }
         
         /**
          * @brief Updates the collision flag based on current joint configuration, velocity and F/T vector.
          * Joint velocity and acceleration are estimated based on input and an internal timer.
          * 
          * @param ft current wrench
          * @param q current joint configuration
          * @param dq current joint velocity
          * @param dt time between samples
          */
         inline void update(const Wrench3D& ft, const Q& q, const Q& dq, double dt) {
            // Estimate acceleration by backward difference
            const Q ddq = (dq - _dqP) / dt;
            
            // Update previous velocity
            _dqP = dq;
            
            update(ft, q, dq, ddq);
         }
         
         /**
          * @brief Updates the current wrench and the  collision flag
          * based on current joint configuration, velocity, acceleration and F/T vector.
          * 
          * @param ft current wrench
          * @param q current joint configuration
          * @param dq current joint velocity
          * @param ddq current joint acceleration
          */
         void update(const Wrench3D& ft, const Q& q, const Q& dq, const Q& ddq);
         
         /**
          * Get current wrench, which is compensated internally
          * 
          * @return current wrench, which is compensated internally
          */
         inline const Wrench3D& getFT() const { return _ft; }
         
         /**
          * @brief Collision status
          * 
          * @return collision flag
          */
         inline double getCollision() const { return _collision; }
         
         /**
          * @brief Set calibration
          * 
          * @param calib calibration
          */
         inline void setCalib(const FTCalib& calib) { _calib = calib; }
         
         /**
          * Utility function: load a calibration from an XML file
          * 
          * @param filename the file to load from
          * @param calib the calibration object to write to
          * @param eTft the calibrated end-effector to F/T transformation
          * @return failure is loading fails
          */
         static bool LoadCalib(const std::string& filename, FTCalib& calib, Transform3D<>& eTft);
         
      private:
         /**
          * Default constructor
          */
         FTCompensation() {}
         
         /**
          * Calculate F/T tool forward kinematics
          */
         inline void fk(const Q& q) {
            // Update robot tool FK
            _dev->setQ(q, _state);
            _bTft = _dev->baseTend(_state);
            
            // Transform to F/T tool
            _bTft.R() = _bTft.R() * _eTft.R();
         }
         
         /**
          * Bias F/T vector
          */
         void bias();
         
         /**
          * Compensate F/T measurement for gravity
          */
         void gravitate();
         
         /**
          * Device pointer
          */
         Device::Ptr _dev;
         
         /**
          * Workcell state copy
          */
         State _state;
         
         /**
          * Copy of current wrench, which is compensated internally
          */
         Wrench3D _ft;
         
         /**
          * Robot tool to F/T tool transformation
          */
         Transform3D<> _eTft;
         
         /**
          * Current forward kinematics of the F/T tool based on current joint position
          */
         Transform3D<> _bTft;
         
         /**
          * Threshold value, must be positive
          */
         Wrench3D _thres;
         
         /**
          * Calibration
          */
         FTCalib _calib;
         
         /**
          * Timer used for estimating joint velocity/acceleration if necessary
          */
         boost::posix_time::ptime _prevT;
         
         /**
          * Previous joint position/velocity for estimating acceleration
          */
         Q _qP, _dqP;
         
         /**
          * Collision flag
          */
         bool _collision;
   };
}

#endif