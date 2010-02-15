#ifndef DYNAMICS_DYNAMICWORKCELL_HPP_
#define DYNAMICS_DYNAMICWORKCELL_HPP_

#include <vector>
#include <map>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>

#include "Body.hpp"
#include "BodyController.hpp"
#include "ContactDataMap.hpp"
#include "MaterialDataMap.hpp"

#include "DynamicDevice.hpp"

namespace dynamics {

    /**
     * @brief the WorkCellDimension describe a center and the box halflengths of
     * the space that the WorkCell expands.
     *
     */
    struct WorkCellDimension {
    public:
        WorkCellDimension(const rw::math::Vector3D<>& c, const rw::math::Vector3D<>& box):
            center(c),boxDim(box){}
        rw::math::Vector3D<> center, boxDim;

    };

    /**
     * @brief the DynamicWorkcell class is a container class for dynamic
     * information/data in a workcell, much like WorkCell is a container
     * class for the kinematic information/data in a workcell
     *
     * The dynamic description includes:
     * Body: severel different forms of bodies that may be constrained by rules
     * Controllers: severel controllers that in some way control/influence the bodies
     * Sensors: bodies that have tactile sensing capability
     *
     */
    class DynamicWorkcell
    {
    public:
        typedef std::vector<Body*> BodyList;
        typedef std::vector<DynamicDevice*> DeviceList;
        typedef std::vector<rwlibs::simulation::SimulatedControllerPtr> ControllerList;
        typedef std::vector<rwlibs::simulation::SimulatedSensorPtr> SensorList;


        /**
         * @brief Constructor
         */
    	DynamicWorkcell(rw::models::WorkCellPtr workcell,
                        const BodyList& bodies,
                        const DeviceList& devices,
                        const ControllerList& controllers);

    	/**
    	 * @brief destructor
    	 */
    	virtual ~DynamicWorkcell();

    	/**
    	 * @brief gets a list of all bodies in the dynamic workcell
    	 */
    	const BodyList& getBodies(){ return _bodies;};

    	/**
    	 * @brief gets a list of all dynamic devices in the dynamic workcell
    	 */
    	const DeviceList& getDynamicDevices(){ return _devices; };

    	DynamicDevice* findDevice(const std::string& name);

    	/**
    	 * @brief gets a list of all controllers in the dynamic workcell
    	 */
        const ControllerList& getControllers(){
             return _controllers;
        }

        /**
         *
         * @param body
         */
        const SensorList& getSensors(){
            return _sensors;
        };


        void addSensor(rwlibs::simulation::SimulatedSensorPtr sensor){
            _sensors.push_back(sensor);
        };

        /**
         * @brief
         */
        //const std::vector<Constraint>& getConstraints();



        /**
         * @brief adds a body to the dynamic workcell.
         *
         * Notice that this will change the length of the default
         * State.
         */
        void addBody(Body* body){
            //TODO: change STATE and WorkCell accordingly
            _bodies.push_back(body);
        }

    	/**
    	 * @brief adds a body controller to the dynamic workcell.
    	 *
    	 * Notice that this will change the length of the default
    	 * State.
    	 */
    	void addController(rwlibs::simulation::SimulatedControllerPtr manipulator){
    	    //TODO: change STATE and WorkCell accordingly
    		_controllers.push_back(manipulator);
    	}

    	/**
    	 * @brief gets the static contact data information
    	 */
    	ContactDataMap& getContactData(){
    	    return _contactDataMap;
    	}

    	/**
    	 * @brief gets the material data information, like friction
    	 * properties
    	 */
    	MaterialDataMap& getMaterialData(){
    	    return _matDataMap;
    	}

        /**
          * @brief gets the body associated with frame f if any.
          */
        Body* getBody(rw::kinematics::Frame *f);

    	/**
    	 * @brief gets the default kinematic workcell
    	 */
    	rw::models::WorkCell* getWorkcell(){ return _workcell.get(); };

    	/**
    	 * @brief the collision margin describe how close
    	 */
    	double getCollisionMargin(){
    	    return _collisionMargin;
    	}

        void setCollisionMargin(double margin){
            _collisionMargin = margin;
        }

        WorkCellDimension getWorldDimension(){
            return _worldDimension;
        }

        void setGravity(const rw::math::Vector3D<>& grav){
            _gravity = grav;
        }

        const rw::math::Vector3D<>& getGravity(){
            return _gravity;
        }

        rw::common::PropertyMap& getEngineSettings(){
        	return _engineSettings;
        }

    private:
        // length of nr of bodies
        BodyList _bodies;

        DeviceList _devices;

        // length of workcell.getTree().getMaxID()
        std::vector<Body*> _frameIdToBody;

        // list of sensors
        SensorList _sensors;
        // list of controllers in the workcell
        ControllerList _controllers;

        // the workcell
        rw::models::WorkCellPtr _workcell;
        // deprecated
        std::map<rw::kinematics::Frame*,Body*> _frameToBody;

        double _collisionMargin;

        WorkCellDimension _worldDimension;
        rw::math::Vector3D<> _gravity;

        MaterialDataMap _matDataMap;
        ContactDataMap _contactDataMap;

        rw::common::PropertyMap _engineSettings;
    };

} // namespace dynamics

#endif /*DYNAMICWORKCELL_HPP_*/
