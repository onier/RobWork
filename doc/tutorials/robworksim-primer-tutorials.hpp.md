RobWorkSim - Dynamic Simulation in RobWork  {#pageRobWorkSimPrimer}
===============================

[TOC]

# Introduction #
This page describes how to do dynamic simulation in C++. Please see the page about the \ref page_xml_dynamicworkcell_format "XML Dynamic WorkCell Format" for details on how to define objects, constraints, controllers, sensors and much more.

# Instantiation of a Physics Engine #
The following lines of code shows how to instantiate and step an \ref rwsim::simulator::ODESimulator "ODE" physics engine.
~~~~{.cpp}
    const DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load("dynamic_workcell.dwc.xml");
    if (dwc == NULL)
    	RW_THROW("Error happened when loading dynamic workcell.");
    const PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine("ODE",dwc);
    if (engine == NULL)
    	RW_THROW("Engine could not be found.");
    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();
    engine->initPhysics(state);
    for(int i=0; i<100; i++) {
        engine->step(0.01, state);
    }
    engine->exitPhysics();
~~~~
The factory method for creating engines, uses the RobWork plugin structure. The plugins are in most cases loaded dynamically, hence the available engines can change depending on which plugins have been compiled and found.

# Physics Engines #
RobWorkSim allows using different physics engines. Examples of typical engines are:
 - \ref rwsim::simulator::ODESimulator "ODE"
 - \ref rwsimlibs::rwpe::RWPEPhysics "RWPEPhysics"
 - \ref rwsimlibs::rwpe::RWPEWorld "RWPEWorld"
 - \ref rwsimlibs::rwpe::RWPEIsland "RWPEIsland"
 - \ref rwsim::simulator::RWSimulator "RWPhysics"
 - \ref rwsim::simulator::BtSimulator "BtSimulator"

## The RobWorkPhysicsEngines ##
The \ref rwsimlibs::rwpe::RWPEPhysics "RWPEPhysics" engine is developed specifically for simulation of manipulation tasks characterized by few dynamic objects and few contacts.

# Tutorial 1 - a simulation loop #

This tutorial demonstrate how a dynamic simulator can be constructed in c++ 
and how the user can control the simulator.

~~~~{.cpp}
void main(char** 
    // Load dynamic
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load( testFilePath() + "/ur_control_test_scene/cup_pg70_table.dwc.xml");

    SerialDeviceController::Ptr devctrl = dwc->findController<SerialDeviceController>("URController");
    Device::Ptr ur = dwc->getWorkcell()->findDevice("UR-6-85-5-A");
    ODESimulator::Ptr odesim = ownedPtr( new ODESimulator( dwc ) );

    State state = dwc->getWorkcell()->getStateStructure()->getDefaultState();

    FKTable table(state);

    // test that the control interface works
    odesim->initPhysics(state);
    Q target(6,0,-0.2,0,0,0,0);
    devctrl->movePTP( target, 100);
    for(int i=0; i<200; i++){
    	std::cout << i << ":";
    	odesim->step(0.01, state);
    	std::cout << ur->getQ(state) << std::endl;
    }
~~~~
