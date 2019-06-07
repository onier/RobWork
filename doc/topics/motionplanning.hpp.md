Motion planning  {#page_rw_motionplanning}
===============

[TOC]

# Introduction # {#sec_rw_motionplanning_intro}
Motion planning is about planning a collision free path from one robot configuration to another, avoiding any obstacles in the workspace.
In the core RobWork library (rw::pathplanning namespace), there is an interface for such planners called \ref rw::pathplanning::QToQPlanner "QToQPlanner".
RobWork provides a pathplanning library with implementation of various planning algorithms. These are found under the namespace rwlibs::pathplanners and implements the \ref rw::pathplanning::QToQPlanner "QToQPlanner" interface:
* \ref rwlibs::pathplanners::ARWPlanner "ARWPlanner": Adaptive Random Walk Planner
* \ref rwlibs::pathplanners::PRMPlanner "PRMPlanner": Probabilistic RoadMap Planner
* \ref rwlibs::pathplanners::RRTPlanner "RRTPlanner": Rapidly-exploring Random Tree Planner
* \ref rwlibs::pathplanners::SBLPlanner "SBLPlanner": Single-query Bi-directional Lazy collision checking Planner
* \ref rwlibs::pathplanners::Z3Planner "Z3Planner"

See the documentation for each of these classes for references to litterature and more information about possible variants of the algorithms.

# RobWorkStudio Planning Plugin # {#sec_rw_motionplanning_rws}

# RobWork C++ # {#sec_rw_motionplanning_rw_cpp}

# RobWork Python # {#sec_rw_motionplanning_rw_python}

# RobWork Java # {#sec_rw_motionplanning_rw_java}

# RobWork LUA # {#sec_rw_motionplanning_rw_lua}
## LUA in RobWorkStudio ## {#sec_rw_motionplanning_rws_lua}