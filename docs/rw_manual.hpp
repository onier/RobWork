// -*- latex -*-

/**

\page page_rw_manual RobWork manual

These are the most important things to cover in this manual:

- Installation

- Libraries, their naming convention, which to use, and how to link to
  them.

- Howto for the most important and stable parts of RobWork.
.

Other things to cover as time permits include:

- Device and workcell file formats and the workcells and devices we
  provide.
.

\section sec_rw_manual_code_examples About the code examples of this manual

All code examples of this manual are self-contained in the sense that
they will compile on their own if pasted into an empty C++ file. Each
example can be found in a file of its own in the \c RobWork/docs
directory. See also the CMakeLists.txt file of this directory for the
setup of the compiler and linker flags.

\section sec_rw_manual_load_workcell Getting started: Loading a workcell

RobWork support workcells described in an XML format as well as the
.wu and .dev workcell file formats used by the TUL program.

The below program loads a workcell from the file named on the command
line. If the loading of the workcell fails, the
rw::loaders::WorkCellLoader::load() function will throw an exception,
and the program will abort with an error message.

\include ex-load-workcell.cpp

\section sec_sec_rw_manual_devices Devices of workcells

A workcell contains a number of devices (see rw::models::Device). You
can for example traverse the devices stored in a workcell and print
their names like this:

\include ex-print-devices.cpp

A device of a specific name can be retrieved from a workcell with
rw::models::WorkCell::findDevice().

\section sec_rw_manual_states Kinematics trees and states

The kinematic structure of the work cell is represented by a tree of
frames (see rw::kinematics::Frame). Each frame has a transformation
(see rw::math::Transform3D) relative to its parent frame and this
transformation may change in response to values assigned for the
frame. A revolute joint of a device (see rw::models::RevoluteJoint) is
for example implemented as a frame that has a single value that
rotates the frame relative to its parent.

It is important in RobWork to note that the values for the frames are
not stored \e within the frames, but are instead stored explicitly in
a value of type State. Given a state for the workcell, the transform
of a frame relative to its parent can be calculated with
rw::kinematics::Frame::getTransform().

The frames of the workcell are always organized in a tree, but for
certain frames the parent that they are connected to can dynamically
be changed. These frames are called \e dynamically \e attachable \e
frames or \e DAFs for short. The parent that a DAF is attached to is
not stored within the DAF itself, but is instead stored externally in
a State value. Different state values can thus correspond to different
structures of the tree. Given a state for the workcell the parent and
children of a frame can be retrieved with
rw::kinematics::Frame::getParent() and
rw::kinematics::Frame::getChildren().

Because the values of the frames and the attachments of DAFs are
stored outside of the workcell, we say that the workcell is \e
stateless.

To illustrate these important ideas, this example shows how to print
the structure of the kinematic tree of the workcell and for each frame
print also the position of the frame is space:

\include ex-print-kinematic-tree.cpp

Here is an example of output produced by the
printDefaultWorkCellStructure() function for a CRSA465 workcell:

\verbatim
WorkCell[d:/src/RobWorkData/TULDevices/CRSA465/CRSA465.dev]
WORLD at Vector3D {0, 0, 0}
 base at Vector3D {0, 0, 0}
  led1 at Vector3D {0, 0, 0.33}
   led2 at Vector3D {0, 0, 0.33}
    led3 at Vector3D {0, 0, 0.635}
     led4 at Vector3D {0, 0, 0.968}
      led5 at Vector3D {0, 0, 0.968}
       led6 at Vector3D {0, 0, 1.044}
        TCP at Vector3D {0, 0, 1.044}
        Link6Geo at Vector3D {0, 0, 1.044}
       Link5Geo at Vector3D {0, 0, 0.968}
      Link4Geo at Vector3D {0, 0, 0.968}
     Link3Geo at Vector3D {0, 0, 0.635}
    Link2Geo at Vector3D {0, 0, 0.33}
   Link1Geo at Vector3D {0, 0, 0.33}
  Link0Geo at Vector3D {0, 0, 0}
\endverbatim

We see from this example that given a state, it is straight-forward to
compute the transform of every single frame in the workcell. RobWork
has some utilities to make calculation of forward kinematics
convenient in the day to day work.

FKTable is used for computing the forward kinematics of a number of
frames for a common state. The results of the forward kinematics are
stored in the FKTable object so that the transform for a frame is not
computed over and over again. This example shows how the transform for
a sequence of frames can be efficiently computed:

\include ex-world-transforms.cpp

\section sec_rw_manual_device_configurations Device configurations and states

Simple path planners don't operate on the level of frames and the
values for frames. Instead the operate on devices (see
rw::models::Device) and configurations (see rw::math::Q) for devices.





*/

/*
----------------------------------------------------------------------
Dead text

The work cell state contains for each frame a number of joint values
for the frame. You can read or write the joint values that belong to a
frame to or from a state with Frame::getQ() and Frame::setQ().

The work cell state contains also information about the structure of
the tree. By changing the work cell state one can release a frame from
the tree and attach it to a new parent frame. We give examples of
this in Section \ref sec_frame_attachments.

*/

/*

Here is how you can include example code in the manual:

\include ex-load-workcell.cpp

This is equivalent to the following:

\code
#include <string>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

... and so on ...
\endcode

Jeg vil foresl� at lade alle eksempler v�re komplette i sig selv, dvs.
hvert eksempel placeres i en fil og man kontrollerer at alle eksempler
compilerer. Se CMakeLists.txt i dette directory.

*/
