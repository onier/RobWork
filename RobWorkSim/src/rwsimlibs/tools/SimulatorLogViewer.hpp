/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TOOLS_SIMULATORLOGVIEWER_HPP_
#define RWSIMLIBS_TOOLS_SIMULATORLOGVIEWER_HPP_

/**
 * @file SimulatorLogViewer.hpp
 *
 * \copydoc rwsimlibs::tools::SimulatorLogViewer
 */

#include <QMainWindow>

#include <rw/common/Ptr.hpp>

namespace Ui { class SimulatorLogViewer; }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }
namespace rwsimlibs { namespace gui { class SimulatorLogWidget; } }

namespace rwsimlibs {
namespace tools {
//! @addtogroup rwsimlibs_tools

//! @{
/**
 * @brief Stand-alone application for visualization of internal data from a Physics Engine.
 *
 * Primary use of this tool is for debugging of engines.
 */
class SimulatorLogViewer: public QMainWindow {
    Q_OBJECT
public:
	SimulatorLogViewer();
	virtual ~SimulatorLogViewer();
	void setDWC(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);
	void setLog(rw::common::Ptr<rwsim::log::SimulatorLogScope> log);

private slots:
	void openDWC();
	void closeDWC();

signals:
	void dwcChanged(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

private:
    Ui::SimulatorLogViewer* const _ui;
    rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::common::Ptr<rwsim::log::SimulatorLogScope> _log;
    rwsimlibs::gui::SimulatorLogWidget* _widget;
};
//! @}
} /* namespace tools */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TOOLS_SIMULATORLOGVIEWER_HPP_ */
