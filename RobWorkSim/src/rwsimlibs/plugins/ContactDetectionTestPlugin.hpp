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

#ifndef RWSIMLIBS_PLUGINS_CONTACTDETECTIONTESTPLUGIN_HPP_
#define RWSIMLIBS_PLUGINS_CONTACTDETECTIONTESTPLUGIN_HPP_

/**
 * @file ContactDetectionTestPlugin.hpp
 *
 * \copydoc rwsimlibs::plugins::ContactDetectionTestPlugin
 */

#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsimlibs/test/ContactTest.hpp>

class PropertyViewEditor;
namespace rw { namespace models { class WorkCell; } }
namespace rwsim { namespace contacts { class ContactDetectorData; } }
namespace rwsim { namespace contacts { class ContactDetectorTracking; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }
namespace rwsimlibs { namespace gui { class SimulatorLogWidget; } }

namespace Ui { class ContactDetectionTestPlugin; }

namespace rwsimlibs {
namespace plugins {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class ContactDetectionTestPlugin: public rws::RobWorkStudioPlugin {
	Q_OBJECT
	Q_INTERFACES( rws::RobWorkStudioPlugin )
	#if RWS_USE_QT5
		Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "ContactDetectionTestPlugin.json")
	#endif
public:
	ContactDetectionTestPlugin();

	virtual ~ContactDetectionTestPlugin();

	void initialize();

private slots:
	void toolBoxChanged(int index);
	void inputChanged();
	void setPose(QListWidgetItem* item);
	void buttonClick();
	void updateGraphics();
	void autoDetect(bool checked);

private:
	void buttonClick(const QObject* button);
	void message(const std::string& msg);
	void error(const std::string& msg);

	void stateChangedListener(const rw::kinematics::State& state);

private:
    Ui::ContactDetectionTestPlugin* const _ui;
    PropertyViewEditor* const _inputEditor;
    rwsimlibs::test::ContactTest::Ptr _test;
    rw::common::Ptr<rw::common::PropertyMap> _input;
    rw::common::Ptr<rw::models::WorkCell> _wc;
    std::map<std::string, rw::kinematics::State> _poses;
    rw::common::Ptr<rwsim::contacts::ContactDetector> _detector;
    rwsim::contacts::ContactDetectorData* const _data;
    rwsim::contacts::ContactDetectorTracking* const _tracking;
    rwsim::log::SimulatorLogScope* const _log;
    rwsimlibs::gui::SimulatorLogWidget* _logWidget;
    rw::common::Ptr<rw::graphics::GroupNode> _pluginGraphics;
};
//! @}
} /* namespace plugins */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_PLUGINS_CONTACTDETECTIONTESTPLUGIN_HPP_ */
