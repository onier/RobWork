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

#include "ContactTest.hpp"
#include "ContactPQPBoxPlaneTest.hpp"

#include <rwsim/contacts/ContactDetector.hpp>

using namespace rw::common;
using namespace rwsim::contacts;
using namespace rwsimlibs::test;

ContactTest::ContactTest() {
}

ContactTest::~ContactTest() {
}

PropertyMap::Ptr ContactTest::getDefaultParameters() const {
	return ownedPtr(new PropertyMap());
}

ContactDetector::Ptr ContactTest::getDetector(const PropertyMap& map) {
	return ContactDetector::makeDefault(getWC(map),map);
}

ContactTest::Factory::Factory():
	ExtensionPoint<ContactTest>("rwsimlibs.test.ContactTest", "ContactTest extension point.")
{
}

std::vector<std::string> ContactTest::Factory::getTests() {
    std::vector<std::string> ids;
    ContactTest::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    ids.push_back("ContactPQPBoxPlaneTest");
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        ids.push_back( ext.getProperties().get("testID",ext.name) );
    }
    return ids;
}

bool ContactTest::Factory::hasTest(const std::string& test) {
    if(test == "ContactPQPBoxPlaneTest")
        return true;
    ContactTest::Factory ep;
    std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
    BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("testID",ext.name) == test)
            return true;
    }
    return false;
}

ContactTest::Ptr ContactTest::Factory::getTest(const std::string& test) {
    if( test == "ContactPQPBoxPlaneTest")
        return new ContactPQPBoxPlaneTest();
    ContactTest::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr& ext, exts){
		const PropertyMap& props = ext->getProperties();
		if(props.get("testID",ext->getName() ) == test){
			return ext->getObject().cast<ContactTest>();
		}
	}
	return NULL;
}
