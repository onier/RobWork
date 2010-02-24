#ifndef RW_PLUGIN_PLUGINFACTORY_HPP
#define RW_PLUGIN_PLUGINFACTORY_HPP

#include "PluginFactoryBase.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/common/macros.hpp>


namespace rw {
namespace plugin {



    /** @addtogroup plugin */
    /*@{*/




template <class T>
class PluginFactory: public PluginFactoryBase {
public:
    PluginFactory(const std::string& identifier, PluginType type):
        PluginFactoryBase(identifier, type)
    {

    }

    virtual rw::common::Ptr<T> make() {
        RW_THROW2(5000, "PluginFactory<T>::make() is not implemented");
    };

    virtual rw::common::Ptr<T> make(const std::string&) {
        RW_THROW2(5001, "PluginFactory<T>::make(const std::string&) is not implemented");
    };

};

/** @} */

} //end namespace plugin
} //end namespace rw


#endif //#ifndef RW_PLUGIN_PLUGINFACTORY_HPP
