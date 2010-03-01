#ifndef RW_PLUGIN_PLUGINFACTORYBASE_HPP
#define RW_PLUGIN_PLUGINFACTORYBASE_HPP

#include <rw/common/Ptr.hpp>
#include <string>

namespace rw {
namespace plugin {



/** @addtogroup plugin */
/*@{*/

/**
 * @brief Base class for PluginFactory.
 *
 * When a PluginFactory is loaded it is stored as a PluginFactoryBase within the rw::plugin::PluginRepository.
 * 
 */
class PluginFactoryBase
{
public:
    /**
     * @brief Constructor 
     *
     * @param [in] identifier used to identify the plugin
     */
    PluginFactoryBase(const std::string& identifier);

    /**
     * @brief Destructor
     */
    virtual ~PluginFactoryBase(void);

    /** 
     * @brief Returns identifier associated with the PluginFactory
     * @return Identifier for the plugin
     */
    virtual std::string identifier() const;

private:
    std::string _identifier;
};

/**
 * @brief Definition of rw::common::Ptr to a PluginFactoryBase
 */
typedef rw::common::Ptr<PluginFactoryBase> PluginFactoryBasePtr;

/** @} */

} //end namespace plugin
} //end namespace rw


#endif //#ifndef RW_PLUGIN_PLUGINFACTORYBASE_HPP
