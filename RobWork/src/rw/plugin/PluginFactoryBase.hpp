#ifndef RW_PLUGIN_PLUGINFACTORYBASE_HPP
#define RW_PLUGIN_PLUGINFACTORYBASE_HPP

#include <rw/common/Ptr.hpp>
#include <string>

namespace rw {
namespace plugin {

enum PluginType { DEVICE = 1, JOINT, FRAME, INVKIN_SOLVER, TRAJECTORY, USER }; 


class PluginFactoryBase
{
public:      
    PluginFactoryBase(const std::string& identifier, PluginType type);
    virtual ~PluginFactoryBase(void);
     
    virtual std::string identifier() const;

    virtual PluginType getType() const;

private:
    std::string _identifier;
    PluginType _type;
};

typedef rw::common::Ptr<PluginFactoryBase> PluginFactoryBasePtr;

} //end namespace plugin
} //end namespace rw


#endif //#ifndef RW_PLUGIN_PLUGINFACTORYBASE_HPP