#include "PluginFactoryBase.hpp"

using namespace rw::plugin;

PluginFactoryBase::PluginFactoryBase(const std::string& identifier, PluginType type):
_identifier(identifier),
_type(type)
{
}

PluginFactoryBase::~PluginFactoryBase(void)
{
}

     
std::string PluginFactoryBase::identifier() const {
    return _identifier;
}

PluginType PluginFactoryBase::getType() const {
    return _type;
}
