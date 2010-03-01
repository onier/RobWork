#include "PluginFactoryBase.hpp"

using namespace rw::plugin;

PluginFactoryBase::PluginFactoryBase(const std::string& identifier):
_identifier(identifier)
{
}

PluginFactoryBase::~PluginFactoryBase(void)
{
}

     
std::string PluginFactoryBase::identifier() const {
    return _identifier;
}

