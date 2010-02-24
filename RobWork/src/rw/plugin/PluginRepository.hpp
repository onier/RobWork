#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP
#define RW_PLUGIN_PLUGINREPOSITORY_HPP


#include "PluginFactory.hpp"

#include <boost/function.hpp>
#include <map>
#include <vector>




namespace rw {
namespace plugin {



/** @addtogroup plugin */
/*@{*/


class PluginRepository
{
public:
    PluginRepository() {};

    ~PluginRepository() {}

    void add(const std::string& filename);

    void addFilesInFolder(const std::string& path);

    void addListener(boost::function<void(void)>& listener);

    std::vector<PluginFactoryBasePtr> getPlugins(PluginType type) const;

    const std::map<std::string, PluginFactoryBasePtr>& getAllPlugins() const;

    std::map<std::string, PluginFactoryBasePtr>& getAllPlugins();


    template <class T>
    std::vector<rw::common::Ptr<PluginFactory<T> > > getPlugins() {
        std::vector<rw::common::Ptr<PluginFactory<T> > > result;
        for (std::map<std::string, PluginFactoryBasePtr>::iterator it = _str2constructorMap.begin(); it != _str2constructorMap.end(); ++it) {
            rw::common::Ptr<PluginFactory<T> > factory = (*it).second.cast<PluginFactory<T> >();
            if (factory != NULL)
                result.push_back(factory);

        }
        return result;
    }


    static PluginRepository& instance();

private:


    std::map<std::string, PluginFactoryBasePtr> _str2constructorMap;

    std::vector<boost::function<void(void)> > _listeners;
};

/** @} */


} //end namespace plugin
} //end namespace rw

#endif //#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP
