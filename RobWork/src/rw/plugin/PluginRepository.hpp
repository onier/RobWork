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

/**
 * @brief The PluginRepository provides a container load methods for plugins
 *
 * The PluginRepository contain a collection of pointers rw::plugin::PluginFactoryBase instances
 * which can be used to create plugins. 
 *
 * The PluginRepository has a number of different methods to load plugins and methods for 
 * searching for a plugins either based on identifiers or by type.
 */
class PluginRepository
{
public:
    /**
     * @brief Constructs an empty repository
     */
    PluginRepository() {};

    /**
     * @brief Destructor
     */
    ~PluginRepository() {}

    /**
     * @brief Loads in a PluginFactoryBase from a file.
     * 
     * If the file could not be loaded or does not contain an object of type
     * PluginFactoryBase a rw::common::Exception is thrown.
     *
     * @param filename [in] File to load
     */
    void load(const std::string& filename);

    /**
     * @brief Attempts to load all dll files in folder
     *
     * If finding a dll file which can not be loaded or does not contain an object of type
     * PluginFactoryBase a rw::common::Exception is thrown.
     * 
     * @param path [in] Path from which to attempt for load plugins
     * @param searchRecursively [in] True to search recursively into subfolders.
     */
    void loadFilesInFolder(const std::string& path, bool searchSubFolders);

    /**
     * @brief Add a listener which should be informed when new plugins are loaded
     *
     * @param listener [in] The function to call for notification
     */
    void addListener(boost::function<void(void)>& listener);

    /**
     * @brief Returns map in which keys are the identifiers of loaded plugins factories and the
     * value is the plugin factory.
     *
     * @return Const reference to std::map with identifier and PluginFactoryBasePtr.
     */
    const std::map<std::string, PluginFactoryBasePtr>& getAllPlugins() const;

    /**
     * @brief Returns map in which keys are the identifiers of loaded plugins factories and the
     * value is the plugin factory.
     *
     * @return Reference to std::map with identifier and PluginFactoryBasePtr.
     */
    std::map<std::string, PluginFactoryBasePtr>& getAllPlugins();

    /**
     * @brief Returns all rw::common::PluginFactory<T> instances which matches the template argument T
     *
     * @return List of all factories matching T 
     */
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


    //static PluginRepository& instance();

private:


    std::map<std::string, PluginFactoryBasePtr> _str2constructorMap;

    std::vector<boost::function<void(void)> > _listeners;
};

/** @} */


} //end namespace plugin
} //end namespace rw

#endif //#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP
