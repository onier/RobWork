#include "PluginRepository.hpp"

#include <rw/common/IOUtil.hpp>
#include <rw/common/os.hpp>
#include "DynamicLibraryLoader.hpp"

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::plugin;


//PluginRepository PluginRepository::_repository;


void PluginRepository::load(const std::string& filename) {
    PluginFactoryBasePtr constructor; 
    try 
    {
        DynamicLibraryLoader<PluginFactoryBase>* loader = new DynamicLibraryLoader<PluginFactoryBase>(filename);
        constructor = ownedPtr(loader->get());
    } catch (const Exception& exp) 
    {
        RW_THROW("Unable to load plugin: "<<filename<<". Failed with message: "<<exp.getMessage().getText());
    }

    if (constructor != NULL) 
    {
        const std::string id = constructor->identifier();
        if (_str2constructorMap.find(id) == _str2constructorMap.end()) {
            _str2constructorMap[id] = constructor;
            Log::debugLog()<<"Loaded Plugin "<<id<<std::endl;
        } else {
            RW_THROW("A Plugin with identifier "<<id<<" has already been loaded!");
        }
    } else {
        RW_THROW("Unable to load plugin: "<<filename);
    }
}

void PluginRepository::loadFilesInFolder(const std::string& path, bool searchSubFolders) {
    
    std::vector<std::string> files = IOUtil::getFilesInFolder(path, true, "*."+OS::getDLLExtension());
//    std::vector<std::string> files = IOUtil::getFilesInFolder(path, true, "*.dll");

    BOOST_FOREACH(std::string str, files) {
        std::cout<<"DLL File = "<<str<<std::endl;
        load(str);        
    }
}

void PluginRepository::addListener(boost::function<void(void)>& listener) {
    _listeners.push_back(listener);
}   




/*std::vector<PluginFactoryBasePtr> PluginRepository::getPlugins(PluginType type) const {
    std::vector<PluginFactoryBasePtr> result;
    for (std::map<std::string, PluginFactoryBasePtr>::const_iterator it = _str2constructorMap.begin(); it != _str2constructorMap.end(); ++it) {
        if ((*it).second->getType() == type) {
            result.push_back((*it).second);
        }
    }
    return result;
}
*/

const std::map<std::string, PluginFactoryBasePtr>& PluginRepository::getAllPlugins() const {
    return _str2constructorMap;
}


std::map<std::string, PluginFactoryBasePtr>& PluginRepository::getAllPlugins() {
    return _str2constructorMap;
}



/*
PluginRepository& PluginRepository::instance() {
    static PluginRepository repository;    
    return repository;
}
*/
