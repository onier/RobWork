SET(FILES_CPP
  DynamicLibraryLoader.cpp
  DynamicLibraryLoaderBase.cpp
  PluginFactory.cpp
  PluginFactoryBase.cpp
  PluginRepository.cpp
)

SET(FILES_HPP
  DynamicLibraryLoader.hpp
  DynamicLibraryLoaderBase.hpp
  PluginFactory.hpp
  PluginFactoryBase.hpp
  PluginRepository.hpp
  FactoryMacro.hpp
)

SOURCE_GROUP(proximity FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(RW_LIB_FILES_CPP ${RW_LIB_FILES_CPP} ${SRC_FILES_CPP} PARENT_SCOPE)
SET(RW_LIB_FILES_HPP ${RW_LIB_FILES_HPP} ${SRC_FILES_HPP} PARENT_SCOPE)

#INSTALL(FILES ${PROXIMITY_FILES_HPP} DESTINATION "include/rw/proximity/")
