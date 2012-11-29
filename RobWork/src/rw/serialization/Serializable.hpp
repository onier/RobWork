#ifndef RW_COMMON_SERIALIZABLE_HPP
#define RW_COMMON_SERIALIZABLE_HPP

#include <cstdlib>
#include <cmath>
#include <string>

#include <boost/any.hpp>
#include <cstdio>
#include <fstream>
#include <rw/common/macros.hpp>
#include <boost/any.hpp>
#include "Archive.hpp"

class InputArchive;
class OutputArchive;

/**
 * @brief interface for defining if a class is serializable
 */
class Serializable {
public:
	virtual ~Serializable(){};
//protected:
	//friend Archive::Access;
	virtual void read(class InputArchive& iarchive, const std::string& id) = 0;
	virtual void write(class OutputArchive& iarchive, const std::string& id) = 0;
};

#endif
