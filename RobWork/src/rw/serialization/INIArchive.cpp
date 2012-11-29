#include "INIArchive.hpp"

#include <boost/filesystem.hpp>

void INIArchive::open(const std::string& filename){
	if( !boost::filesystem::exists(filename.c_str()) ){
		//create the file

	}

	_fstr = new std::fstream(filename.c_str());
	_ofs = _fstr;
	_ifs = _fstr;
	_isopen =  _fstr->is_open();
}



 void INIArchive::read(bool& val, const std::string& id){
	int res = readInt(id);
	if(res==0)
		val = false;
	else
		val = true;
 }

 void INIArchive::read(std::string& val, const std::string& id){
	 _ifs->getline(_line,500);
	 std::pair<std::string,std::string> valname = getNameValue();
	 //std::cout << valname.first << "  " << valname.second << std::endl;
	 val = valname.second;
 }

 void INIArchive::read(std::vector<std::string>& val, const std::string& id){
		_ifs->getline(_line,500);
		std::pair<std::string,std::string> valname = getNameValue();
		if(id!=valname.first)
			RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
	    // read from array
		boost::split(val, valname.second, boost::is_any_of("\t "));
 }
