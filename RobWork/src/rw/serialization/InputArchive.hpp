#ifndef RW_COMMON_INPUTARCHIVE_HPP
#define RW_COMMON_INPUTARCHIVE_HPP

#include "Archive.hpp"

#include "Serializable.hpp"
#include <boost/cstdint.hpp>
#include <boost/type_traits.hpp>

/**
 * @brief an archive interface for reading from a serialized class.
 */
class InputArchive: public Archive {
public:
	//! @brief constructor
    InputArchive(){};

    //! @brief open an inputstream for reading
    virtual void open(std::istream& ifs) = 0;

    // utils to handle arrays
    virtual void readEnterScope(const std::string& id) = 0;
    virtual void readLeaveScope(const std::string& id) = 0;

    // reading primitives to archive
    virtual void read(bool& val, const std::string& id) = 0;
    virtual void read(boost::int8_t& val, const std::string& id) = 0;
    virtual void read(boost::uint8_t& val, const std::string& id) = 0;
    virtual void read(boost::int16_t& val, const std::string& id) = 0;
    virtual void read(boost::uint16_t& val, const std::string& id) = 0;
    virtual void read(boost::int32_t& val, const std::string& id) = 0;
    virtual void read(boost::uint32_t& val, const std::string& id) = 0;
    virtual void read(boost::int64_t& val, const std::string& id) = 0;
    virtual void read(boost::uint64_t& val, const std::string& id) = 0;
    virtual void read(float& val, const std::string& id) = 0;
    virtual void read(double& val, const std::string& id) = 0;
    virtual void read(std::string& val, const std::string& id) = 0;

    virtual void read(std::vector<bool>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int8_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint8_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int16_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint16_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int32_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint32_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::int64_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<boost::uint64_t>& val, const std::string& id) = 0;
    virtual void read(std::vector<float>& val, const std::string& id) = 0;
    virtual void read(std::vector<double>& val, const std::string& id) = 0;
    virtual void read(std::vector<std::string>& val, const std::string& id) = 0;




    // convienience wrappers for reading primitives
    bool readBool(const std::string& id){ bool b; read(b,id); return b;};
    int readInt(const std::string& id){ int b; read(b,id); return b;};
    unsigned int readUInt(const std::string& id){ unsigned int b; read(b,id); return b;};

    boost::int8_t readInt8(const std::string& id){ boost::int64_t b; read(b,id); return b;};
    boost::uint8_t readUInt8(const std::string& id){ boost::uint64_t b; read(b,id); return b;};

    boost::int64_t readInt64(const std::string& id){ boost::int64_t b; read(b,id); return b;};
    boost::uint64_t readUInt64(const std::string& id){ boost::uint64_t b; read(b,id); return b;};
    double readDouble(const std::string& id) { double b; read(b,id); return b;};
    std::string readString(const std::string& id) { std::string b; read(b,id); return b;};


/*
    std::string readString(const std::string& id){
        std::string result;
        read(result, id);
        return result;
    }
*/

    //
    template<class T>
    void read(T& object, const std::string& id){
    	// first test if T is any of the primitives
    	if( boost::is_const<T>::value_type ){
    		RW_THROW("type T cannot be of type const!");
    	} else if( boost::is_reference<T>::value_type ){
    		RW_THROW("type T cannot be of type reference!");
    	} else if( boost::is_floating_point<T>::value_type || boost::is_integral<T>::value_type){
    		T* val = new T;
    		read(*val,id);
    		return val;
    	}

        // test if T inherit from Serializable
        if(boost::is_base_of<Serializable, T>::value_type) {
        	object.read(*this, id);
        } else {
        	// the T does not

        }

        /*
        T* data = Archive::Access::load<T>(*this, id);

        if(data==NULL){
            // try read with generic call
            boost::any anyval = read(id);
            data = boost::any_cast<T>(&anyval);
            if(data == NULL ){
                RW_THROW("No leader for data! id:"<< id);
            }
        }
        readLeaveScope(id);
        return data;
        */
    }


protected:

    /**
     * @brief this is the fallback call in case a class does not implement load/save funtionality
     * then the implementation on the OutputArchive might be able to save the class.
     *
     * @note unfortunately this require the implementing InputArchive to use downcast methods...
     */
    virtual boost::any read(const std::string& id){ RW_THROW("No handler for this type of data!");};
};
#endif
