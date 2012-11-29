#ifndef RW_COMMON_OUTPUTARCHIVE_HPP
#define RW_COMMON_OUTPUTARCHIVE_HPP

#include "Archive.hpp"
#include "Serializable.hpp"
#include <boost/cstdint.hpp>
#include <boost/type_traits.hpp>


/**
 * @brief serializable objects can be written to an output archive.
 *
 * This class define an interface for serializing data.
 */
class OutputArchive : public Archive {
public:
	//! @brief destructor
    virtual ~OutputArchive(){};

	//! @copydoc Archive::open
    virtual void open(std::ostream& ofs) = 0;

    // utils to handle arrays
    virtual void writeEnterScope(const std::string& id) = 0;
    virtual void writeLeaveScope(const std::string& id) = 0;

    // writing primitives to archive
    virtual void write(bool val, const std::string& id) = 0;
    virtual void write(boost::int8_t val, const std::string& id) = 0;
    virtual void write(boost::uint8_t val, const std::string& id) = 0;
    virtual void write(boost::int16_t val, const std::string& id) = 0;
    virtual void write(boost::uint16_t val, const std::string& id) = 0;
    virtual void write(boost::int32_t val, const std::string& id) = 0;
    virtual void write(boost::uint32_t val, const std::string& id) = 0;
    virtual void write(boost::int64_t val, const std::string& id) = 0;
    virtual void write(boost::uint64_t val, const std::string& id) = 0;
    virtual void write(float val, const std::string& id) = 0;
    virtual void write(double val, const std::string& id) = 0;
    virtual void write(const std::string& val, const std::string& id) = 0;

    virtual void write(const std::vector<bool>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::int8_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::uint8_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::int16_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::uint16_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::int32_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::uint32_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::int64_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<boost::uint64_t>& val, const std::string& id) = 0;
    virtual void write(const std::vector<float>& val, const std::string& id) = 0;
    virtual void write(const std::vector<double>& val, const std::string& id) = 0;
    virtual void write(const std::vector<std::string>& val, const std::string& id) = 0;


    // a pointer value

/*
    // what about list/ types?
    template<class T>
    void write(std::vector<T>& data, const std::string& id){
        enterArray(id);

        leaveArray(id);
    }

    template<class T>
    void write(std::list<T>& data, const std::string& id){

    }

    template<class T>
    void write(T* array, int nrElems, const std::string& id){

    }
*/
    // now for the complex types, these needs to implement save/load functionality
    template<class T>
    void write(const T& data, const std::string& id){
        // the data method must have an implementation of load/save and if not then we try the generic write
        // method which could provide a solution by the implementation itself
    	if( boost::is_const<T>::value_type ){
    		RW_THROW("type T cannot be of type const!");
    	} else if( boost::is_reference<T>::value_type ){
    		RW_THROW("type T cannot be of type reference!");
    	} else if( boost::is_floating_point<T>::value_type || boost::is_integral<T>::value_type){
    		T* val = new T;
    		write(*val, id);
    		return val;
    	}

        // test if T inherit from Serializable
        if(boost::is_base_of<Serializable, T>::value_type) {
        	object.write(*this, id);
        } else {
        	// the T does not

        }

/*
        try {
            Archive::Access::save<T>(data, *this, id);
        } catch (...){
            // we fall back to this call
            boost::any adata(data);
            write(adata, id);
        }
        */
    }

    /**
     * @brief this is the fallback call in case a class does not implement load/save funtionality
     * then the implementation on the OutputArchive might be able to save the class.
     *
     * @note unfortunately this require the implementing InputArchive to use downcast methods...
     */
    virtual void write(boost::any& anydata, const std::string& id){ RW_THROW("No handler for this type of data!");};
};
#endif
