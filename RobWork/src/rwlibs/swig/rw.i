%module rw

%{
#include <RobWorkConfig.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#if defined (SWIGLUA)
#include <rwlibs/swig/lua/Lua.hpp>
#endif

using namespace rwlibs::swig;
using rw::math::Metric;
using namespace rw::math;
using rw::trajectory::Interpolator;
using rw::trajectory::Blend;
using rw::trajectory::Path;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rw::trajectory::InterpolatorTrajectory;
using rw::pathplanning::PathPlanner;
%}

%pragma(java) jniclassclassmodifiers="class"
#if defined (SWIGJAVA)
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
SWIG_JAVABODY_TYPEWRAPPER(public, public, public, SWIGTYPE)
#endif

#if defined(SWIGPYTHON)
%feature("doxygen:ignore:beginPythonOnly", range="end:endPythonOnly", contents="parse");
%feature("doxygen:ignore:beginJavaOnly", range="end:endJavaOnly");
#elif defined(SWIGJAVA)
%feature("doxygen:ignore:beginPythonOnly", range="end:endPythonOnly");
%feature("doxygen:ignore:beginJavaOnly", range="end:endJavaOnly", contents="parse");
#else
%feature("doxygen:ignore:beginPythonOnly", range="end:endPythonOnly");
%feature("doxygen:ignore:beginJavaOnly", range="end:endJavaOnly");
#endif

%include <std_string.i>
%include <std_vector.i>
//%include <shared_ptr.i>

#if !defined(SWIGJAVA)
%include "carrays.i"
%array_class(double, doubleArray);
#else
%include "arrays_java.i";
#endif

#if defined(SWIGJAVA)
	%rename(multiply) operator*;
	%rename(divide) operator/;
	%rename(equals) operator==;
	%rename(negate) operator-() const;
	%rename(subtract) operator-;
	%rename(add) operator+;
#endif

#if (defined(SWIGPYTHON) || defined(SWIGLUA))
%feature("flatnested");
#endif

%include <stl.i>

/*
%define COVARIANT(DERIVED, BASE)
%types(rw::common::Ptr<DERIVED> = rw::common::Ptr<BASE>) %{
        *newmemory = SWIG_CAST_NEW_MEMORY;
        return (void*) new rw::common::Ptr<BASE>(*(rw::common::Ptr<DERIVED>*)$from);
%}
%enddef

%COVARIANT(Apple, Fruit)
*/

void writelog(const std::string& msg);

/********************************************
 * General utility functions
 ********************************************/

%inline %{
    void sleep(double t){
        ::rw::common::TimerUtil::sleepMs( (int) (t*1000) );
    }
    double time(){
        return ::rw::common::TimerUtil::currentTime( );
    }
    long long timeMs(){
        return ::rw::common::TimerUtil::currentTimeMs( );
    }
    void info(const std::string& msg){
        ::rw::common::Log::infoLog() << msg;
    }
    void debug(const std::string& msg){
        ::rw::common::Log::debugLog() << msg;
    }
    void warn(const std::string& msg){
        ::rw::common::Log::warningLog() << msg;
    }
    void error(const std::string& msg){
        ::rw::common::Log::errorLog() << msg;
    }
%}



/********************************************
 * Constants
 ********************************************/

%constant double Pi = rw::math::Pi;
%constant double Inch2Meter = rw::math::Inch2Meter;
%constant double Meter2Inch = rw::math::Meter2Inch;
%constant double Deg2Rad = rw::math::Deg2Rad;
%constant double Rad2Deg = rw::math::Rad2Deg;

/********************************************
 * STL vectors (primitive types)
 ********************************************/
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
	%extend std::vector<std::string> { char *__str__() { return printCString(*$self); } }
#endif

namespace std {
	%template(StringVector) std::vector<string>;
	%template(DoubleVector) std::vector<double>;
	%template(IntVector) std::vector<int>;
};

/********************************************
 * COMMON
 ********************************************/

namespace rw { namespace common {

/**
  * @brief The Ptr type represents a smart pointer that can take ownership
  * of the underlying object.
  *
  * If the underlying object is owned by the smart pointer, it is destructed
  * when there is no more smart pointers pointing to the object.
  */
template<class T> class Ptr
{
public:
    //! @brief Empty smart pointer (Null).
    Ptr();

    /**
      * @brief Construct new smart pointer that takes ownership of the
      * underlying object.
      *
      * @param ptr The object to take ownership of.
      */
    Ptr(T* ptr);

    /**
      * @brief Construct smart pointer from other smart pointer.
      *
      * @param p the other (compatible) smart pointer.
      */
    template <class S>
    Ptr(const Ptr<S>& p);

    bool isShared();

    /**
      * @brief Check if smart pointer is null.
      *
      * @return true if smart pointer is null.
      */
    bool isNull();

    template<class A>
    bool operator==(const rw::common::Ptr<A>& p) const;
#if defined(SWIGJAVA)
	%rename(dereference) get;
#endif
    T* get() const;

    T *operator->() const;
};

#if defined(SWIGPYTHON)
 %pythonprepend ownedPtr(T*) %{
  args[0].thisown = 0
 %}
#endif
/**
  * @brief Construct a smart pointer that takes ownership over a raw object \b ptr.
  *
  * @param ptr the object to take ownership of.
  */
template <class T>
Ptr<T> ownedPtr(T* ptr);

}}

%define OWNEDPTR(ownedPtr_type)
namespace rw { namespace common {
#if defined(SWIGJAVA)
 %typemap (in) ownedPtr_type* %{
  jclass objcls = jenv->GetObjectClass(jarg1_);
  const jfieldID memField = jenv->GetFieldID(objcls, "swigCMemOwn", "Z");
  jenv->SetBooleanField(jarg1_, memField, (jboolean)false);
  $1 = *(std::remove_const<ownedPtr_type>::type **)&jarg1;
 %}
#elif defined(SWIGLUA)
 %typemap (in,checkfn="SWIG_isptrtype") ownedPtr_type* %{
  if (!SWIG_IsOK(SWIG_ConvertPtr(L,$input,(void**)&$1,$descriptor,SWIG_POINTER_DISOWN))){
    SWIG_fail_ptr("$symname",$input,$descriptor);
  }
 %}
#endif
 %template (ownedPtr) ownedPtr<ownedPtr_type>;
#if (defined(SWIGLUA) || defined(SWIGJAVA))
 %clear ownedPtr_type*;
#endif
}}
%enddef

/** @addtogroup swig */
/* @{ */

//! @copydoc rw::common::PropertyMap
class PropertyMap
{
public: 
	//! @copydoc rw::common::PropertyMap::PropertyMap
	PropertyMap();
	//! @copydoc rw::common::PropertyMap::has
	bool has(const std::string& identifier) const;
    //! @copydoc rw::common::PropertyMap::size
    size_t size() const;
    //! @copydoc rw::common::PropertyMap::empty 
    bool empty() const;
    //! @copydoc rw::common::PropertyMap::erase
    bool erase(const std::string& identifier);
    
	%extend {
		
		bool getBool(const std::string& id){ return $self->get<bool>(id); }
		void setBool(const std::string& id, bool val){  $self->set<bool>(id,val); }
		void set(const std::string& id, bool val){  $self->set<bool>(id,val); }

		std::string& getString(const std::string& id){ return $self->get<std::string>(id); }
		void setString(const std::string& id, std::string val){  $self->set<std::string>(id,val); }
		void set(const std::string& id, std::string val){  $self->set<std::string>(id,val); }
		
		std::vector<std::string>& getStringList(const std::string& id){ return $self->get<std::vector<std::string> >(id); }
		void setStringList(const std::string& id, std::vector<std::string> val){ $self->set<std::vector<std::string> >(id,val); }
		void set(const std::string& id, std::vector<std::string> val){ $self->set<std::vector<std::string> >(id,val); }
		
		rw::math::Q& getQ(const std::string& id){ return $self->get<rw::math::Q>(id); }
		void setQ(const std::string& id, rw::math::Q q){ $self->set<rw::math::Q>(id, q); }
		void set(const std::string& id, rw::math::Q q){ $self->set<rw::math::Q>(id, q); }

		rw::math::Pose6D<double>& getPose(const std::string& id){ return $self->get<rw::math::Pose6D<double> >(id); }
		void setPose6D(const std::string& id, rw::math::Pose6D<double> p){  $self->set<rw::math::Pose6D<double> >(id, p); }
		void set(const std::string& id, rw::math::Pose6D<double> p){  $self->set<rw::math::Pose6D<double> >(id, p); }
		
		rw::math::Vector3D<double>& getVector3(const std::string& id){ return $self->get<rw::math::Vector3D<double> >(id); }
		void setVector3(const std::string& id, rw::math::Vector3D<double> p){  $self->set<rw::math::Vector3D<double> >(id, p); }
		void set(const std::string& id, rw::math::Vector3D<double> p){  $self->set<rw::math::Vector3D<double> >(id, p); }

		rw::math::Transform3D<double> & getTransform3(const std::string& id){ return $self->get<rw::math::Transform3D<double> >(id); }
		void setTransform3(const std::string& id, rw::math::Transform3D<double>  p){  $self->set<rw::math::Transform3D<double> >(id, p); }
		void set(const std::string& id, rw::math::Transform3D<double>  p){  $self->set<rw::math::Transform3D<double> >(id, p); }

		PropertyMap& getMap(const std::string& id){ return $self->get<PropertyMap>(id); }
		void setMap(const std::string& id, PropertyMap p){  $self->set<PropertyMap>(id, p); }
		void set(const std::string& id, PropertyMap p){  $self->set<PropertyMap>(id, p); }

		void load(const std::string& filename){ *($self) = rw::loaders::DOMPropertyMapLoader::load(filename); }
		void save(const std::string& filename){ rw::loaders::DOMPropertyMapSaver::save( *($self), filename ); }
		
	}    
 
};
%template (PropertyMapPtr) rw::common::Ptr<PropertyMap>;
OWNEDPTR(PropertyMap)

/**
 * \brief Provides basic log functionality.
 *
 * The Log class owns a number of LogWriters in a static map, which can be accessed
 * using a string identifier. All logs are global.
 *
 * By default the Log class contains a Debug, Info, Warning and Error log. These can be accessed
 * statically as:
 * \code
 * Log::debugLog() <<  "This is an debug message";
 * Log::infoLog() << "This is an info message";
 * Log::warnLog() << "This is an error message";
 * Log::errorLog() << "This is an error message";
 * \endcode
 * or on the log instance
 * \code
 * Log &log = Log::log();
 * log.debug() <<  "This is an debug message";
 * log.info() << "This is an info message";
 * log.warn() << "This is an error message";
 * log.error() << "This is an error message";
 * \endcode
 * or using one one the RW_LOG, RW_LOGLINE or RW_LOG2 macros, e.g.
 * \code
 * RW_LOG_INFO("The value of x is "<<x);
 * RW_LOG_DEBUG("The value of x is "<<x);
 * RW_LOG_ERROR(Log::infoId(), "The value of x is "<<x);
 * \endcode
 *
 * You can control what logs are active both using a loglevel and by using a log mask.
 * The loglevel enables all logs with LogIndex lower or equal to the loglevel. As default
 * loglevel is LogIndex::info which means debug and all user logs are disabled. However,
 * logs can be individually enabled using log masks which will override loglevel setting.
 *
 * Notice that logmasks cannot disable logs that are below or equal to loglevel.
 *
 * change loglevel:
 * \code
 * Log::log().setLevel(Log::Debug);
 * \endcode
 *
 *
 */
class Log
{
public:
    //! @brief loglevel mask
	enum LogIndexMask {
		FatalMask=1, CriticalMask=2,
		ErrorMask=4, WarningMask=8,
		InfoMask=16, DebugMask=32,
		User1Mask=64, User2Mask=128,
		User3Mask=256, User4Mask=512,
		User5Mask=1024, User6Mask=2048,
		User7Mask=4096, User8Mask=8096,
		AllMask = 0xFFFF
	};



	/**
	 * @brief Indices for different logs. The loglevel will be Info as default. Everything below the
	 * loglevel is enabled.
	 */
	enum LogIndex {
		Fatal=0, Critical=1,
		Error=2, Warning=3,
		Info=4, Debug=5,
		User1=6, User2=7,
		User3=8, User4=9,
		User5=10, User6=11,
		User7=12, User8=13
	};

	/**
	 * @brief Convert a LogIndex to a mask.
	 *
	 * @param idx [in] the LogIndex.
	 * @return the mask enabling the given log level.
	 */
    static LogIndexMask toMask(LogIndex idx){
            LogIndexMask toMaskArr[] = {FatalMask, CriticalMask,
                                      ErrorMask, WarningMask,
                                                InfoMask, DebugMask,
                                                User1Mask, User2Mask,
                                                User3Mask, User4Mask,
                                                User5Mask, User6Mask,
                                                User7Mask, User8Mask,
                                                AllMask};
            return toMaskArr[idx];
        }

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the info loglevel
	 *
	 * @return info LogWriter
	 */
    static LogWriter& infoLog();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the warning loglevel
	 *
	 * @return warning LogWriter
	 */
    static LogWriter& warningLog();


	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the error loglevel
	 *
	 * @return error LogWriter
	 */
    static LogWriter& errorLog();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the debug loglevel
	 *
	 * @return debug LogWriter
	 */
    static LogWriter& debugLog();

	/**
	 * @brief returns the global log instance. Global in the sence
	 * of whatever is linked staticly together.
	 *
	 * @return a Log
	 */
    static rw::common::Ptr<Log> getInstance();

    /**
     * @brief convenience function of getInstance
     *
     * @return a Log
     */
    static Log& log();

    /**
     * @brief sets the instance of the log class
     *
     * @param log [in] the log that will be used through the static log methods.
     */
    static void setLog(rw::common::Ptr<Log> log);

    //************************* Here follows the member interface

    /**
     * @brief constructor
     */
    Log();

    /**
     * @brief Destructor
     */
    virtual ~Log();

    /**
     * @brief set the loglevel. Any log with LogIndex equal to or less than
     * loglevel will be enabled. Any log above will be disabled unless an
     * enabled mask is specified for that log
     *
     * @param loglevel [in] the level
     */
    void setLevel(LogIndex loglevel);


    /**
     * @brief gets the log writer associated to logindex \b id
     *
     * @param id [in] logindex
     * @return log writer
     */
    rw::common::Ptr<LogWriter> getWriter(LogIndex id);

    /**
     * @brief Associates a LogWriter with the LogIndex \b id.
     *
     * SetWriter can either be used to redefine an existing log or to create a new
     * custom output.
     *
     * Example:
     * \code
     * Log::SetWriter(Log::User1, new LogStreamWriter(std::cout));
     * RW_LOG(Log::User1, "Message send to User log 1");
     * \endcode
     *
     * @param id [in] the LogIndex that the logwriter is associated with.
     * @param writer [in] LogWriter object to use
     */
    void setWriter(LogIndex id, rw::common::Ptr<LogWriter> writer);

    /**
     * @brief Associates a LogWriter with the logs specified with \b mask.
     *
     * SetWriter can either be used to redefine an existing log or to create a new
     * custom output.
     *
     * Example:
     * \code
     * log.setWriterForMask(Log::InfoMask | Log::DebugMask, new LogStreamWriter(std::cout));
     * RW_LOG(Log::Info, "Message send to User log 1");
     * \endcode
     *
     * @param mask [in] the LogIndexMask that the logwriter is associated with.
     * @param writer [in] LogWriter object to use
     */
	void setWriterForMask(int mask, rw::common::Ptr<LogWriter> writer);

    %extend {
		/**
		 * @brief Returns the LogWriter that is associated with LogIndex \b id
		 *
		 * If the \b id is unknown an exception is thrown.
		 *
		 * @param id [in] loglevel
		 * @return Reference to LogWriter object
		 */
		LogWriter& getLogWriter(LogIndex id) {
			return $self->get(id);
		}
    };

    /**
     * @brief Writes \b message to the log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] Log identifier
     * @param message [in] String message to write
     */
    void write(LogIndex id, const std::string& message);

    /**
     * @brief Writes \b message to the logwriter associated with LogIndex \b id
     *
     * If the \b id cannot be found an exception is thrown

     *
     * @param id [in] Log identifier
     * @param message [in] Message to write
     */
    void write(LogIndex id, const Message& message);

    /**
     * @brief Writes \b message followed by a '\\n' to the log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] Log identifier
     * @param message [in] Message to write
     */
    void writeln(LogIndex id, const std::string& message);

    /**
     * @brief Calls flush on the specified log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] loglevel
     */
    void flush(LogIndex id);


    /**
     * @brief Calls flush on all logs
     */
    void flushAll();


    /**
     * @brief Removes a log
     *
     * If the \b id cannot be found an exception is thrown
     *
     * @param id [in] Log identifier
     */
    void remove(LogIndex id);

	/**
	 * @brief Removes all log writers
	 */
	void removeAll();

	//! @brief Make indentation to make logs easier to read.
	void increaseTabLevel();

	//! @brief Decrease the indentation.
	void decreaseTabLevel();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the info loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& info();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the warning loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& warning();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the error loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& error();

	/**
	 * @brief convenience function for getting the LogWriter
	 * that is associated with the debug loglevel
	 *
	 * @return info LogWriter
	 */
    LogWriter& debug();

	/**
	 * @brief Enable log(s) given by log mask.
	 *
	 * @param mask [in] the mask for the logs to enable.
	 */
	void setEnable(int mask);

   /**
     * @brief Checks if the given LogIndex is enabled. This can be used to
     * determine if a certain log level will be displayed or not.
     *
     * @param idx [in] the level
     */
    bool isEnabled(LogIndex idx);

	/**
	 * @brief Disable log(s) given by log mask.
	 *
	 * @param mask [in] the mask for the logs to disable.
	 */
	void setDisable(int mask);
};

%template (LogPtr) rw::common::Ptr<Log>;
OWNEDPTR(Log)

/**
 * @brief Write interface for Logs
 *
 * LogWriter provides an output strategy for a log.
 */
class LogWriter
{
public:
    /**
     * @brief Descructor
     */
    virtual ~LogWriter();

    /**
     * @brief Flush method
     */
    void flush();

	/**
	 * @brief Set the tab level
	 */
	void setTabLevel(int tabLevel);


    /**
     * @brief Writes \b str to the log
     * @param str [in] message to write
     */
    void write(const std::string& str);

    /**
     * @brief Writes \b msg to the log
     *
     * Default behavior is to use write(const std::string&) for the standard
     * streaming representation of \b msg.
     *
     * @param msg [in] message to write
     */
    void write(const Message& msg);

    /**
     * @brief Writes \b str as a line
     *
     * By default writeln writes \b str followed by a '\\n'. However, logs
     * are free to implement a line change differently.
     */
    void writeln(const std::string& str);

    /**
     * @brief general stream operator
     */
    template< class T>
    LogWriter& operator<<( T t );

#if defined(SWIGPYTHON)
    /**
     * @brief specialized stream operator
     */
    LogWriter& operator<<(const std::string& str);

    /**
     * @brief Write Message to log.
     * @param msg [in] the message.
     * @return a reference to this LogWriter for chaining of stream operators.
     */
    LogWriter& operator<<(const Message& msg);


    /**
     * @brief specialized stream operator
     */
    LogWriter& operator<<(const char* str);

    /**
     * @brief Handle the std::endl and other stream functions.
     */
    LogWriter& operator<<(std::ostream& (*pf)(std::ostream&));
#endif

protected:
    LogWriter();

	virtual void doWrite(const std::string& message) = 0;
	virtual void doSetTabLevel(int tabLevel) = 0;
	virtual void doFlush() = 0;
};

%template (LogWriterPtr) rw::common::Ptr<LogWriter>;

/**
 * @brief Standard type for user messages of robwork.
 *
 * Messages are used for exception, warnings, and other things that are
 * reported to the user.
 *
 * Message values should contain the source file name and line number so
 * that it is easy to look up the place in the code responsible for the
 * generation of the message.
 *
 * RW_THROW and RW_WARN of macros.hpp have been introduced for the throwing
 * of exceptions and emission of warnings.
 */
class Message
{
public:
    /**
     * @brief Constructor
     *
     * Messages of RobWork are all annotated by the originating file name,
     * the originating line number, and a message text for the user.
     *
     * Supplying all the file, line, and message parameters can be a little
     * painfull, so a utility for creating messages is available from the
     * file macros.hpp.
     *
     * @param file [in] The source file name.
     *
     * @param line [in] The source file line number.
     *
     * @param message [in] A message for a user.
     */
    Message(const std::string& file,
            int line,
            const std::string& message = "");

    /**
     * @brief The name of source file within which the message was
     * constructed.
     *
     * @return The exception file name.
     */
    const std::string& getFile() const;

    /**
     * @brief The line number for the file at where the message was
     * constructed.
     *
     * @return The exception line number.
     */
    int getLine() const;

    /**
     * @brief The message text meant for the user.
     *
     * @return The message text.
     */
    const std::string& getText() const;

    /**
     * @brief Returns a full description of the message containing file, line number and message.
     */
    std::string getFullText() const;

    /**
     * @brief general stream operator
     */
    template< class T>
    Message& operator<<( T t );
};

class ThreadPool { 
public:
    ThreadPool(int threads = -1);
    virtual ~ThreadPool();
    unsigned int getNumberOfThreads() const;
    void stop();
    bool isStopping();
	unsigned int getQueueSize();
	void waitForEmptyQueue();
};

%template (MessagePtr) rw::common::Ptr<Message>;

%template (ThreadPoolPtr) rw::common::Ptr<ThreadPool>;
OWNEDPTR(ThreadPool)

class ThreadTask {
public:
	typedef enum TaskState {
    	INITIALIZATION,
    	IN_QUEUE,
    	EXECUTING,
    	CHILDREN,
    	IDLE,
    	POSTWORK,
    	DONE
    } TaskState;

	ThreadTask(rw::common::Ptr<ThreadTask> parent);
	ThreadTask(rw::common::Ptr<ThreadPool> pool);
	virtual ~ThreadTask();
	bool setThreadPool(rw::common::Ptr<ThreadPool> pool);
	rw::common::Ptr<ThreadPool> getThreadPool();
	//virtual void run();
	//virtual void subTaskDone(ThreadTask* subtask);
	//virtual void idle();
	//virtual void done();
    bool execute();
    TaskState wait(ThreadTask::TaskState previous);
    void waitUntilDone();
    TaskState getState();
    bool addSubTask(rw::common::Ptr<ThreadTask> subtask);
    std::vector<rw::common::Ptr<ThreadTask> > getSubTasks();
    void setKeepAlive(bool keepAlive);
    bool keepAlive();
};

%template (ThreadTaskPtr) rw::common::Ptr<ThreadTask>;
%template (ThreadTaskPtrVector) std::vector<rw::common::Ptr<ThreadTask> >;
OWNEDPTR(ThreadTask)

class Plugin {
protected:
	 Plugin(const std::string& id, const std::string& name, const std::string& version);
	 
public:
	const std::string& getId();
    const std::string& getName();
    const std::string& getVersion();
};

%template (PluginPtr) rw::common::Ptr<Plugin>;
%template (PluginPtrVector) std::vector<rw::common::Ptr<Plugin> >;

struct ExtensionDescriptor {
	ExtensionDescriptor();
	ExtensionDescriptor(const std::string& id_, const std::string& point_);

    std::string id,name,point;
    rw::common::PropertyMap props;

    //rw::common::PropertyMap& getProperties();
    const rw::common::PropertyMap& getProperties() const;
};

class Extension {
public:
	Extension(ExtensionDescriptor desc, Plugin* plugin);
	
	const std::string& getId();
	const std::string& getName();
};

%template (ExtensionPtr) rw::common::Ptr<Extension>;
%template (ExtensionPtrVector) std::vector<rw::common::Ptr<Extension> >;

class ExtensionRegistry {
public:
	ExtensionRegistry();
	static rw::common::Ptr<ExtensionRegistry> getInstance();
	std::vector<rw::common::Ptr<Extension> > getExtensions(const std::string& ext_point_id) const;
	std::vector<rw::common::Ptr<Plugin> > getPlugins() const;
};

%template (ExtensionRegistryPtr) rw::common::Ptr<ExtensionRegistry>;



/********************************************
 * RWLIBS SIMULATION
 ********************************************/

%nodefaultctor Simulator;
class Simulator {
public:
   struct UpdateInfo {
	   UpdateInfo();
	   UpdateInfo(double dt_step);

	   double dt;
	   double dt_prev;
	   double time;
	   bool rollback;
   };
   
   virtual ~Simulator();
   virtual void step(double dt) = 0;
   virtual void reset(State& state) = 0;
   virtual void init(State& state) = 0;
   virtual double getTime() = 0;
   virtual void setEnabled(Frame* frame, bool enabled) = 0;
   virtual State& getState() = 0;
   virtual PropertyMap& getPropertyMap() = 0;

};

%template (SimulatorPtr) rw::common::Ptr<Simulator>;


/********************************************
 * ROBWORK CLASS
 ********************************************/ 
 class RobWork {
 public:
	RobWork();
	
	static rw::common::Ptr<RobWork> getInstance();
	
	std::string getVersion() const;
	void initialize();
 };
 
 %template (RobWorkPtr) rw::common::Ptr<RobWork>;

/********************************************
 * GEOMETRY
 ********************************************/

class GeometryData {
public:
    typedef enum {PlainTriMesh,
                  IdxTriMesh,
                  SpherePrim, BoxPrim, OBBPrim, AABBPrim,
                  LinePrim, PointPrim, PyramidPrim, ConePrim,
                  TrianglePrim, CylinderPrim, PlanePrim, RayPrim,
                  UserType} GeometryType;

    virtual GeometryType getType() const = 0;
    virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true) = 0;
    static std::string toString(GeometryType type);
};

%template (GeometryDataPtr) rw::common::Ptr<GeometryData>;
OWNEDPTR(GeometryData);

class TriMesh: public GeometryData {
public:
    virtual Triangle getTriangle(size_t idx) const = 0;
    virtual void getTriangle(size_t idx, Triangle& dst) const = 0;
    virtual void getTriangle(size_t idx, Trianglef& dst) const = 0;
    virtual size_t getSize() const = 0;
    virtual size_t size() const = 0;
    virtual rw::common::Ptr<TriMesh> clone() const = 0;
    rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    //rw::common::Ptr<const TriMesh> getTriMesh(bool forceCopy=true) const;
};

%template (TriMeshPtr) rw::common::Ptr<TriMesh>;

class Primitive: public GeometryData {
public:
    rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    virtual rw::common::Ptr<TriMesh> createMesh(int resolution) const = 0;
    virtual rw::math::Q getParameters() const = 0;
};

class Sphere: public Primitive {
public:
    //! constructor
    Sphere(const rw::math::Q& initQ);
    Sphere(double radi):_radius(radi);
    double getRadius();
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    rw::math::Q getParameters() const;
    GeometryData::GeometryType getType() const;
};

class Box: public Primitive {
public:
    Box();
    Box(double x, double y, double z);
    Box(const rw::math::Q& initQ);
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    rw::math::Q getParameters() const;
    GeometryType getType() const;
};

class Cone: public Primitive {
public:
    Cone(const rw::math::Q& initQ);
    Cone(double height, double radiusTop, double radiusBot);
    double getHeight();
    double getTopRadius();
    double getBottomRadius();
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    rw::math::Q getParameters() const;
    GeometryType getType() const;
};

class Plane: public Primitive {
public:
    Plane(const rw::math::Q& q);
    Plane(const rw::math::Vector3D<double>& n, double d);
    Plane(const rw::math::Vector3D<double>& p1,
          const rw::math::Vector3D<double>& p2,
          const rw::math::Vector3D<double>& p3);

    rw::math::Vector3D<double>& normal();
    //const rw::math::Vector3D<double>& normal() const;
#if defined(SWIGJAVA)
	double d() const;
#else
    double& d();
#endif
    double distance(const rw::math::Vector3D<double>& point);
    double refit( std::vector<rw::math::Vector3D<double> >& data );
    rw::common::Ptr<TriMesh> createMesh(int resolution) const ;
    rw::math::Q getParameters() const;
    GeometryType getType() const;
};

/**
 * @brief Cylinder primitive.
 */
class Cylinder: public Primitive {
public:
    //! @brief Default constructor with no parameters.
	Cylinder();

	/**
	  * @brief Cylinder with parameters specified.
	  *
	  * @param radius the radius.
	  * @param height the height.
	  */

	Cylinder(float radius, float height);
	virtual ~Cylinder();
	double getRadius() const;
	double getHeight() const;
	
	/**
	  * @brief Create a mesh representation of the cylinder.
	  *
	  * @param resolution the resolution.
	  * @return the TriMesh.
	  */
	rw::common::Ptr<TriMesh> createMesh(int resolution) const;
	rw::math::Q getParameters() const;
	GeometryType getType() const;
};

class ConvexHull3D {
public:
    virtual void rebuild(const std::vector<rw::math::Vector3D<double> >& vertices) = 0;
    virtual bool isInside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual double getMinDistInside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual double getMinDistOutside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual rw::common::Ptr<PlainTriMeshN1> toTriMesh() = 0;
};


class Geometry {
public:
    Geometry(rw::common::Ptr<GeometryData> data, double scale=1.0);

    Geometry(rw::common::Ptr<GeometryData> data,
             const rw::math::Transform3D<double> & t3d,
             double scale=1.0);

    double getScale() const;
    void setScale(double scale);
    void setTransform(const rw::math::Transform3D<double> & t3d);
    const rw::math::Transform3D<double> & getTransform() const;
    rw::common::Ptr<GeometryData> getGeometryData();
#if !defined(SWIGJAVA)
    const rw::common::Ptr<GeometryData> getGeometryData() const;
#endif
    void setGeometryData(rw::common::Ptr<GeometryData> data);
    const std::string& getName() const;
    const std::string& getId() const;
    void setName(const std::string& name);
    void setId(const std::string& id);
    static rw::common::Ptr<Geometry> makeSphere(double radi);
    static rw::common::Ptr<Geometry> makeBox(double x, double y, double z);
    static rw::common::Ptr<Geometry> makeCone(double height, double radiusTop, double radiusBot);
    static rw::common::Ptr<Geometry> makeCylinder(float radius, float height);
};

%template (GeometryPtr) rw::common::Ptr<Geometry>;
%template (GeometryPtrVector) std::vector<rw::common::Ptr<Geometry> >;
OWNEDPTR(Geometry);

class STLFile {
public:
    static void save(const TriMesh& mesh, const std::string& filename);
    static rw::common::Ptr<PlainTriMeshN1f> load(const std::string& filename);
};

class PlainTriMeshN1
{
};

%template (PlainTriMeshN1Ptr) rw::common::Ptr<PlainTriMeshN1>;

class PlainTriMeshN1f
{
};

%template (PlainTriMeshN1fPtr) rw::common::Ptr<PlainTriMeshN1f>;

%nodefaultctor Triangle;
class Triangle
{
};

%nodefaultctor Trianglef;
class Trianglef
{
};


class PointCloud: public GeometryData {
	public:
        PointCloud();
        PointCloud(int w, int h);

/*
		GeometryType getType() const;
		size_t size() const;
		bool isOrdered();
	    std::vector<rw::math::Vector3D<float> >& getData();
	    const std::vector<rw::math::Vector3D<float> >& getData() const;
        const rw::math::Vector3D<float>& operator()(int x, int y) const;
	    rw::math::Vector3D<float>& operator()(int x, int y);
	    int getWidth() const;
        int getHeight() const;
	    void resize(int w, int h);

		static rw::common::Ptr<PointCloud> loadPCD( const std::string& filename );

        static void savePCD( const PointCloud& cloud,
                                                    const std::string& filename ,
                                                    const rw::math::Transform3D<float>& t3d =
	                                                            rw::math::Transform3D<float>::identity());
*/
	};


/********************************************
 * GRAPHICS
 ********************************************/

%template (WorkCellScenePtr) rw::common::Ptr<WorkCellScene>;
%template (DrawableNodePtr) rw::common::Ptr<DrawableNode>;
%template (DrawableNodePtrVector) std::vector<rw::common::Ptr<DrawableNode> >;

OWNEDPTR(WorkCellScene);

%constant int DNodePhysical = DrawableNode::Physical;
%constant int DNodeVirtual = DrawableNode::Virtual;
%constant int DNodeDrawableObject = DrawableNode::DrawableObject;
%constant int DNodeCollisionObject = DrawableNode::CollisionObject;
%nodefaultctor DrawableNode;
%nodefaultctor WorkCellScene;

class DrawableNode {
public:

    enum DrawType {
        //! Render in solid
        SOLID,
        //! Render in wireframe
        WIRE,
        //! Render both solid and wireframe
        OUTLINE
    };

    virtual void setHighlighted(bool b) = 0;

    virtual bool isHighlighted() const = 0;

    virtual void setDrawType(DrawType drawType) = 0;

    virtual void setTransparency(float alpha) = 0;

    virtual float getTransparency() = 0;

    bool isTransparent();

    virtual void setScale(float scale) = 0;

    virtual float getScale() const = 0;

    virtual void setVisible(bool enable) = 0;

    virtual bool isVisible() = 0;

    virtual const rw::math::Transform3D<double> & getTransform() const  = 0;

    virtual void setTransform(const rw::math::Transform3D<double> & t3d) = 0;

    virtual void setMask(unsigned int mask) = 0;
    virtual unsigned int getMask() const = 0;
};

class Model3D {
public:
    Model3D(const std::string& name);
    virtual ~Model3D();
    //struct Material;
    //struct MaterialFaces;
    //struct MaterialPolys;
    //struct Object3D;
    //typedef enum{
    //    AVERAGED_NORMALS //! vertex normal is determine as an avarage of all adjacent face normals
    //    ,WEIGHTED_NORMALS //! vertex normal is determined as AVARAGED_NORMALS, but with the face normals scaled by the face area
    //    } SmoothMethod;
    //void optimize(double smooth_angle, SmoothMethod method=WEIGHTED_NORMALS);
    //int addObject(Object3D::Ptr obj);
    //void addGeometry(const Material& mat, rw::common::Ptr<Geometry> geom);
    //void addTriMesh(const Material& mat, const rw::geometry::TriMesh& mesh);
    //int addMaterial(const Material& mat);
    //Material* getMaterial(const std::string& matid);
    bool hasMaterial(const std::string& matid);
    void removeObject(const std::string& name);
    //std::vector<Material>& getMaterials();
    //std::vector<Object3D::Ptr>& getObjects();
    const rw::math::Transform3D<double>& getTransform();
    void setTransform(const rw::math::Transform3D<double>& t3d);
    const std::string& getName();
    void setName(const std::string& name);
    int getMask();
    void setMask(int mask);
    rw::common::Ptr<GeometryData> toGeometryData();
    bool isDynamic() const;
    void setDynamic(bool dynamic);
};

%template (Model3DPtr) rw::common::Ptr<Model3D>;
%template (Model3DPtrVector) std::vector<rw::common::Ptr<Model3D> >;
OWNEDPTR(Model3D);

class WorkCellScene {
 public:

     rw::common::Ptr<WorkCell> getWorkCell();

     void setState(const State& state);

     //rw::graphics::GroupNode::Ptr getWorldNode();
     void updateSceneGraph(State& state);
     //void clearCache();

     void setVisible(bool visible, Frame* f);

     bool isVisible(Frame* f);

     void setHighlighted( bool highlighted, Frame* f);
     bool isHighlighted( Frame* f);
     void setFrameAxisVisible( bool visible, Frame* f);
     bool isFrameAxisVisible( Frame* f);
     //void setDrawType( DrawableNode::DrawType type, Frame* f);
     //DrawableNode::DrawType getDrawType( Frame* f );

     void setDrawMask( unsigned int mask, Frame* f);
     unsigned int getDrawMask( Frame* f );
     void setTransparency(double alpha, Frame* f);

     //DrawableGeometryNode::Ptr addLines( const std::string& name, const std::vector<rw::geometry::Line >& lines, Frame* frame, int dmask=DrawableNode::Physical);
     //DrawableGeometryNode::Ptr addGeometry(const std::string& name, rw::common::Ptr<Geometry> geom, Frame* frame, int dmask=DrawableNode::Physical);
     rw::common::Ptr<DrawableNode> addFrameAxis(const std::string& name, double size, Frame* frame, int dmask=DrawableNode::Virtual);
     //rw::common::Ptr<DrawableNode> addModel3D(const std::string& name, rw::common::Ptr<Model3D> model, Frame* frame, int dmask=DrawableNode::Physical);
     //rw::common::Ptr<DrawableNode> addImage(const std::string& name, const rw::sensor::Image& img, Frame* frame, int dmask=DrawableNode::Virtual);
     //rw::common::Ptr<DrawableNode> addScan(const std::string& name, const rw::sensor::Scan2D& scan, Frame* frame, int dmask=DrawableNode::Virtual);
     //rw::common::Ptr<DrawableNode> addScan(const std::string& name, const rw::sensor::Image25D& scan, Frame* frame, int dmask=DrawableNode::Virtual);
     //rw::common::Ptr<DrawableNode> addRender(const std::string& name, rw::graphics::Render::Ptr render, Frame* frame, int dmask=DrawableNode::Physical);

     rw::common::Ptr<DrawableNode> addDrawable(const std::string& filename, Frame* frame, int dmask);
     void addDrawable(rw::common::Ptr<DrawableNode> drawable, Frame*);

     //std::vector<rw::common::Ptr<DrawableNode> > getDrawables();
     //std::vector<rw::common::Ptr<DrawableNode> > getDrawables(Frame* f);

     //std::vector<rw::common::Ptr<DrawableNode> > getDrawablesRec(Frame* f, State& state);
     rw::common::Ptr<DrawableNode> findDrawable(const std::string& name);

     rw::common::Ptr<DrawableNode> findDrawable(const std::string& name, Frame* frame);

     std::vector<rw::common::Ptr<DrawableNode> > findDrawables(const std::string& name);

     bool removeDrawables(Frame* f);

     bool removeDrawables(const std::string& name);

     bool removeDrawable(rw::common::Ptr<DrawableNode> drawable);

     bool removeDrawable(rw::common::Ptr<DrawableNode> drawable, Frame* f);

     bool removeDrawable(const std::string& name);
     bool removeDrawable(const std::string& name, Frame* f);
     Frame* getFrame(rw::common::Ptr<DrawableNode>  d);

     //rw::graphics::GroupNode::Ptr getNode(Frame* frame);
 };
 
 %nodefaultctor SceneViewer;
 class SceneViewer
 {
 };
 
 %template (SceneViewerPtr) rw::common::Ptr<SceneViewer>;

/********************************************
 * GRASPPLANNING
 ********************************************/

/********************************************
 * INVKIN
 ********************************************/
 
 %include <rwlibs/swig/rwinvkin.i>

/********************************************
 * KINEMATICS
 ********************************************/

%nodefaultctor State;
class State
{
public:
	std::size_t size() const;
	State clone();
};
%template (StateVector) std::vector<State>;

/**
 * @brief the basic building block for the stateless design using
 * the StateStructure class. A StateData represents a size,
 * a unique id, and a unique name, when inserted into the StateStructure.
 * The size will allocate "size"-doubles in State objects originating from the
 * StateStructure.
 */
class StateData {
protected:
    StateData(int size, const std::string& name);
public:
    /**
     * @brief The name of the state data.
     *
     * @return The name of the state data.
     */
    const std::string& getName() const;

    /**
     * @brief The number of doubles allocated by this StateData in
     * each State object.
     *
     * @return The number of doubles allocated by the StateData
     */
    int size() const;

#if !defined(SWIGJAVA)
    /**
     * @brief An array of length size() containing the values for
     * the state data.
     *
     * It is OK to call this method also for a StateData with zero size.
     *
     * @param state [in] The state containing the StateData values.
     *
     * @return The values for the frame.
     */
    double* getData(State& state);
#endif
#if defined(SWIGJAVA)
%apply double[] {double *};
#endif
    /**
     * @brief Assign for \b state data the size() of values of the array \b
     * vals.
     *
     * The array \b vals must be of length at least size().
     *
     * @param state [inout] The state to which \b vals are written.
     *
     * @param vals [in] The joint values to assign.
     */
    void setData(State& state, const double* vals) const;
};

class StateStructure {
public:
	StateStructure();
	 
	void addFrame(Frame *frame, Frame *parent=NULL);
	const State& getDefaultState() const;
	const std::vector<Frame*>& getFrames() const;
};
%template (StateStructurePtr) rw::common::Ptr<StateStructure>;
OWNEDPTR(StateStructure);

/**
 * @brief The type of node of forward kinematic trees.
 *
 * Types of joints are implemented as subclasses of Frame. The
 * responsibility of a joint is to implement the getTransform() method that
 * returns the transform of the frame relative to whatever parent it is
 * attached to.
 *
 * The getTransform() method takes as parameter the set of joint values
 * State for the tree. Joint values for a particular frame can be accessed
 * via State::getQ(). A frame may contain pointers to other frames so that
 * the transform of a frame may depend on the joint values for other frames
 * also.
 */
class Frame : public StateData
{
public:

    /**
     * @brief Post-multiply the transform of the frame to the parent transform.
     *
     * The transform is calculated for the joint values of \b state.
     *
     * The exact implementation of getTransform() depends on the type of
     * frame. See for example RevoluteJoint and PrismaticJoint.
     *
     * @param parent [in] The world transform of the parent frame.
     * @param state [in] Joint values for the forward kinematics tree.
     * @param result [in] The transform of the frame in the world frame.
     */
    void multiplyTransform(const rw::math::Transform3D<double>& parent,
                           const State& state,
                           rw::math::Transform3D<double>& result) const;

    /**
     * @brief The transform of the frame relative to its parent.
     *
     * The transform is calculated for the joint values of \b state.
     *
     * The exact implementation of getTransform() depends on the type of
     * frame. See for example RevoluteJoint and PrismaticJoint.
     *
     * @param state [in] Joint values for the forward kinematics tree.
     *
     * @return The transform of the frame relative to its parent.
     */
    rw::math::Transform3D<double> getTransform(const State& state) const;

#if !defined(SWIGJAVA) 
    /**
     * @brief Miscellaneous properties of the frame.
     *
     * The property map of the frame is provided to let the user store
     * various settings for the frame. The settings are typically loaded
     * from setup files.
     *
     * The low-level manipulations of the property map can be cumbersome. To
     * ease these manipulations, the PropertyAccessor utility class has been
     * provided. Instances of this class are provided for a number of common
     * settings, however it is undecided if these properties are a public
     * part of RobWork.
     *
     * @return The property map of the frame.
     */
    const PropertyMap& getPropertyMap() const;
#endif

    /**
     * @brief Miscellaneous properties of the frame.
     *
     * The property map of the frame is provided to let the user store
     * various settings for the frame. The settings are typically loaded
     * from setup files.
     *
     * The low-level manipulations of the property map can be cumbersome. To
     * ease these manipulations, the PropertyAccessor utility class has been
     * provided. Instances of this class are provided for a number of common
     * settings, however it is undecided if these properties are a public
     * part of RobWork.
     *
     * @return The property map of the frame.
     */
    PropertyMap& getPropertyMap();


    /**
     * @brief The number of degrees of freedom (dof) of the frame.
     *
     * The dof is the number of joint values that are used for controlling
     * the frame.
     *
     * Given a set joint values of type State, the getDof() number of joint
     * values for the frame can be read and written with State::getQ() and
     * State::setQ().
     *
     * @return The number of degrees of freedom of the frame.
     */
    int getDOF() const;


    // The parents

#if !defined(SWIGJAVA)
    //! @brief The parent of the frame or NULL if the frame is a DAF.
    const Frame* getParent() const;
#endif

    //! @brief The parent of the frame or NULL if the frame is a DAF.
    Frame* getParent();

    /**
     * @brief Returns the parent of the frame
     *
     * If no static parent exists it look for at DAF parent. If such
     * does not exists either it returns NULL.
     *
     * @param state [in] the state to consider
     * @return the parent
     */
    Frame* getParent(const State& state);

#if !defined(SWIGJAVA)
    /**
     * @brief Returns the parent of the frame
     *
     * If no static parent exists it look for at DAF parent. If such
     * does not exists either it returns NULL.
     *
     * @param state [in] the state to consider
     * @return the parent
     */
    const Frame* getParent(const State& state) const;

    /**
     * @brief The dynamically attached parent or NULL if the frame is not a
     * DAF.
     */
    const Frame* getDafParent(const State& state) const;
#endif

    /**
     * @brief The dynamically attached parent or NULL if the frame is not a
     * DAF.
     */
    Frame* getDafParent(const State& state);

    // Iterator stuff left out of script interface for now!

    // Dynamic frame attachments.

    /**
     * @brief Move a frame within the tree.
     *
     * The frame \b frame is detached from its parent and reattached to \b
     * parent. The frames \b frame and \b parent must both belong to the
     * same kinematics tree.
     *
     * Only frames with no static parent (see getParent()) can be moved.
     *
     * @param parent [in] The frame to attach \b frame to.
     * @param state [inout] The state to which the attachment is written.
     */
    void attachTo(Frame* parent, State& state);

    /**
     * @brief Test if this frame is a Dynamically Attachable Frame
     *
     * @return true if this frame is a DAF, false otherwise
     */
    bool isDAF();

    /**
     * @brief Get the transform relative to world.
     *
     * @param state [in] the state.
     * @return transform relative to world.
     */
    rw::math::Transform3D<double> wTf(const State& state) const;

    /**
     * @brief Get the transform of other frame relative to this frame.
     *
     * @param to [in] the other frame
     * @param state [in] the state.
     * @return transform of frame \b to relative to this frame.
     */
    rw::math::Transform3D<double> fTf(const Frame* to, const State& state) const;

private:
    // Frames should not be copied.
    Frame(const Frame&);
    Frame& operator=(const Frame&);
};

%template (FramePtr) rw::common::Ptr<Frame>;
%template (FrameCPtr) rw::common::Ptr<const Frame>;
%template (FrameVector) std::vector<Frame*>;

class MovableFrame: public Frame{
public:
   explicit MovableFrame(const std::string& name);

   void setTransform(const rw::math::Transform3D<double> & transform, State& state);
};

class FixedFrame: public Frame {
public:
    FixedFrame(const std::string& name, const rw::math::Transform3D<double> & transform);
    void setTransform(const rw::math::Transform3D<double> & transform);

    const rw::math::Transform3D<double> & getFixedTransform() const;
};

%inline %{
    rw::math::Transform3D<double>  frameTframe(const Frame* from, const Frame* to, const State& state){
        return ::rw::kinematics::Kinematics::frameTframe(from, to, state );
    }

    rw::math::Transform3D<double>  worldTframe(const Frame* to, const State& state){
        return ::rw::kinematics::Kinematics::worldTframe( to,  state);
    }

    Frame* worldFrame(Frame* frame, const State& state) {
        return ::rw::kinematics::Kinematics::worldFrame( frame, state );
    }

    void gripFrame(Frame* item, Frame* gripper, State& state){
        return ::rw::kinematics::Kinematics::gripFrame( item, gripper, state);
    }

    void gripFrame(MovableFrame* item, Frame* gripper, State& state){
        return ::rw::kinematics::Kinematics::gripFrame( item, gripper, state);
    }

    bool isDAF(const Frame* frame){
        return ::rw::kinematics::Kinematics::isDAF( frame );
    }
%}

class Joint: public Frame
{
};

%template (JointVector) std::vector<Joint*>;


/********************************************
 * LOADERS
 ********************************************/

/**
 * @brief Extendible interface for loading of WorkCells from files.
 *
 * By default, the following formats are supported:
 *
 * - File extensions ".wu", ".wc", ".tag", ".dev" will be loaded using
 *   the TULLoader.
 * - Remaining file extensions will be loaded using the standard RobWork
 *   XML format (XMLRWLoader).
 *
 * The Factory defines an extension point "rw.loaders.WorkCellLoader"
 * that makes it possible to add loaders for other file formats than the
 * ones above. Extensions take precedence over the default loaders.
 *
 * The WorkCell loader is chosen based on a case-insensitive file extension
 * name. So "scene.wc.xml" will be loaded by the same loader as
 * "scene.WC.XML"
 *
 * WorkCells are supposed to be loaded using the WorkCellLoaderFactory.load function:
 * @beginPythonOnly
 * ::\n
 *     wc = WorkCellLoaderFactory.load("scene.wc.xml")
 *     if wc.isNull():
 *         raise Exception("WorkCell could not be loaded")
 * @endPythonOnly
 * @beginJavaOnly <pre> \code
 * WorkCellPtr wc = WorkCellLoaderFactory.load("scene.wc.xml");
 * if (wc.isNull())
 *     throw new Exception("WorkCell could not be loaded.");
 * \endcode </pre> @endJavaOnly
 * Alternatively a WorkCell can be loaded in the less convenient way:
 * @beginPythonOnly
 * ::\n
 *    loader = WorkCellLoaderFactory.getWorkCellLoader(".wc.xml");
 *    wc = loader.load("scene.wc.xml")
 *    if wc.isNull():
 *        raise Exception("WorkCell could not be loaded")
 * @endPythonOnly
 * @beginJavaOnly <pre> \code
 * WorkCellLoaderPtr loader = WorkCellLoaderFactory.getWorkCellLoader(".wc.xml");
 * WorkCellPtr wc = loader.loadWorkCell("scene.wc.xml");
 * if (wc.isNull())
 *     throw new Exception("WorkCell could not be loaded.");
 * \endcode </pre> @endJavaOnly
 */
class WorkCellLoader {
public:
	virtual ~WorkCellLoader();
    /**
     * @brief Load a WorkCell from a file.
     *
     * @param filename [in] path to workcell file.
     */
	virtual rw::common::Ptr<WorkCell> loadWorkCell(const std::string& filename) = 0;

protected:
	WorkCellLoader();
};

%template (WorkCellLoaderPtr) rw::common::Ptr<WorkCellLoader>;

/**
 * @brief A factory for WorkCellLoader. This factory also defines the
 * "rw.loaders.WorkCellLoader" extension point where new loaders can be
 * registered.
 */
class WorkCellLoaderFactory {
public:
	/**
	 * @brief Get loaders for a specific format.
	 *
	 * @param format [in] the extension (including initial dot).
	 * The extension name is case-insensitive.
	 * @return a suitable loader.
	 */
	static rw::common::Ptr<WorkCellLoader> getWorkCellLoader(const std::string& format);

    /**
     * @brief Loads/imports a WorkCell from a file.
     *
     * An exception is thrown if the file can't be loaded.
     * The RobWork XML format is supported by default, as well as
     * TUL WorkCell format.
     *
     * @param filename [in] name of the WorkCell file.
     */
	static rw::common::Ptr<WorkCell> load(const std::string& filename);
private:
	WorkCellLoaderFactory();
};

class ImageLoader {
public:
	virtual ~ImageLoader();
	virtual rw::common::Ptr<Image> loadImage(const std::string& filename) = 0;
	virtual std::vector<std::string> getImageFormats() = 0;
	virtual bool isImageSupported(const std::string& format);
};

%template (ImageLoaderPtr) rw::common::Ptr<ImageLoader>;

class ImageLoaderFactory {
public:
	ImageLoaderFactory();
	static rw::common::Ptr<ImageLoader> getImageLoader(const std::string& format);
	static bool hasImageLoader(const std::string& format);
	static std::vector<std::string> getSupportedFormats();
};

class Image {
};

%template (ImagePtr) rw::common::Ptr<Image>;

#if defined(RW_HAVE_XERCES)

class XMLTrajectoryLoader
{
public:
    XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");
    XMLTrajectoryLoader(std::istream& instream, const std::string& schemaFileName = "");

    enum Type { QType = 0, Vector3DType, Rotation3DType, Transform3DType};
    Type getType();
    rw::common::Ptr<Trajectory<rw::math::Q> > getQTrajectory();
    rw::common::Ptr<Trajectory<rw::math::Vector3D<double> > > getVector3DTrajectory();
    rw::common::Ptr<Trajectory<rw::math::Rotation3D<double> > > getRotation3DTrajectory();
    rw::common::Ptr<Trajectory<rw::math::Transform3D<double> > > getTransform3DTrajectory();
};

class XMLTrajectorySaver
{
public:
    static bool save(const Trajectory<rw::math::Q>& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Vector3D<double> >& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Rotation3D<double> >& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Transform3D<double> >& trajectory, const std::string& filename);
    static bool write(const Trajectory<rw::math::Q>& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<rw::math::Vector3D<double> >& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<rw::math::Rotation3D<double> >& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<rw::math::Transform3D<double> >& trajectory, std::ostream& outstream);
private:
    XMLTrajectorySaver();
};

#endif

/********************************************
 * MATH
 ********************************************/
%include <rwlibs/swig/rwmath.i>

// Utility function within rw::Math
rw::math::Rotation3D<double> getRandomRotation3D();
rw::math::Transform3D<double>  getRandomTransform3D(const double translationLength = 1);

namespace rw { namespace math {
    class Math
    {
    public:
        Math() = delete;
        ~Math() = delete;
        static inline double clamp(double val, double min, double max);

        static rw::math::Q clampQ(const rw::math::Q& q,
                                  const rw::math::Q& min,
                                  const rw::math::Q& max);

        static rw::math::Q clampQ(const rw::math::Q& q,
                                  const std::pair<rw::math::Q, rw::math::Q>& bounds);

        static rw::math::Vector3D<double> clamp(const rw::math::Vector3D<double>& q,
                                          const rw::math::Vector3D<double>& min,
                                          const rw::math::Vector3D<double>& max);

        static double ran();

        static void seed(unsigned seed);

        static void seed();

        static double ran(double from, double to);

        static int ranI(int from, int to);

        static double ranNormalDist(double mean, double sigma);

        static rw::math::Q ranQ(const rw::math::Q& from, const rw::math::Q& to);

        static rw::math::Q ranQ(const std::pair<rw::math::Q,rw::math::Q>& bounds);

        static rw::math::Q ranDir(size_t dim, double length = 1);
        
        static rw::math::Q ranWeightedDir(size_t dim, const rw::math::Q& weights, double length = 1);

        static double round(double d);

        static rw::math::Q sqr(const rw::math::Q& q);

        static rw::math::Q sqrt(const rw::math::Q& q);

        static rw::math::Q abs(const rw::math::Q& v);

        static double min(const rw::math::Q& v);

        static double max(const rw::math::Q& v);

        static double sign(double s);

        static rw::math::Q sign(const rw::math::Q& q);

        static int ceilLog2(int n);
        
        static long long factorial(long long n);

        static bool isNaN(double d);
    };
}} // end namespaces

/********************************************
 * MODELS
 ********************************************/
 
 %nodefaultctor JacobianCalculator;
//! @brief JacobianCalculator provides an interface for obtaining a Jacobian
class JacobianCalculator
{
public:
    //! @brief Destructor
    virtual ~JacobianCalculator();

    %extend {
		/**
		 * @brief Returns the Jacobian associated to \b state
		 *
		 * @param state [in] State for which to calculate the Jacobian
		 * @return Jacobian for \b state
		 */
		virtual rw::math::Jacobian getJacobian(const State& state) const {
			return $self->get(state);
		}
    };
};

%template (JacobianCalculatorPtr) rw::common::Ptr<JacobianCalculator>;

class WorkCell {
public:
    WorkCell(const std::string& name);
    std::string getName() const;
    Frame* getWorldFrame() const;

    void addFrame(Frame* frame, Frame* parent=NULL);
    void addDAF(Frame* frame, Frame* parent=NULL);
    void remove(Frame* frame);
    void addDevice(rw::common::Ptr<Device> device);
    
    Frame* findFrame(const std::string& name) const;

    %extend {
        MovableFrame* findMovableFrame(const std::string& name)
        { return $self->rw::models::WorkCell::findFrame<MovableFrame>(name); }
        FixedFrame* findFixedFrame(const std::string& name)
        { return $self->rw::models::WorkCell::findFrame<FixedFrame>(name); }
        void addObject(rw::common::Ptr<Object> object)
        { $self->rw::models::WorkCell::add(object); }
    };

    std::vector<Frame*> getFrames() const;
    rw::common::Ptr<Device> findDevice(const std::string& name) const;
    %extend {
        rw::common::Ptr<JointDevice> findJointDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<JointDevice>(name); }
        rw::common::Ptr<SerialDevice> findSerialDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<SerialDevice>(name); }
        rw::common::Ptr<TreeDevice> findTreeDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<TreeDevice>(name); }
        rw::common::Ptr<ParallelDevice> findParallelDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<ParallelDevice>(name); }
        std::vector<rw::common::Ptr<Device> > getDevices() const
                { return $self->rw::models::WorkCell::getDevices(); }
    };
    
    rw::common::Ptr<Object> findObject(const std::string& name) const;
    void add(rw::common::Ptr<Object> object);
    void remove(rw::common::Ptr<Object> object);


    State getDefaultState() const;

    //rw::common::Ptr<StateStructure> getStateStructure();

    PropertyMap& getPropertyMap();
private:
    WorkCell(const WorkCell&);
    WorkCell& operator=(const WorkCell&);
};

%template (WorkCellPtr) rw::common::Ptr<WorkCell>;

class Object
{
public:
    //! destructor
    virtual ~Object();
    const std::string& getName();
    Frame* getBase();
    const std::vector<Frame*>& getFrames();
    void addFrame(Frame* frame);
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry() const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;

    // stuff that should be implemented by deriving classes
     const std::vector<rw::common::Ptr<Geometry> >& getGeometry(const State& state) const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels(const State& state) const;
    virtual double getMass(State& state) const = 0;
    virtual rw::math::Vector3D<double> getCOM(State& state) const = 0;
    virtual rw::math::InertiaMatrix<double> getInertia(State& state) const = 0;
};
%template (ObjectPtr) rw::common::Ptr<Object>;
OWNEDPTR(Object);

class RigidObject : public Object {
public:
	RigidObject(Frame* baseframe);
	RigidObject(Frame* baseframe, rw::common::Ptr<Geometry> geom);
	RigidObject(Frame* baseframe, std::vector<rw::common::Ptr<Geometry> > geom);
	RigidObject(std::vector<Frame*> frames);
	RigidObject(std::vector<Frame*> frames, rw::common::Ptr<Geometry> geom);
	RigidObject(std::vector<Frame*> frames, std::vector<rw::common::Ptr<Geometry> > geom);
	void addGeometry(rw::common::Ptr<Geometry> geom);
	void removeGeometry(rw::common::Ptr<Geometry> geom);
	void addModel(rw::common::Ptr<Model3D> model);
	void removeModel(rw::common::Ptr<Model3D> model);
    double getMass() const;
    void setMass(double mass);
    rw::math::InertiaMatrix<double> getInertia() const;
    void setInertia(const rw::math::InertiaMatrix<double>& inertia);
    void setCOM(const rw::math::Vector3D<double>& com);
    void approximateInertia();
    void approximateInertiaCOM();
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry() const ;
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;
    double getMass(State& state) const;
    rw::math::InertiaMatrix<double> getInertia(State& state) const;
    rw::math::Vector3D<double> getCOM(State& state) const;
};

%template (RigidObjectPtr) rw::common::Ptr<RigidObject>;
OWNEDPTR(RigidObject);

class DeformableObject: public Object
{
public:
     //! constructor
    DeformableObject(Frame* baseframe, int nr_of_nodes);

    //DeformableObject(Frame* baseframe, rw::common::Ptr<Model3D> model);

    //DeformableObject(Frame* baseframe, rw::common::Ptr<Geometry> geom);

    //! destructor
    virtual ~DeformableObject();

    rw::math::Vector3D<float>& getNode(int id, State& state) const;
    void setNode(int id, const rw::math::Vector3D<float>& v, State& state);
    
    //const std::vector<rw::geometry::IndexedTriangle<> >& getFaces() const;
    void addFace(unsigned int node1, unsigned int node2, unsigned int node3);
    //rw::geometry::IndexedTriMesh<float>::Ptr getMesh(State& cstate);
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry(const State& state) const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels(const State& state) const;
    
    double getMass(State& state) const;
    rw::math::Vector3D<double> getCOM(State& state) const;
    rw::math::InertiaMatrix<double> getInertia(State& state) const;
    void update(rw::common::Ptr<Model3D> model, const State& state);
};

%template (DeformableObjectPtr) rw::common::Ptr<DeformableObject>;
OWNEDPTR(DeformableObject);

class Device
{
public:
    Device(const std::string& name);
    //void registerStateData(rw::kinematics::StateStructure::Ptr sstruct);
    virtual void setQ(const rw::math::Q& q, State& state) const = 0;
    virtual rw::math::Q getQ(const State& state) const = 0;
    virtual std::pair<rw::math::Q,rw::math::Q> getBounds() const = 0;
    virtual rw::math::Q getVelocityLimits() const = 0;
    virtual void setVelocityLimits(const rw::math::Q& vellimits) = 0;
    virtual rw::math::Q getAccelerationLimits() const = 0;
    virtual void setAccelerationLimits(const rw::math::Q& acclimits) = 0;
    virtual size_t getDOF() const = 0;
    std::string getName() const;
    void setName(const std::string& name);
    virtual Frame* getBase() = 0;
    virtual Frame* getEnd() = 0;
#if !defined(SWIGJAVA)
    virtual const Frame* getBase() const = 0;
    virtual const Frame* getEnd() const = 0;
#endif
    rw::math::Transform3D<double>  baseTframe(const Frame* f, const State& state) const;
    rw::math::Transform3D<double>  baseTend(const State& state) const;
    rw::math::Transform3D<double>  worldTbase(const State& state) const;
    virtual rw::math::Jacobian baseJend(const State& state) const = 0;
    virtual rw::math::Jacobian baseJframe(const Frame* frame,const State& state) const;
    virtual rw::math::Jacobian baseJframes(const std::vector<Frame*>& frames,const State& state) const;
    //virtual rw::common::Ptr<JacobianCalculator> baseJCend(const State& state) const;
    //virtual JacobianCalculatorPtr baseJCframe(const kinematics::Frame* frame, const State& state) const;
    //virtual JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames, const State& state) const = 0;
private:
    Device(const Device&);
    Device& operator=(const Device&);
};

%template (DevicePtr) rw::common::Ptr<Device>;
%template (DeviceCPtr) rw::common::Ptr<const Device>;
%template (DevicePtrVector) std::vector<rw::common::Ptr<Device> >;
OWNEDPTR(Device)

%extend rw::common::Ptr<Device> {
    rw::common::Ptr<const Device> asDeviceCPtr() { return *$self; }
}

class JointDevice: public Device
{
public:
    const std::vector<Joint*>& getJoints() const;
    void setQ(const rw::math::Q& q, State& state) const;
    rw::math::Q getQ(const State& state) const;
    size_t getDOF() const;
    std::pair<rw::math::Q, rw::math::Q> getBounds() const;
    void setBounds(const std::pair<rw::math::Q, rw::math::Q>& bounds);
    rw::math::Q getVelocityLimits() const;
    void setVelocityLimits(const rw::math::Q& vellimits);
    rw::math::Q getAccelerationLimits() const;
    void setAccelerationLimits(const rw::math::Q& acclimits);
    rw::math::Jacobian baseJend(const State& state) const;

    //JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames,
    //                                   const State& state) const;

    Frame* getBase();
    virtual Frame* getEnd();

#if !defined(SWIGJAVA)
    const Frame* getBase() const;
    virtual const Frame* getEnd() const;
#endif
};

%template (JointDevicePtr) rw::common::Ptr<JointDevice>;
%template (JointDeviceCPtr) rw::common::Ptr<const JointDevice>;
OWNEDPTR(JointDevice)

class CompositeDevice: public JointDevice
{
public:
    CompositeDevice(
        Frame* base,
		const std::vector<rw::common::Ptr<Device> >& devices,
        Frame* end,
        const std::string& name,
        const State& state);

    CompositeDevice(
        Frame *base,
		const std::vector<rw::common::Ptr<Device> >& devices,
        const std::vector<Frame*>& ends,
        const std::string& name,
        const State& state);
};

%template (CompositeDevicePtr) rw::common::Ptr<CompositeDevice>;
OWNEDPTR(CompositeDevice)

%extend rw::common::Ptr<CompositeDevice> {
    rw::common::Ptr<Device> asDevicePtr() { return *$self; }
    rw::common::Ptr<const Device> asDeviceCPtr() { return *$self; }
    rw::common::Ptr<JointDevice> asJointDevicePtr() { return *$self; }
    rw::common::Ptr<const JointDevice> asJointDeviceCPtr() { return *$self; }
}

class SerialDevice: public JointDevice
{
};
%template (SerialDevicePtr) rw::common::Ptr<SerialDevice>;
%template (SerialDeviceCPtr) rw::common::Ptr<const SerialDevice>;
OWNEDPTR(SerialDevice)

%extend rw::common::Ptr<SerialDevice> {
    rw::common::Ptr<const SerialDevice> asSerialDeviceCPtr() { return *$self; }
}

class ParallelDevice: public JointDevice
{
};
%template (ParallelDevicePtr) rw::common::Ptr<ParallelDevice>;
OWNEDPTR(ParallelDevice)

class TreeDevice: public JointDevice
{
public:
	TreeDevice(
		Frame* base,
		const std::vector<Frame*>& ends,
		const std::string& name,
		const State& state);
};
%template (TreeDevicePtr) rw::common::Ptr<TreeDevice>;
%template (TreeDeviceCPtr) rw::common::Ptr<const TreeDevice>;
OWNEDPTR(TreeDevice)

%nodefaultctor DHParameterSet;
class DHParameterSet
{
};

%template (DHParameterSetVector) std::vector<DHParameterSet>;

/********************************************
 * PATHPLANNING
 ********************************************/

%include <rwlibs/swig/rwplanning.i>

/********************************************
 * PLUGIN
 ********************************************/

/********************************************
 * PROXIMITY
 ********************************************/

class ProximityData {
};

%nodefaultctor ProximityStrategy;
/**
 * @brief The ProximityStrategy interface is a clean interface
 * for defining methods that are common for different proximity
 * strategy classes. Specifically adding of geometric models and
 * relating them to frames.
 */
class ProximityStrategy {
};

%template (ProximityStrategyPtr) rw::common::Ptr<ProximityStrategy>;

%nodefaultctor CollisionStrategy;
/**
 * @brief An interface that defines methods to test collision between
 * two objects.
 */
class CollisionStrategy : public virtual ProximityStrategy {
};

%template (CollisionStrategyPtr) rw::common::Ptr<CollisionStrategy>;

%nodefaultctor DistanceStrategy;
/**
 * @brief This is an interface that defines methods for computing the minimum distance
 * between geometric objects. If geometry objects has been related to frames (see ProximityStrategy)
 * then distance functions computing the distance between the geometry attached to frames can also be used.
 */
class DistanceStrategy : public virtual ProximityStrategy {
};

%template (DistanceStrategyPtr) rw::common::Ptr<DistanceStrategy>;

%nodefaultctor CollisionDetector;
class CollisionDetector
{
public:
	CollisionDetector(rw::common::Ptr<WorkCell> workcell);
	CollisionDetector(rw::common::Ptr<WorkCell> workcell, rw::common::Ptr<CollisionStrategy> strategy);

    /**
     * @brief Check the workcell for collisions.
     *
     * @param state [in] The state for which to check for collisions.
     * @param data [in/out] Defines parameters for the collision check, the results and also
     * enables caching inbetween calls to incollision
     * @return true if a collision is detected; false otherwise.
     */
    bool inCollision(const State& state, class ProximityData &data) const;

    %extend {
    /*
        static rw::common::Ptr<CollisionDetector> make(rw::common::Ptr<WorkCell> workcell){
            return rw::common::ownedPtr( new CollisionDetector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()) );
        }
    */

        static rw::common::Ptr<CollisionDetector> make(rw::common::Ptr<WorkCell> workcell, rw::common::Ptr<CollisionStrategy> strategy){
            return rw::common::ownedPtr( new CollisionDetector(workcell, strategy) );
        }
    }
};

%template (CollisionDetectorPtr) rw::common::Ptr<CollisionDetector>;
OWNEDPTR(CollisionDetector)

/********************************************
 * SENSOR
 ********************************************/

/********************************************
 * TRAJECTORY
 ********************************************/

template <class T>
class Timed
{
public:
    Timed();
    Timed(double time, const T& value);

    double getTime() const;
    T& getValue();

    %extend {
        void setTime(double time){
            $self->rw::trajectory::Timed<T>::getTime() = time;
        }
    };
};

%template (TimedQ) Timed<rw::math::Q>;
%template (TimedState) Timed<State>;

template <class T>
class Path: public std::vector<T>
{
public:

    Path();
    Path(size_t cnt);
    Path(size_t cnt, const T& value);
    Path(const std::vector<T>& v);

#if (defined (SWIGJAVA) && SWIG_VERSION >= 0x040000)
    %extend {
        int size(){ return boost::numeric_cast<int>($self->std::vector<T >::size()); }
        T& elem(size_t idx){ return (*$self)[idx]; }
    };
#else
    %extend {
        size_t size(){ return $self->std::vector<T >::size(); }
        T& elem(size_t idx){ return (*$self)[idx]; }
    };
#endif
};

%template (TimedQVector) std::vector<Timed<rw::math::Q> >;
%template (TimedStateVector) std::vector<Timed<State> >;
%template (TimedQVectorPtr) rw::common::Ptr<std::vector<Timed<rw::math::Q> > >;
%template (TimedStateVectorPtr) rw::common::Ptr<std::vector<Timed<State> > >;
OWNEDPTR(std::vector<Timed<rw::math::Q> > )
//OWNEDPTR(std::vector<Timed<State> > )

%template (PathSE3) Path<rw::math::Transform3D<double> >;
%template (PathSE3Ptr) rw::common::Ptr<Path<rw::math::Transform3D<double> > >;
%template (PathQ) Path<rw::math::Q>;
%template (PathQPtr) rw::common::Ptr<Path<rw::math::Q> >;
%template (PathTimedQ) Path<Timed<rw::math::Q> >;
%template (PathTimedQPtr) rw::common::Ptr<Path<Timed<rw::math::Q> > >;
%template (PathTimedState) Path<Timed<State> >;
%template (PathTimedStatePtr) rw::common::Ptr<Path<Timed<State> > >;
OWNEDPTR(Path<rw::math::Transform3D<double> > )
OWNEDPTR(Path<rw::math::Q> )
OWNEDPTR(Path<Timed<rw::math::Q> > )
OWNEDPTR(Path<Timed<State> > )

%extend Path<rw::math::Q> {
    rw::common::Ptr<Path<Timed<rw::math::Q> > > toTimedQPath(rw::math::Q speed){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(speed, *$self);
        return rw::common::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::common::Ptr<Path<Timed<rw::math::Q> > > toTimedQPath(rw::common::Ptr<Device> dev){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(*dev, *$self);
        return rw::common::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::common::Ptr<Path<Timed<State> > > toTimedStatePath(rw::common::Ptr<Device> dev,
                                                     const State& state){
        rw::trajectory::TimedStatePath tpath =
                rw::trajectory::TimedUtil::makeTimedStatePath(*dev, *$self, state);
        return rw::common::ownedPtr( new rw::trajectory::TimedStatePath(tpath) );
    }

};

%extend Path<Timed<State> > {
	
	static rw::common::Ptr<Path<Timed<State> > > load(const std::string& filename, rw::common::Ptr<WorkCell> wc){
		rw::common::Ptr<rw::trajectory::TimedStatePath> spath = 
                    rw::common::ownedPtr(new rw::trajectory::TimedStatePath);
                *spath = rw::loaders::PathLoader::loadTimedStatePath(*wc, filename);
		return rw::common::Ptr<rw::trajectory::TimedStatePath>( spath );
	}
	
	void save(const std::string& filename, rw::common::Ptr<WorkCell> wc){		 		
		rw::loaders::PathLoader::storeTimedStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::common::Ptr<Path<Timed<State> > > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).back().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			Timed<State> tstate = (*spath)[i]; 
			tstate.getTime() += startTime;
			(*$self).push_back( tstate );
		}
	}
	
};

%extend Path<State > {
	
	static rw::common::Ptr<Path<State> > load(const std::string& filename, rw::common::Ptr<WorkCell> wc){
            rw::common::Ptr<rw::trajectory::StatePath> spath = rw::common::ownedPtr(new rw::trajectory::StatePath);
            *spath = rw::loaders::PathLoader::loadStatePath(*wc, filename);
		return rw::common::ownedPtr( spath );
	}
	
	void save(const std::string& filename, rw::common::Ptr<WorkCell> wc){		 		
		rw::loaders::PathLoader::storeStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::common::Ptr<Path<State> > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).front().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			(*$self).push_back( (*spath)[i] );
		}
	}
	
	
	rw::common::Ptr<Path<Timed<State> > > toTimedStatePath(double timeStep){
		rw::common::Ptr<TimedStatePath> spath = 
			rw::common::ownedPtr( new rw::trajectory::TimedStatePath() );	
		for(size_t i = 0; i<spath->size(); i++){
			Timed<State> tstate(timeStep*i, (*spath)[i]); 
			spath->push_back( tstate );
		}	
		return spath;
	}
	
};



template <class T>
class Blend
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double tau1() const = 0;
    virtual double tau2() const = 0;
};

%template (BlendR1) Blend<double>;
%template (BlendR2) Blend<rw::math::Vector2D<double> >;
%template (BlendR3) Blend<rw::math::Vector3D<double> >;
%template (BlendSO3) Blend<rw::math::Rotation3D<double> >;
%template (BlendSE3) Blend<rw::math::Transform3D<double> >;
%template (BlendQ) Blend<rw::math::Q>;

%template (BlendR1Ptr) rw::common::Ptr<Blend<double> >;
%template (BlendR2Ptr) rw::common::Ptr<Blend<rw::math::Vector2D<double> > >;
%template (BlendR3Ptr) rw::common::Ptr<Blend<rw::math::Vector3D<double> > >;
%template (BlendSO3Ptr) rw::common::Ptr<Blend<rw::math::Rotation3D<double> > >;
%template (BlendSE3Ptr) rw::common::Ptr<Blend<rw::math::Transform3D<double> > >;
%template (BlendQPtr) rw::common::Ptr<Blend<rw::math::Q> >;

OWNEDPTR(Blend<double> )
OWNEDPTR(Blend<rw::math::Vector2D<double> > )
OWNEDPTR(Blend<rw::math::Vector3D<double> > )
OWNEDPTR(Blend<rw::math::Rotation3D<double> > )
OWNEDPTR(Blend<rw::math::Transform3D<double> > )
OWNEDPTR(Blend<rw::math::Q> )

template <class T>
class Interpolator
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double duration() const = 0;
};

%template (InterpolatorR1) Interpolator<double>;
%template (InterpolatorR2) Interpolator<rw::math::Vector2D<double> >;
%template (InterpolatorR3) Interpolator<rw::math::Vector3D<double> >;
%template (InterpolatorSO3) Interpolator<rw::math::Rotation3D<double> >;
%template (InterpolatorSE3) Interpolator<rw::math::Transform3D<double> >;
%template (InterpolatorQ) Interpolator<rw::math::Q>;

%template (InterpolatorR1Ptr) rw::common::Ptr<Interpolator<double> >;
%template (InterpolatorR2Ptr) rw::common::Ptr<Interpolator<rw::math::Vector2D<double> > >;
%template (InterpolatorR3Ptr) rw::common::Ptr<Interpolator<rw::math::Vector3D<double> > >;
%template (InterpolatorSO3Ptr) rw::common::Ptr<Interpolator<rw::math::Rotation3D<double> > >;
%template (InterpolatorSE3Ptr) rw::common::Ptr<Interpolator<rw::math::Transform3D<double> > >;
%template (InterpolatorQPtr) rw::common::Ptr<Interpolator<rw::math::Q> >;

OWNEDPTR(Interpolator<double> )
OWNEDPTR(Interpolator<rw::math::Vector2D<double> > )
OWNEDPTR(Interpolator<rw::math::Vector3D<double> > )
OWNEDPTR(Interpolator<rw::math::Rotation3D<double> > )
OWNEDPTR(Interpolator<rw::math::Transform3D<double> > )
OWNEDPTR(Interpolator<rw::math::Q> )

class LinearInterpolator: public Interpolator<double> {
public:
    LinearInterpolator(const double& start,
                          const double& end,
                          double duration);

    virtual ~LinearInterpolator();

    double x(double t) const;
    double dx(double t) const;
    double ddx(double t) const;
    double duration() const;
};


class LinearInterpolatorQ: public Interpolator<rw::math::Q> {
public:
    LinearInterpolatorQ(const rw::math::Q& start,
                          const rw::math::Q& end,
                          double duration);

    virtual ~LinearInterpolatorQ();

    rw::math::Q x(double t) const;
    rw::math::Q dx(double t) const;
    rw::math::Q ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorR3: public Interpolator<rw::math::Rotation3D<double> > {
public:
    LinearInterpolatorR3(const rw::math::Rotation3D<double> & start,
                          const rw::math::Rotation3D<double> & end,
                          double duration);

    rw::math::Rotation3D<double>  x(double t) const;
    rw::math::Rotation3D<double>  dx(double t) const;
    rw::math::Rotation3D<double>  ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorSO3: public Interpolator<rw::math::Rotation3D<double> > {
public:
    LinearInterpolatorSO3(const rw::math::Rotation3D<double> & start,
                          const rw::math::Rotation3D<double> & end,
                          double duration);

    rw::math::Rotation3D<double>  x(double t) const;
    rw::math::Rotation3D<double>  dx(double t) const;
    rw::math::Rotation3D<double>  ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorSE3: public Interpolator<rw::math::Transform3D<double> > {
public:
    LinearInterpolatorSE3(const rw::math::Transform3D<double> & start,
                          const rw::math::Transform3D<double> & end,
                          double duration);

    rw::math::Transform3D<double>  x(double t) const;
    rw::math::Transform3D<double>  dx(double t) const;
    rw::math::Transform3D<double>  ddx(double t) const;
    double duration() const;
};


//////////// RAMP interpolator


class RampInterpolatorR3: public Interpolator<rw::math::Vector3D<double> > {
public:
    RampInterpolatorR3(const rw::math::Vector3D<double>& start, const rw::math::Vector3D<double>& end,
                       double vellimit,double acclimit);

    rw::math::Vector3D<double> x(double t) const;
    rw::math::Vector3D<double> dx(double t) const;
    rw::math::Vector3D<double> ddx(double t) const;
    double duration() const;
};

class RampInterpolatorSO3: public Interpolator<rw::math::Rotation3D<double> > {
public:
    RampInterpolatorSO3(const rw::math::Rotation3D<double> & start,
                          const rw::math::Rotation3D<double> & end,
                          double vellimit,double acclimit);

    rw::math::Rotation3D<double>  x(double t) const;
    rw::math::Rotation3D<double>  dx(double t) const;
    rw::math::Rotation3D<double>  ddx(double t) const;
    double duration() const;
};

class RampInterpolatorSE3: public Interpolator<rw::math::Transform3D<double> > {
public:
    RampInterpolatorSE3(const rw::math::Transform3D<double> & start,
                          const rw::math::Transform3D<double> & end,
                          double linvellimit,double linacclimit,
                          double angvellimit,double angacclimit);

    rw::math::Transform3D<double>  x(double t) const;
    rw::math::Transform3D<double>  dx(double t) const;
    rw::math::Transform3D<double>  ddx(double t) const;
    double duration() const;
};

class RampInterpolator: public Interpolator<double> {
public:
    RampInterpolator(const double& start, const double& end, const double& vellimits, const double& acclimits);
    //RampInterpolator(const double& start, const double& end, const double& vellimits, const double& acclimits, double duration);

    double x(double t) const;
    double dx(double t) const;
    double ddx(double t) const;
    double duration() const;
};

class RampInterpolatorQ: public Interpolator<rw::math::Q> {
public:
    RampInterpolatorQ(const rw::math::Q& start, const rw::math::Q& end, const rw::math::Q& vellimits, const rw::math::Q& acclimits);
    //RampInterpolatorQ(const rw::math::Q& start, const rw::math::Q& end, const rw::math::Q& vellimits, const rw::math::Q& acclimits, double duration);

    rw::math::Q x(double t) const;
    rw::math::Q dx(double t) const;
    rw::math::Q ddx(double t) const;
    double duration() const;
};



template <class T>
class Trajectory
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double duration() const = 0;
    virtual double startTime() const = 0;
    virtual double endTime() const;

    std::vector<T> getPath(double dt, bool uniform = true);
    //virtual typename rw::common::Ptr< TrajectoryIterator<T> > getIterator(double dt = 1) const = 0;

protected:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() {};
};

%template (TrajectoryState) Trajectory<State>;
%template (TrajectoryR1) Trajectory<double>;
%template (TrajectoryR2) Trajectory<rw::math::Vector2D<double> >;
%template (TrajectoryR3) Trajectory<rw::math::Vector3D<double> >;
%template (TrajectorySO3) Trajectory<rw::math::Rotation3D<double> >;
%template (TrajectorySE3) Trajectory<rw::math::Transform3D<double> >;
%template (TrajectoryQ) Trajectory<rw::math::Q>;

%template (TrajectoryStatePtr) rw::common::Ptr<Trajectory<State> >;
%template (TrajectoryR1Ptr) rw::common::Ptr<Trajectory<double> >;
%template (TrajectoryR2Ptr) rw::common::Ptr<Trajectory<rw::math::Vector2D<double> > >;
%template (TrajectoryR3Ptr) rw::common::Ptr<Trajectory<rw::math::Vector3D<double> > >;
%template (TrajectorySO3Ptr) rw::common::Ptr<Trajectory<rw::math::Rotation3D<double> > >;
%template (TrajectorySE3Ptr) rw::common::Ptr<Trajectory<rw::math::Transform3D<double> > >;
%template (TrajectoryQPtr) rw::common::Ptr<Trajectory<rw::math::Q> >;

OWNEDPTR(Trajectory<State> )
OWNEDPTR(Trajectory<double> )
OWNEDPTR(Trajectory<rw::math::Vector2D<double> > )
OWNEDPTR(Trajectory<rw::math::Vector3D<double> > )
OWNEDPTR(Trajectory<rw::math::Rotation3D<double> > )
OWNEDPTR(Trajectory<rw::math::Transform3D<double> > )
OWNEDPTR(Trajectory<rw::math::Q> )

template <class T>
class InterpolatorTrajectory: public Trajectory<T> {
public:
    InterpolatorTrajectory(double startTime = 0);
    void add(rw::common::Ptr<Interpolator<T> > interpolator);
    void add(rw::common::Ptr<Blend<T> > blend,
             rw::common::Ptr<Interpolator<T> > interpolator);
    void add(InterpolatorTrajectory<T>* trajectory);
    size_t getSegmentsCount() const;



    //std::pair<rw::common::Ptr<Blend<T> >, rw::common::Ptr<Interpolator<T> > > getSegment(size_t index) const;
};

%template (InterpolatorTrajectoryR1) InterpolatorTrajectory<double>;
%template (InterpolatorTrajectoryR2) InterpolatorTrajectory<rw::math::Vector2D<double> >;
%template (InterpolatorTrajectoryR3) InterpolatorTrajectory<rw::math::Vector3D<double> >;
%template (InterpolatorTrajectorySO3) InterpolatorTrajectory<rw::math::Rotation3D<double> >;
%template (InterpolatorTrajectorySE3) InterpolatorTrajectory<rw::math::Transform3D<double> >;
%template (InterpolatorTrajectoryQ) InterpolatorTrajectory<rw::math::Q>;


/*
class TrajectoryFactory
{
public:
    static rw::common::Ptr<StateTrajectory> makeFixedTrajectory(const State& state, double duration);
    static rw::common::Ptr<QTrajectory> makeFixedTrajectory(const rw::math::Q& q, double duration);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectory(const TimedStatePath& path);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectory(const StatePath& path,
        const models::WorkCell& workcell);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectoryUnitStep(const StatePath& path);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const TimedQPath& path);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const rw::math::Q& speeds);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const models::Device& device);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, rw::common::Ptr<QMetric> metric);
    static rw::common::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const std::vector<double>& times);
    static rw::common::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const rw::common::Ptr<Transform3DMetric> metric);
    static rw::common::Ptr<StateTrajectory> makeEmptyStateTrajectory();
    static rw::common::Ptr<QTrajectory > makeEmptyQTrajectory();
};

*/
 
/********************************************
 * LUA functions
 ********************************************/


#if defined (SWIGLUA)
%luacode {

-- Group: Lua functions
-- Var: print_to_log
local print_to_log = true

-- Var: overrides the global print function
local global_print = print

-- Function: print
--  Forwards the global print functions to the rw.print functions
--  whenever print_to_log is defined.
function print(...)
    if print_to_log then
        for i, v in ipairs{...} do
            if i > 1 then rw.writelog("\t") end
            rw.writelog(tostring(v))
        end
        rw.writelog('\n')
    else
        global_print(...)
    end
end

-- Function:
function reflect( mytableArg )
 local mytable
 if not mytableArg then
  mytable = _G
 else
  mytable = mytableArg
 end
   local a = {} -- all functions
   local b = {} -- all Objects/Tables

 if type(mytable)=="userdata" then
   -- this is a SWIG generated user data, show functions and stuff
   local m = getmetatable( mytable )
   for key,value in pairs( m['.fn'] ) do
      if (key:sub(0, 2)=="__") or (key:sub(0, 1)==".") then
          table.insert(b, key)
      else
          table.insert(a, key)
      end
   end
   table.sort(a)
   table.sort(b)
   print("Object type: \n  " .. m['.type'])

   print("Member Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   for i,n in ipairs(b) do print("  " .. n .. "(...)") end

 else
   local c = {} -- all constants
   for key,value in pairs( mytable ) do
      -- print(type(value))
      if (type(value)=="function") then
          table.insert(a, key)
      elseif (type(value)=="number") then
          table.insert(c, key)
      else
          table.insert(b, key)
      end
   end
   table.sort(a)
   table.sort(b)
   table.sort(c)
   print("Object type: \n  " .. "Table")

   print("Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   print("Constants:")
   for i,n in ipairs(c) do print("  " .. n) end
   print("Misc:")
   for i,n in ipairs(b) do print("  " .. n) end


--  print("Metatable:")
--  for key,value in pairs( getmetatable(mytable) ) do
--      print(key);
--      print(value);
--  end

 end
 end

function help( mytable )
   reflect( mytable )
end
}
#endif


