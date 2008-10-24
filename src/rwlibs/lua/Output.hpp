/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RWLIBS_LUA_OUTPUT_HPP
#define RWLIBS_LUA_OUTPUT_HPP

/**
 * @file Output.hpp
 */

namespace rwlibs { namespace lua {

    /** @addtogroup lua */
    /*@{*/

    /**
       @brief Output-buffer for Lua programs.
    */
    class Output {
    public:
        /**
           @brief Handle user-output \b str.
        */
        virtual void write(const std::string& str) = 0;

        /**
           @brief Destructor
         */
        virtual ~Output() {}

    private:
        Output(const Output&);
        Output& operator=(const Output&);

    protected:
        /**
           @brief Constructor
         */
        Output() {}
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
