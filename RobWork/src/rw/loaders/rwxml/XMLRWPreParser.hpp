/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_LOADERS_XMLPREPARSER_HPP
#define RW_LOADERS_XMLPREPARSER_HPP



#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/phoenix1.hpp>
#include <boost/spirit/include/classic_common.hpp>
#include <boost/spirit/include/classic_ast.hpp>
#include <boost/spirit/include/classic_parse_tree.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/spirit/include/classic_functor_parser.hpp>
#include <boost/spirit/include/classic_exceptions.hpp>
#include <boost/spirit/include/classic_actor.hpp>
#include <iostream>
#include <cassert>
#include <vector>

#include <sstream>

#include <boost/numeric/ublas/vector.hpp>

#include "DependencyGraph.hpp"

#include <rw/common/StringUtil.hpp>

namespace rw {
namespace loaders {
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief Pre-parser for the XML RW format. All include, define, comments and use
	 * elements are handled.
	 */
    class XMLRWPreParser {
    public:

        /**
         * @brief Pre-parser for the XML RW format. All include, define, comments and use
         * elements are handled.
         *
         * @param filename [in] name of file to parse
         * @param output [out] the data result of parsing the file
         * @param filemap [out] the position to file info result of parsing the file
         *
         * @return true if pasing was a succes, false otherwise
         */
        static bool parse(
            const std::string& filename,
            std::vector<char> &output,
            std::vector< std::pair<size_t,boost::spirit::classic::file_position> > &filemap);

        /**
         * @brief Pre-parses for the XML RW format. All include, define, comments and use
         * elements are handled.
		 *
         * @param filename [in] name of file to parse
         * @param output [out] the data result of parsing the file
         * @param filemap [out] the position to file info result of parsing the file
         * @param graph [in] a dependency graph between file includes
         *
         * @return true if pasing was a succes, false otherwise
         */
        static bool parse(
            const std::string& filename,
            std::vector<char> &output,
            std::vector< std::pair<size_t,boost::spirit::classic::file_position> > &filemap,
            DependencyGraph &graph );

    };
    /*@}*/
}
}
#endif /*RW_LOADERS_XMLPREPARSER_HPP*/

