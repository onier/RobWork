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

#include "ShowLog.hpp"

#include <rws/RobWorkStudio.hpp>
#include <sstream>
#include <QEvent>
#include <rw/common/Log.hpp>

#define MESSAGE_ADDED_EVENT 2345

using namespace rw::graphics;
using namespace rw::common;
using namespace rws;


    //----------------------------------------------------------------------
    // Standard plugin methods

    class WriterWrapper: public LogWriter {
    public:
        typedef std::pair<std::string, QColor> Message;
        WriterWrapper(ShowLog* slog, QColor color, Log::LogIndex id):
            _slog(slog),_color(color),_id(id)
        {

        }

        virtual ~WriterWrapper(){}

        virtual void flush(){
            //_slog->flush();
        }

        /**
         * @brief Writes \b str to the log
         * @param str [in] message to write
         */
        virtual void write(const std::string& input){
            /*    	std::stringstream buf;


            if(_isNewLine){
                buf	<< "["<<_id<<"] : ";
            }
            size_t lpos = 0;
            size_t pos = str.find("\n");
            while(pos!=std::string::npos){
                buf << str.substr(pos,pos-lpos) << "\n    ";
                lpos = pos;
                pos = str.find("\n", pos+1);
            }
            buf << str.substr(lpos) << "\n";

            _slog->write(buf.str(),_color);
            */
            //_slog->write(str,_color);

			std::stringstream sstr;
			sstr << std::setw(_tabLevel)<<std::setfill(' ');
			sstr << input;

			_msgQueue.push_back( Message(sstr.str().c_str(),_color) );
            _isNewLine = false;
            QApplication::postEvent( _slog, new QEvent((QEvent::Type)MESSAGE_ADDED_EVENT) );

        }

        virtual void writeln(const std::string& input){
			std::stringstream sstr;
			sstr << std::setw(_tabLevel)<<std::setfill(' ');
			sstr << input;

            _msgQueue.push_back( Message(sstr.str().c_str(),_color) );
            //_slog->write(str,_color);
            _isNewLine = true;
            QApplication::postEvent( _slog, new QEvent((QEvent::Type)MESSAGE_ADDED_EVENT) );
        }

        std::vector< std::pair<std::string, QColor> > _msgQueue;

		void setTabLevel(int tabLevel) {
			_tabLevel = tabLevel;
		}

    private:
        ShowLog *_slog;
        QColor _color;
        Log::LogIndex _id;
        bool _isNewLine;
		int _tabLevel;

    };


QIcon ShowLog::getIcon() {
  //  Q_INIT_RESOURCE(resources);
    return QIcon(":/log.png");
}

ShowLog::ShowLog():
    RobWorkStudioPlugin("Log", getIcon() )
{
    _editor = new QTextEdit(this);
    _editor->setReadOnly(true);
    _editor->setCurrentFont( QFont("Courier New", 10) );

    _endCursor = new QTextCursor( );
    *_endCursor = _editor->textCursor();

    setWidget(_editor);  // Sets the widget on the QDockWidget

    _writers.push_back( ownedPtr( new WriterWrapper(this, Qt::black, Log::Info) ) );
    _writers.push_back( ownedPtr( new WriterWrapper(this, Qt::darkYellow, Log::Warning) ) );
    _writers.push_back( ownedPtr( new WriterWrapper(this, Qt::darkRed, Log::Error) ) );

}

ShowLog::~ShowLog()
{
    delete _endCursor;
}

bool ShowLog::event(QEvent *event){
    if(event->type()==MESSAGE_ADDED_EVENT){
        BOOST_FOREACH(rw::common::Ptr<WriterWrapper> writer, _writers){
            for(unsigned int i=0;i<writer->_msgQueue.size();i++){
                write(writer->_msgQueue[i].first, writer->_msgQueue[i].second);
            }
            writer->_msgQueue.clear();
        }
        return true;
    } else {
        event->ignore();
    }

    return RobWorkStudioPlugin::event(event);
}

void ShowLog::open(rw::models::WorkCell* workcell)
{
	if( workcell==NULL )
		return;

	log().info() << "WorkCell opened: " << StringUtil::quote(workcell->getName()) << std::endl;
}

void ShowLog::close()
{
	log().info() << "WorkCell closed!" << std::endl;
}

void ShowLog::receiveMessage(
    const std::string& plugin,
    const std::string& id,
    const rw::common::Message& msg)
{
	RW_WARN("Deprecated function, use log().info() << \"your string\" instead");
    /*std::stringstream buf;
    buf
        << id
        << " ["
        << plugin
        << "] "
        << "("
        << msg.getFile()
        << ":"
        << msg.getLine()
        << "):\n"
        << msg.getText()
        << "\n";
    _editor->append(buf.str().c_str());
    */
}


void ShowLog::write(const std::string& str, const QColor& color){


    _editor->setTextCursor(*_endCursor);
    _editor->setTextColor( color );

	//_endCursor->insertText(str.c_str());
	_editor->insertPlainText( str.c_str() );
	*_endCursor = _editor->textCursor();
	//_editor->insertPlainText(  );
}

void ShowLog::frameSelectedListener(rw::kinematics::Frame* frame) {
	if(frame==NULL)
		return;
	log().info() << "Frame selected: " << frame->getName() << std::endl;
}

void ShowLog::initialize() {
    setParent(getRobWorkStudio());
    getRobWorkStudio()->frameSelectedEvent().add(
    		boost::bind(&ShowLog::frameSelectedListener, this, _1), this);

    log().setWriter(Log::Info, _writers[0]);
    //log().setWriter(Log::Warning, _writers[1]);
    //log().setWriter(Log::Error, _writers[2]);

}


//----------------------------------------------------------------------

#ifndef RWS_USE_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Log, ShowLog)
#endif