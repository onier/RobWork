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

#include "StateStructure.hpp"

#include "FixedFrame.hpp"
#include "State.hpp"
#include "QState.hpp"
#include "StateSetup.hpp"
#include "TreeState.hpp"

#include <rw/math/Transform3D.hpp>

using rw::math::Transform3D;
using namespace rw::kinematics;

StateStructure::StateStructure():
    _version(0),
    _root(NULL),
    _stateSetupUniqueId(0)
{
    _root = new FixedFrame("WORLD",Transform3D<>::identity());
    // now add the state data of the frame
    //_frames.push_back(_root);
    addDataInternal(_root);
    _frames[_root->getID()] = _root;
    // and setup the static frame-parent relationship
    _root->setParent(NULL);
    _frameIdxMap[_root->getName()] = _root->getID();

    updateDefaultState();
}

StateStructure::~StateStructure()
{
    // Make Sure that shared Frame pointers are not attached to this statestructure
    for(size_t i = 0; i < _frames.size(); i++) {
        //std::cout << "Frame[" << i+1 << "/" << _frames.size() << "]" << std::endl;
        if (_frames[i] != nullptr) {
            _frames[i]->setID(-1,NULL);
            if(_frames[i]->getParent() != NULL) {
                _frames[i]->getParent()->removeChild(_frames[i]);
                _frames[i]->setParent(NULL);
            }
        }
    }
}

bool StateStructure::has(const StateData * const data){
    const int id = data->getID();
    return (id>=0 && id<getMaxID() && _allDatas[id]!=NULL);
}

void StateStructure::addData(StateData * const data){

    addDataInternal(data);

    updateDefaultState();

    _stateDataAddedEvent.fire(data);
}

void StateStructure::addData(const boost::shared_ptr<StateData> data){
    addDataInternal(data);

    updateDefaultState();

    _stateDataAddedEvent.fire(data.get());
}

void StateStructure::addDataInternal(StateData * const data){
    // frame must not be in tree allready
    if(has(data))
        RW_THROW("The StateData has allready been added! State data can only be added once.");

    _version++;
    const int id = allocateDataID();
    data->setID(id, this);
    // There is no turning back. Ownership is forever taken.
    boost::shared_ptr<StateData> sharedData( data );
    _allDatas.at(id) = sharedData;
    _currDatas.at(id) = sharedData;
    _stateIdxMap[sharedData->getName()] = id;
    // make room in the initial state data array
    // now frame must be in the tree
    RW_ASSERT(has(data));
    // lastly update the default state
}

void StateStructure::addDataInternal(const boost::shared_ptr<StateData> data){
    // frame must not be in tree allready
    if(has(data.get())){
        RW_THROW("The StateData has allready been added! State data can only be added once. " << data->getName() << " ID: " << data->getID());
    }
    _version++;
    const int id = allocateDataID();
    data->setID(id, this);
    
    boost::shared_ptr<StateData> sharedData = data;
    _allDatas.at(id) = sharedData;
    _currDatas.at(id) = sharedData;
    _stateIdxMap[sharedData->getName()] = id;
    // make room in the initial state data array
    // now frame must be in the tree
    RW_ASSERT(has(data.get()));
    // lastly update the default state
}

void StateStructure::addFrame(Frame::Ptr frame, Frame::Ptr parent){
    // both frame and parent must be well defined
    if(frame == NULL)
        RW_THROW("Input frame must not be NULL!");
    if(parent == NULL) {
        parent = getRoot();
    }
    RW_ASSERT(frame && parent);

    // and parent must exist in the tree, but not the frame
    if( has(frame.get()) ){
        RW_THROW("The frame has already been added to the state structure!");
    }

    if( !has(parent.get()) ){
        RW_THROW("The parent is not part of the state structure and should be added before frame is!");
    }

    // and lastly we must check if the frame has been added to other StateStructure
    if( frame->getID()!=-1  ){
        RW_THROW("The frame has already been added to another state structure");
    }
    // check if frame name is unique
    if( findFrame(frame->getName())!=NULL )
        RW_THROW("Frame name is not unique: "<< frame->getName());

    // update the parent child relationships
    frame->setParent(parent.get());
    parent->addChild(frame.get());

    // now add the state data of the frame
    if(frame.isShared()) {
        addDataInternal(frame.getSharedPtr());
    } else {
        addDataInternal(frame.get());
    }

    // add the frame to the framelist
    _frames[frame->getID()] = frame.get();
    // remember to add the frame to the frameIdxMap
    _frameIdxMap[frame->getName()] = frame->getID();

    updateDefaultState();

    // and in the end cleanup
    cleanup();

    _stateDataAddedEvent.fire(frame.get());
}

void StateStructure::addDAF(Frame::Ptr frame, Frame::Ptr parent){
    // both frame and parent must be well defined
    RW_ASSERT(frame && parent);
    // and parent must exist in the tree, but not the frame
    RW_ASSERT(!has(frame.get()) && has(parent.get()));
    // and lastly we must check if the frame has been added to other StateStructure
    RW_ASSERT( frame->getID()==-1 );
    // check if frame name is unique
    if(findFrame(frame->getName())!=NULL)
        RW_THROW("Frame name is not unique!");

    // now add the state data of the frame
    if(frame.isShared()) {
        addDataInternal(frame.getSharedPtr());
    } else {
        addDataInternal(frame.get());
    }
    // push it to the frame list and DAF list
    _DAFs.push_back(frame.get());
    _frames[frame->getID()] = frame.get();

    updateDefaultState();

    // and insert the dynamic frame-parent relationship in the default state
    _defaultState.getTreeState().attachFrame(frame.get(),parent.get());
    // remember to add the frame to the frameIdxMap
    _frameIdxMap[frame->getName()] = frame->getID();
    // and in the end cleanup
    cleanup();

    _stateDataAddedEvent.fire(frame.get());
}

void StateStructure::remove(StateData *data) {
    RW_ASSERT(data);
    RW_ASSERT(data->getID()>0);
    _version++;
    int id = data->getID();
    if( _frames[id]!=NULL ){
        // check if frame has staticly connected children
        Frame::iterator_pair iter = _frames[id]->getChildren();
        if( iter.first!=iter.second )
            RW_THROW("Frame has staticly connected children and therefore cannot be removed from tree!");
        // make sure the parent frame gets any static connections deleted to the frame
        Frame *parent = _frames[id]->getParent();
        if(parent!=NULL)
            parent->removeChild(_frames[id]);

        // remember to remove the from the frameIdxMap
        _frameIdxMap.erase(_frames[id]->getName());
    }
    // setting data in current data list to null
    _currDatas[id] = boost::shared_ptr<StateData>();
    // update default state
    // the dynamicly attached frames will automaticly be attached to world
    updateDefaultState();
    // perform cleanup
    cleanup(id);
    _stateDataRemovedEvent.fire(data);
}

State StateStructure::upgradeState(const State& oldState)
{
    State state = _defaultState;
    state.copy( oldState );
    return state;
}

const State& StateStructure::getDefaultState() const
{
    return _defaultState;
}

void StateStructure::setDefaultState(const State &state)
{
    // check if state version
    _defaultState.copy(state);
}

void StateStructure::cleanup(int ID) {
    // first run through StateSetup list and remove all state setups that are not
    // used anymore
    StateSetupList::iterator iter = _setups.begin();
    while(iter!=_setups.end()){
        if((*iter).use_count()>1){
            ++iter;
        } else {
            //std::cout << "Erasing StateSetup v." << (*iter)->getVersion() << std::endl;
            // erase the setup from the list
            iter = _setups.erase(iter);
        }
    }

    // NEXT run through statedata list and remove all data that
    // is not pointed to by anything else.
    // remember to also remove instances from the daf and frame list
    for(boost::shared_ptr<StateData>& data : _allDatas) {
        if(data != NULL){
            if(data.use_count()==1 || data->getID() == ID){
                // delete the data and put it on available list
                const int id = data->getID();
                // test if data is a daf
                for(size_t i=0;i<_DAFs.size();i++){
                    if(_DAFs[i]!=NULL && _DAFs[i]->getID()==id){
                        _DAFs[i] = NULL;
                        break;
                    }
                }
                // if its a frame then remove it from its parent
                
                if (_frames[ id ] != NULL ){
                    Frame *frame = _frames [id];
                    if(frame->getParent()!=NULL){
                        frame->getParent()->removeChild( frame );
                    }              
                    _frames[id] = NULL;
                    data->setID(-1,NULL);
                }
                data = boost::shared_ptr<StateData>();
                _availableDataIds.push_back(id);
            }
        }
    }


}

void StateStructure::updateDefaultState(){
    // first create a StateSetup
    boost::shared_ptr<StateSetup> setup(new StateSetup(_version,*this, _currDatas));
    _setups.push_back(setup);
    // construct qstate and tree state and then a new State
    QState qstate(setup);
    TreeState tstate(setup);
    State newState( qstate, tstate, _stateSetupUniqueId);
    _stateSetupUniqueId++;
    // copy the default values into newState
    newState.copy( _defaultState );
    // all caches that are null should be updated with their default cache
    for(boost::shared_ptr<StateData>& data : _currDatas) {
        if(data==NULL)
            continue;
        if(data->hasCache()){
            if( data->getCache(newState) == NULL )
                data->setCache( data->getDefaultCache()->clone(), newState );
        }
    }

    _defaultState = newState;
}

int StateStructure::allocateDataID()
{
    // check if any ids are currently available
    if (_availableDataIds.empty()) {
        // if not then generate a new id
        const int id = (int)_allDatas.size();
        _allDatas.push_back(boost::shared_ptr<StateData>());
        _currDatas.push_back(boost::shared_ptr<StateData>());
        _frames.push_back(NULL);
        RW_ASSERT(_allDatas.size() == _currDatas.size());
        return id;
    } else {
        const int id = _availableDataIds.back();
        _availableDataIds.pop_back();
        return id;
    }
}

Frame* StateStructure::findFrame(const std::string& name) const {
    typedef FrameIdxMap::const_iterator I;
    const I pos = _frameIdxMap.find(name);
    if (pos == _frameIdxMap.end())
        return NULL;
    int idx = pos->second;
    if( _allDatas[idx]==NULL )
        return NULL;
    return _frames[pos->second];
}

boost::shared_ptr<StateData> StateStructure::findData(const std::string& name) const {
	// TODO: make a map for faster searching of state data

	typedef std::map<std::string,int>::const_iterator I;
    const I pos = _stateIdxMap.find(name);
    if (pos == _stateIdxMap.end())
        return boost::shared_ptr<StateData>();
    int idx = pos->second;
    return _allDatas[idx];
}
