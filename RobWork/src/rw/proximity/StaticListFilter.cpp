/*
 * StaticListFilter.cpp
 *
 *  Created on: Apr 23, 2009
 *      Author: jimali
 */

#include "StaticListFilter.hpp"

StaticListFilter::StaticListFilter(rw::models::WorkCellPtr workcell)
{

}

StaticListFilter::StaticListFilter(rw::models::WorkCellPtr workcell, const CollisionSetup& setup)
{

}

StaticListFilter::StaticListFilter(rw::models::WorkCellPtr workcell, CollisionStrategyPtr strategy, const CollisionSetup& setup)
{

}

void StaticListFilter::include(const kinematics::FramePair& framepair)
{

}

void StaticListFilter::include(kinematics::Frame* frame)
{

}

void StaticListFilter::exclude(const kinematics::FramePair& framepair)
{

}

void StaticListFilter::exclude(kinematics::Frame* frame)
{

}

	//////// interface inherited from BroadPhaseStrategy
void StaticListFilter::reset(const rw::kinematics::State& state)
{

}

void StaticListFilter::update(const rw::kinematics::State& state)
{

}

const rw::kinematics::FramePair& StaticListFilter::next()
{

}

bool StaticListFilter::hasNext()
{

}

CollisionSetup& StaticListFilter::getCollisionSetup()
{

}

void StaticListFilter::addModel(const rw::kinematics::Frame* frame, const rw::geometry::Geometry& geom)
{

}

void StaticListFilter::removeModel(const rw::kinematics::Frame* frame, const std::string& geoid)
{

}

