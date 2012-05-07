/*
 * SensorMgr.cpp
 *
 *  Created on: 2012-05-01
 *      Author: siarheig
 */

#include "SensorMgr.h"

#include <StelApp.hpp>
#include <StelCore.hpp>
#include <StelMovementMgr.hpp>

SensorMgr::SensorMgr() {
	setObjectName("SensorMgr");

}

SensorMgr::~SensorMgr() {

}

void SensorMgr::init() {
	_eventThread = new SensorEventThread();
	_eventThread->start();
	StelCore* core = StelApp::getInstance().getCore();
	connect(_eventThread, SIGNAL(geo_changed(double, double)), core, SLOT(setGPSLocation(double, double)));
	StelMovementMgr* mMgr = core->getMovementMgr();

	connect(_eventThread, SIGNAL(changed(double,double,float,float)), mMgr, SLOT(setSensorDirection(double,double,float,float)));
}

void SensorMgr::deinit() {
	_eventThread->terminate();
	delete _eventThread;
}

void SensorMgr::update(double deltaTime) {
}






