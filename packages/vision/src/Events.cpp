/*
 * Copyright (C) 2008 Robotics at Maryland
 * Copyright (C) 2008 Joseph Lisee <jlisee@umd.edu>
 * All rights reserved.
 *
 * Author: Joseph Lisee jlisee@umd.edu>
 * File:  packages/math/src/Events.cpp
 */

// Project Includes
#include "core/include/Feature.h"
#include "vision/include/Events.h"

RAM_CORE_EVENT_TYPE(ram::vision::EventType, LIGHT_FOUND);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, LIGHT_LOST);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, LIGHT_ALMOST_HIT);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, PIPE_FOUND);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, PIPE_CENTERED);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, PIPE_LOST);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, GATE_FOUND);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, BIN_FOUND);
RAM_CORE_EVENT_TYPE(ram::vision::EventType, BIN_LOST);

// This section is only needed when we are compiling the wrappers
// This registers converters to work around some issues with Boost.Python
// automatic down casting
#if defined(RAM_WITH_WRAPPERS)

#include "core/include/EventConverter.h"

static ram::core::SpecificEventConverter<ram::vision::RedLightEvent>
RAM_VISION_REDLIGHTEVENT;

static ram::core::SpecificEventConverter<ram::vision::PipeEvent>
RAM_VISION_PIPEEVENT;

#endif // RAM_WITH_WRAPPERS
