// commander.h

#ifndef COMMANDER_H
#define COMMANDER_H

#include "fishcode/VisOffset.h"
#include "fishcode/SetSwimMode.h"

#define SWIM_MODE_COUNT         6
#define SWIM_MODE_IDLE          0 
#define SWIM_MODE_BTTN_DRIVE    1
#define SWIM_MODE_RC_MANUAL     2
#define SWIM_MODE_VIS_SERVO     3
#define SWIM_MODE_CIRCLE		4
#define SWIM_MODE_TEST			5

using namespace fishcode;

void commandSwimMode(int mode);
void cb_SetSwimModeRqst(const SetSwimMode::ConstPtr& msg);
void cb_VisOffset(const VisOffset::ConstPtr& msg);

#endif