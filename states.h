
#pragma once

// has to be here and not in .ino due to arduino magic
enum State {
  IDLE,
  RAMPING_UP,
  ACTIVE
  // TODO: ACTIVE_AFTER_LOCKDOWN (guarantee minimal active time), RAMPING_DOWN
};

