
"use strict";

let SetSpeedOverride = require('./SetSpeedOverride.js')
let SetWorkpiece = require('./SetWorkpiece.js')
let SetPTPCartesianSpeedLimits = require('./SetPTPCartesianSpeedLimits.js')
let SetPTPJointSpeedLimits = require('./SetPTPJointSpeedLimits.js')
let TimeToDestination = require('./TimeToDestination.js')
let SetEndpointFrame = require('./SetEndpointFrame.js')
let SetSmartServoLinSpeedLimits = require('./SetSmartServoLinSpeedLimits.js')
let ConfigureControlMode = require('./ConfigureControlMode.js')
let SetSmartServoJointSpeedLimits = require('./SetSmartServoJointSpeedLimits.js')

module.exports = {
  SetSpeedOverride: SetSpeedOverride,
  SetWorkpiece: SetWorkpiece,
  SetPTPCartesianSpeedLimits: SetPTPCartesianSpeedLimits,
  SetPTPJointSpeedLimits: SetPTPJointSpeedLimits,
  TimeToDestination: TimeToDestination,
  SetEndpointFrame: SetEndpointFrame,
  SetSmartServoLinSpeedLimits: SetSmartServoLinSpeedLimits,
  ConfigureControlMode: ConfigureControlMode,
  SetSmartServoJointSpeedLimits: SetSmartServoJointSpeedLimits,
};
