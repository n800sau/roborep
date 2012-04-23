package au.n800s.robo.common;

import IRoboCallback;

// Declare the interface.
interface IRobo {
  String getName();
  String getVersion();
  Float getBattery();

//      AppId = ctx.getPackageName();

//  int registerCallback(in String appId, in IRoboCallback cb);
//  int unregisterCallback(in String appId);
}
