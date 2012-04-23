package au.n800s.robo.common;

import IRoboCallback;

// Declare the interface.
interface IRobo {
  String getName();
  String getVersion();
  int registerCallback(in int id, in IRoboCallback cb);
  int unregisterCallback(in int id);
}
