/*
 * This file is auto-generated.  DO NOT MODIFY.
 * Original file: /mnt/sda5/n800s/work/sourceforge/robotarr-code/android/RoboCommon/src/au/n800s/robo/common/IRoboCallback.aidl
 */
package au.n800s.robo.common;
// Declare the interface.

public interface IRoboCallback extends android.os.IInterface
{
/** Local-side IPC implementation stub class. */
public static abstract class Stub extends android.os.Binder implements au.n800s.robo.common.IRoboCallback
{
private static final java.lang.String DESCRIPTOR = "au.n800s.robo.common.IRoboCallback";
/** Construct the stub at attach it to the interface. */
public Stub()
{
this.attachInterface(this, DESCRIPTOR);
}
/**
 * Cast an IBinder object into an au.n800s.robo.common.IRoboCallback interface,
 * generating a proxy if needed.
 */
public static au.n800s.robo.common.IRoboCallback asInterface(android.os.IBinder obj)
{
if ((obj==null)) {
return null;
}
android.os.IInterface iin = (android.os.IInterface)obj.queryLocalInterface(DESCRIPTOR);
if (((iin!=null)&&(iin instanceof au.n800s.robo.common.IRoboCallback))) {
return ((au.n800s.robo.common.IRoboCallback)iin);
}
return new au.n800s.robo.common.IRoboCallback.Stub.Proxy(obj);
}
public android.os.IBinder asBinder()
{
return this;
}
@Override public boolean onTransact(int code, android.os.Parcel data, android.os.Parcel reply, int flags) throws android.os.RemoteException
{
switch (code)
{
case INTERFACE_TRANSACTION:
{
reply.writeString(DESCRIPTOR);
return true;
}
}
return super.onTransact(code, data, reply, flags);
}
private static class Proxy implements au.n800s.robo.common.IRoboCallback
{
private android.os.IBinder mRemote;
Proxy(android.os.IBinder remote)
{
mRemote = remote;
}
public android.os.IBinder asBinder()
{
return mRemote;
}
public java.lang.String getInterfaceDescriptor()
{
return DESCRIPTOR;
}
}
}
}
