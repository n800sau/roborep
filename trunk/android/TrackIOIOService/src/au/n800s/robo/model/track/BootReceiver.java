package au.n800s.robo.model.track;

public class BootReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context context, Intent intent) {
        Intent startServiceIntent = new Intent(context, IOIORemoteService.class);
        context.startService(startServiceIntent);
    }
}

