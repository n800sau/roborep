package au.n800s.robo.model.3pi;

public class BootReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context context, Intent intent) {
        Intent startServiceIntent = new Intent(context, IOIORemoteService.class);
        context.startService(startServiceIntent);
    }
}

