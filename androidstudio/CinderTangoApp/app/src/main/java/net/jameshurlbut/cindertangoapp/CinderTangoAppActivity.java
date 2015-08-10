package net.jameshurlbut.cindertangoapp;

import android.content.Intent;

import org.libcinder.app.CinderNativeActivity;

public class CinderTangoAppActivity extends CinderNativeActivity {
    static final String TAG = "CinderTangoAppActivity";

    public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
    public static final String EXTRA_VALUE_VIO = "MOTION_TRACKING_PERMISSION";
    public static final String EXTRA_VALUE_VIOADF = "ADF_LOAD_SAVE_PERMISSION";

    // The unique request code for permission intent.
    private static final int PERMISSION_REQUEST_CODE = 0;

    protected void onResume(){
        super.onResume();

        Intent intent1 = new Intent();
        intent1.setAction("android.intent.action.REQUEST_TANGO_PERMISSION");
        intent1.putExtra(EXTRA_KEY_PERMISSIONTYPE, EXTRA_VALUE_VIO);
        startActivityForResult(intent1, 0);

        Intent intent2 = new Intent();
        intent2.setAction("android.intent.action.REQUEST_TANGO_PERMISSION");
        intent2.putExtra(EXTRA_KEY_PERMISSIONTYPE, EXTRA_VALUE_VIOADF);
        startActivityForResult(intent2, 0);

    }
}
