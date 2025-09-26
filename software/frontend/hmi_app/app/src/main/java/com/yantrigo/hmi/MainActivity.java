package com.yantrigo.hmi;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import java.io.File;

import org.json.JSONObject;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import android.os.Handler;




public class MainActivity extends Activity {

    // UI elements
    private TextView tvStatus, tvSpeak, tvAction;
    private ImageView img;
    private Button btnMotion, btnNeck, btnHands, btnSpeech, btnEyes;
    private Button btnNamaste, btnDance, btnWishHappyBirthday, btnStop, btnShutdown;
    
    private Handler handler = new Handler();
private Runnable jsonUpdater;
private static final int REFRESH_INTERVAL = 5000; // 5 seconds
private String jsonPath = "/sdcard/Download/command.json";

    // Version with completed parameter
private void updateResponseJSON(String action, String completed) {
    try {
        JSONObject json = new JSONObject();
        json.put("action", action);
        json.put("action_completed", completed);

        String path = "/sdcard/Download/response.json";
        FileOutputStream fos = new FileOutputStream(path);
        fos.write(json.toString().getBytes("UTF-8"));
        fos.close();

    } catch (Exception e) {
        e.printStackTrace();
        tvStatus.setText("Status: Error writing response.json");
    }
}

// Version without completed parameter (defaults to "false")
private void updateResponseJSON(String action) {
    updateResponseJSON(action, "false");
}

    private void startJsonAutoRefresh() {
    jsonUpdater = new Runnable() {
        @Override
        public void run() {
            loadCommandJSON(jsonPath);  // reload JSON
            handler.postDelayed(this, REFRESH_INTERVAL); // schedule next update
        }
    };
    handler.post(jsonUpdater); // start first update
}

    
    private void loadCommandJSON(String path){
    try {
        FileInputStream fis = new FileInputStream(path);
        int size = fis.available();
        byte[] buffer = new byte[size];
        fis.read(buffer);
        fis.close();

        String jsonString = new String(buffer, "UTF-8");
        JSONObject json = new JSONObject(jsonString);

        String status = json.optString("status", "Status Unknown");
        String message = json.optString("message", "Hello");
        String image = json.optString("image", "humanoid.png");
        String action_completed = json.optString("action_completed", "false");
        
        tvStatus.setText("Status: " + status);
        tvSpeak.setText("You Speak: " + message);
        // Load new image for Eyes Test
            String newImagePath = "/sdcard/Download/"+image;
            File imgFile = new File(newImagePath);
            if(imgFile.exists()){
                Bitmap bitmap = BitmapFactory.decodeFile(imgFile.getAbsolutePath());
                img.setImageBitmap(bitmap);
            }
       if(action_completed.equals("true")){
       		updateResponseJSON("none", "true");
       		tvAction.setText("Current Action: " + "none");
       }

    } catch (Exception e){
        tvStatus.setText("Status: Error reading command.json");
        tvSpeak.setText("You Speak: Error");
        e.printStackTrace();
    }
}
    
    @Override
protected void onCreate(Bundle savedInstanceState){
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    // Initialize UI elements
    tvStatus = (TextView) findViewById(R.id.tvStatus);
    tvSpeak = (TextView) findViewById(R.id.tvSpeak);
    tvAction = (TextView) findViewById(R.id.tvAction);

    img = (ImageView) findViewById(R.id.myImageView);
    loadImage("/sdcard/Download/humanoid.png");

    btnMotion = (Button) findViewById(R.id.btnMotion);
    btnNeck = (Button) findViewById(R.id.btnNeck);
    btnHands = (Button) findViewById(R.id.btnHands);
    btnSpeech = (Button) findViewById(R.id.btnSpeech);
    btnEyes = (Button) findViewById(R.id.btnEyes);

    btnNamaste = (Button) findViewById(R.id.btnNamaste);
    btnDance = (Button) findViewById(R.id.btnDance);
    btnWishHappyBirthday = (Button) findViewById(R.id.btnWishHappyBirthday);
    btnStop = (Button) findViewById(R.id.btnStop);
    btnShutdown = (Button) findViewById(R.id.btnShutdown);

    // Load initial JSON
    loadCommandJSON(jsonPath);

    // Start auto-refresh every 5 seconds
    startJsonAutoRefresh();

    // Set button listeners
    setupButtonListeners();
}


    // Load image from storage
    private void loadImage(String path){
        File imgFile = new File(path);
        if(imgFile.exists()){
            Bitmap bitmap = BitmapFactory.decodeFile(imgFile.getAbsolutePath());
            img.setImageBitmap(bitmap);
        } else {
            tvStatus.setText("Status: Image not found!");
        }
    }

    // Setup all button click events
private void setupButtonListeners() {

    // TEST ACTIONS
    btnMotion.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Motion Test";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);

            // Status and Speak are now updated from command.json
            // TODO: Add your motion logic here
        }
    });

    btnNeck.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Neck Test";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
            // TODO: Add your neck logic here
        }
    });

    btnHands.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Hands Test";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
        }
    });

    btnSpeech.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Speech Test";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
            // TODO: Add TTS or speech logic here
        }
    });

    btnEyes.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Eyes Test";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);

 

            // TODO: Add eyes movement logic here
        }
    });

    // COMMON ACTIONS
    btnNamaste.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Namaste";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
            // TODO: Trigger Namaste action on robot
        }
    });

    btnDance.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Dance";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
            // TODO: Trigger dance sequence
        }
    });

    btnWishHappyBirthday.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Wish Happy Birthday";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
            // TODO: Trigger birthday action
        }
    });

    btnStop.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Stop";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
            // TODO: Stop all robot actions
        }
    });

    btnShutdown.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            String currentAction = "Shutdown";
tvAction.setText("Current Action: " + currentAction);
updateResponseJSON(currentAction);
            // TODO: Send shutdown command to robot or close app
        }
    });
}

}

