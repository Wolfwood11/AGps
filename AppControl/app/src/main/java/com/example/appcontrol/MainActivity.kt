package com.example.appcontrol

import android.os.Bundle
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        findViewById<Button>(R.id.btnTrack).setOnClickListener {
            UdpClient.sendCommand("SETMODE TRACK")
        }
        findViewById<Button>(R.id.btnSpeed).setOnClickListener {
            UdpClient.sendCommand("SETMODE SPD")
        }
        findViewById<Button>(R.id.btnSession).setOnClickListener {
            UdpClient.sendCommand("START SESSION")
        }
        UdpClient.startListening(this)
    }
}
