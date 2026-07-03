package com.example.rtklib_localization

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.TextView
import com.example.rtklib_localization.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)


    }

    /**
     * A native method that is implemented by the 'rtklib_localization' native library,
     * which is packaged with this application.
     */


    companion object {
        // Used to load the 'rtklib_localization' library on application startup.
        init {
            System.loadLibrary("rtklib_localization")
        }
    }
}