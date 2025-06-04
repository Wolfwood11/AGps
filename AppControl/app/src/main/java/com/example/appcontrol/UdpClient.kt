package com.example.appcontrol

import android.content.Context
import java.io.File
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.concurrent.thread

object UdpClient {
    private const val PORT = 4210
    private const val HOST = "192.168.4.1"
    private var socket: DatagramSocket? = null
    private var logFile: File? = null

    fun sendCommand(command: String) {
        thread {
            try {
                val data = buildControlMessage(command)
                val packet = DatagramPacket(data, data.size, InetAddress.getByName(HOST), PORT)
                getSocket().send(packet)
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }

    fun startListening(context: Context) {
        thread {
            val buf = ByteArray(256)
            while (true) {
                try {
                    val packet = DatagramPacket(buf, buf.size)
                    getSocket().receive(packet)
                    processPacket(packet.data, packet.length, context)
                } catch (_: Exception) {
                }
            }
        }
    }

    private fun processPacket(data: ByteArray, len: Int, context: Context) {
        val bb = ByteBuffer.wrap(data, 0, len).order(ByteOrder.LITTLE_ENDIAN)
        val timestamp = bb.int.toLong() and 0xffffffffL
        val type = bb.int
        if (type == 1) { // Geo
            val nmeaBytes = ByteArray(len - 8)
            bb.get(nmeaBytes)
            val line = String(nmeaBytes).trim { it <= '\u0000' }
            logFile?.appendText(line)
        } else if (type == 3) { // SessionStart
            logFile = File(context.getExternalFilesDir(null), "session_${timestamp}.nmea")
        }
    }

    private fun buildControlMessage(command: String): ByteArray {
        val bb = ByteBuffer.allocate(8 + 64).order(ByteOrder.LITTLE_ENDIAN)
        bb.putInt(0) // timestamp not used
        bb.putInt(2) // Control type
        val bytes = command.toByteArray()
        val arr = ByteArray(64)
        System.arraycopy(bytes, 0, arr, 0, bytes.size.coerceAtMost(64))
        bb.put(arr)
        return bb.array()
    }

    private fun getSocket(): DatagramSocket {
        if (socket == null) {
            socket = DatagramSocket()
        }
        return socket!!
    }
}
