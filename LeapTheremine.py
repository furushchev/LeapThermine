#!/usr/bin/python
# -*- coding:utf-8 -*-

import Leap, sys
from time import time
import wave
import struct
import math
import numpy as np
import pyaudio


class LeapListener(Leap.Listener):
    pyaudio_ = pyaudio.PyAudio()
    prev_time = time()
    samplingRate = 44100
    stream_ = None

    def createSineWave (self, A, f0, length):
        """振幅A
           基本周波数f0
           長さlength秒の正弦波を作成して返す"""
        fs = self.samplingRate
        data = []
        for n in np.arange(length * fs):
            s = A * np.sin(2 * np.pi * f0 * n / fs)
            if s > 1.0:  s = 1.0
            if s < -1.0: s = -1.0
            data.append(s)
        # [-32768, 32767]
        data = [int(x * 32767.0) for x in data]
        # to binary
        data = struct.pack("h" * len(data), *data)
        return data

    def play (self, data, bit):
        self.stream_.write(data[:-3])

    def on_init(self, controller):
        self.stream_ = self.pyaudio_.open(format=pyaudio.paInt16,
                                          channels=1,
                                          rate=int(self.samplingRate),
                                          output= True)
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        print "Disconnected"

    def on_exit(self, controller):
        self.stream_.close()
        self.pyaudio_.terminate()
        print "Exited"

    def on_frame(self, controller):
        frame = controller.frame()
        curr_time = time()
        diff_time = curr_time - self.prev_time
        print "fps: %f" % (1./diff_time)
        self.prev_time = curr_time

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        if not frame.hands.empty:
            hand = frame.hands[0]

            print "Hand sphere radius: %f mm, palm position: %s" % (
                  hand.sphere_radius, hand.palm_position)

            freq = hand.palm_position.y - 50.0
            if freq < 0.0: freq = 0.0
            elif freq > 350: freq = 350.0
            freq = 1.0 - (freq / 350.0)
            freq = math.exp((freq * 6.9003) + 2.31) # 2.31 - 9.2103 -> 10 - 10000

            amplitude = 0.5
            if len(frame.hands) > 1:
                py = frame.hands[1].palm_position.y - 50.0
                if py < 0.0: py = 0.0
                elif py > 350: py = 350.0
                amplitude = 1.0 - (py / 350.0)

            print "freq: %f, amplitude: %f" % (freq, amplitude)

            tone = self.createSineWave(amplitude, freq, diff_time)
            self.play(tone, 16)

            normal = hand.palm_normal
            direction = hand.direction

            print "Hand pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG)

        if not (frame.hands.empty and frame.gestures().empty):
            print ""

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

if __name__ == '__main__':
    listener = LeapListener()
    controller = Leap.Controller()
    controller.add_listener(listener)
    print "Press Enter to quit..."
    sys.stdin.readline()
    controller.remove_listener(listener)
