# -*- coding: utf-8 -*-
"""
Created on Sat May 30 07:39:08 2020

@author: Robeir
"""


import speech_recognition as sr
import subprocess

# Record Audio
r = sr.Recognizer()

with sr.Microphone() as source:
    print("Please wait. Calibrating microphone...")
    # listen for 5 seconds and create the ambient noise energy level
    r.adjust_for_ambient_noise(source, duration=5)
    print("Say something!")
    audio = r.listen(source)

# Speech recognition using Google Speech Recognition
try:
# for testing purposes, we're just using the default API key
# to use another API key, use `r.recognize_google(audio,key="GOOGLE_SPEECH_RECOGNITION_API_KEY")
# instead of `r.recognize_google(audio)`
    print("You said: " + r.recognize_google(audio))
    subprocess.Popen(["espeak",r.recognize_google(audio)])

except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")

except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))