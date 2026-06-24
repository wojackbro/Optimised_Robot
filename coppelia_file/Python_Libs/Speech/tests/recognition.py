#!/usr/bin/env python3
# NOTE: this example requires PyAudio because it uses the Microphone class

import time
import speech_recognition as sr

check_words = [ 'stop', 'fermati', 'basta' ]
global ex_check

# this is called from the background thread
def callback(recognizer, audio):
    # received audio data, now we'll recognize it using Google Speech Recognition
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        rec_text = recognizer.recognize_google(audio, language="it-IT")
        print(f"\nHAI DETTO QUESTO: {rec_text}\n")
        for w in check_words:
            if rec_text.casefold() == w.casefold():
                print(w)
                ex_check = True
    except sr.UnknownValueError:
        print("\nNON RIESCO A CAPIRE!\n")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))


if __name__ == '__main__':
    r = sr.Recognizer()
    m = sr.Microphone()
    with m as source:
        r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening

    # start listening in the background (note that we don't have to do this inside a `with` statement)
    stop_listening = r.listen_in_background(m, callback)
    # `stop_listening` is now a function that, when called, stops background listening

    try:
        ex_check = False
        run = True
        while run:
            if ex_check:
                print("\nAddio!\n")
                run = False
    except KeyboardInterrupt:
        print("\nVabbene, mi fermo!\n")

    # calling this function requests that the background listener stop listening
    stop_listening(wait_for_stop=False)
