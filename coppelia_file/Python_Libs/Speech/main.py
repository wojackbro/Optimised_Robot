import time
import speech_recognition as sr
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import Enums

break_words = [ 'break', 'basta' ]
stop_words = [ 'stop', 'fermati', "fermo" ]
resume_words = [ 'start', 'resume', 'ricomincia', 'inizia', 'vai', 'avanti', 'go' ]
lf_words = [ 'segui la linea', 'follow the line', 'follow', 'line', 'linea' ]
ms_words = [ 'fai missione', 'missione', 'oggetto', 'prendi oggetto', 'take the item', 'item', 'mission' ]

global break_check 
global stop_check  
global resume_check
global lf_check
global ms_check

# this is called from the background thread
def callback(recognizer, audio):
    # received audio data, now we'll recognize it using Google Speech Recognition
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        rec_text = recognizer.recognize_google(audio, language="it-IT")
        print(f'[ INFO ] You said: "{rec_text}"')
        for b in break_words:
            if rec_text.casefold() == b.casefold():
                global break_check
                break_check = True
        for s in stop_words:
            if rec_text.casefold() == s.casefold():
                global stop_check  
                stop_check = True
        for r in resume_words:
            if rec_text.casefold() == r.casefold():
                global resume_check
                resume_check = True
        for l in lf_words:
            if rec_text.casefold() == l.casefold():
                global lf_check
                lf_check = True
        for m in ms_words:
            if rec_text.casefold() == m.casefold():
                global ms_check
                ms_check = True
    except sr.UnknownValueError:
        print("[ INFO ] Waiting for a vocal response...")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def main():
    # RemoteCoppelia Setup
    client = RemoteAPIClient()
    sim = client.require('sim') 
    stop = {
        'id': 'stop_signal',
        'data': Enums.get(Enums.VocalCMD, "STOP")
    }
    resume = {
        'id': 'resume_signal',
        'data': Enums.get(Enums.VocalCMD, "START")
    }
    switch_lf = {
        'id': 'switch_lf_signal',
        'data': Enums.get(Enums.VocalCMD, "SWITCH_LF")
    }
    switch_ms = {
        'id': 'switch_ms_signal',
        'data': Enums.get(Enums.VocalCMD, "SWITCH_MS")
    }
    
    # VocalRecognition Setup
    r = sr.Recognizer()
    m = sr.Microphone()
    with m as source:
        r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening
    # start listening in the background
    stop_listening = r.listen_in_background(m, callback)
    # `stop_listening` is now a function that, when called, stops background listening

    print("[ START ] VocalRecognition")
    try:
        global break_check
        break_check = False
        global stop_check
        stop_check = False
        global resume_check
        resume_check = False
        global lf_check
        lf_check = False
        global ms_check
        ms_check = False
        run = True
        while run:
            if break_check:
                print(f'[ INFO ] Stop Recognition Loop!')
                run = False
            if stop_check:
                print(f'[ CMD ] Send STOP_SIGNAL to robot!')
                sim.broadcastMsg(stop)
                stop_check = False
            if resume_check:
                print(f'[ CMD ] Send RESUME_SIGNAL to robot!')
                sim.broadcastMsg(resume)
                resume_check = False
            if lf_check:
                print(f'[ CMD ] Send LINE-FOLLOW_SIGNAL to robot!')
                sim.broadcastMsg(switch_lf)
                lf_check = False
            if ms_check:
                print(f'[ CMD ] Send MISSION_SIGNAL to robot!')
                sim.broadcastMsg(switch_ms)
                ms_check = False
    except KeyboardInterrupt:
        print("[ END ] VocalRecognition")

    # calling this function requests that the background listener stop listening
    stop_listening(wait_for_stop=False)


if __name__ == "__main__":
    main()
