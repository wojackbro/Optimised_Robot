import time
import speech_recognition as sr
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import Enums

# Stop the program
break_words = [ 'break', 'basta' ]

# Stop the robot
stop_words = [ 'stop', 'fermati', 'fermo', 'halt', 'wait', 'aspetta', 'pausa',
    'pause', 'resta fermo', 'non muoverti', 'freeze', 'stai fermo' ]

# Resume the robot movements
resume_words = [ 'start', 'resume', 'ricomincia', 'inizia', 'vai', 'avanti', 'go',
    'continua', 'continue', 'riprendi', 'muoviti', 'move', 'parti',
    'prosegui', 'proceed', 'forward', 'restart', 'riavvia' ]

# Enable the LineFollowing mode
lf_words = [ 'segui la linea', 'follow the line', 'follow', 'line', 'linea',
    'segui linea', 'line follow', 'modalità linea', 'line mode',
    'traccia', 'track', 'segui traccia', 'follow track', 'percorso',
    'path', 'segui il percorso', 'follow the path' ]

# Enable the Mission mode
ms_words = [ 'fai missione', 'missione', 'oggetto', 'prendi oggetto', 'take the item',
    'item', 'mission', 'vai all\'oggetto', 'go to item', 'reach item',
    'raggiungi oggetto', 'prendi', 'take', 'grab', 'afferra',
    'raccogli', 'collect', 'pick up', 'modalità missione', 'mission mode' ]

# Make the robot avoid an obstacle ( clockwise )
cw_words = [
    'orario', 'senso orario', 'in senso orario', 'clockwise', 'cw',
    'gira orario', 'verso orario', 'destra oraria', 'orologio',
    'come le lancette', 'senso delle lancette'
]

# Make the robot avoid an obstacle ( counterwise )
ccw_words = [
    'antiorario', 'senso antiorario', 'in senso antiorario',
    'counterclockwise', 'counter clockwise', 'anticlockwise', 'ccw',
    'gira antiorario', 'verso antiorario', 'contro orario',
    'contro le lancette', 'senso contrario alle lancette'
]

# Make the robot avoid an obstacle in an automatic way
auto_words = [
    'automatico', 'auto', 'automatic', 'modalità automatica',
    'automatic mode', 'da solo', 'decidi tu', 'normale', 'default'
]

# Turning 180
turn_words = [
    'ruota', 'rotate', 'dietro', 'girati', 'back', 'comeback'
]

global break_check 
global stop_check  
global resume_check
global lf_check
global ms_check
global cw_check
global ccw_check
global auto_check
global turn_check

# this is called from the background thread
def callback(recognizer, audio):
    # received audio data, now we'll recognize it using Google Speech Recognition
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        ''' rec_text = recognizer.recognize_google(audio, language="it-IT") '''
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
        for cw in cw_words:
            if rec_text.casefold() == cw.casefold():
                global cw_check
                cw_check = True
        for ccw in ccw_words:
            if rec_text.casefold() == ccw.casefold():
                global ccw_check
                ccw_check = True
        for at in auto_words:
            if rec_text.casefold() == at.casefold():
                global auto_check
                auto_check = True
        for t in turn_words:
            if rec_text.casefold() == t.casefold():
                global turn_check
                turn_check = True
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
    avoid_cw = {
        'id': 'avoid_cw',
        'data': Enums.get(Enums.VocalCMD, "AVOID_CW")
    }
    avoid_ccw = {
        'id': 'avoid_ccw',
        'data': Enums.get(Enums.VocalCMD, "AVOID_CCW")
    }
    avoid_auto = {
        'id': 'avoid_auto',
        'data': Enums.get(Enums.VocalCMD, "AVOID_AUTO")
    }
    turn = {
        'id': 'turn',
        'data': Enums.get(Enums.VocalCMD, "TURN_180")
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
        global cw_check
        cw_check = False
        global ccw_check
        ccw_check = False
        global auto_check
        auto_check = False
        global turn_check
        turn_check = False
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
            if cw_check:
                print(f'[ CMD ] Send AVOID_CW to robot!')
                sim.broadcastMsg(avoid_cw)
                cw_check = False
            if ccw_check:
                print(f'[ CMD ] Send AVOID_CCW to robot!')
                sim.broadcastMsg(avoid_ccw)
                ccw_check = False
            if auto_check:
                print(f'[ CMD ] Send AVOID_AUTO to robot!')
                sim.broadcastMsg(avoid_auto)
                auto_check = False
            if turn_check:
                print(f'[ CMD ] Send TURN_180 to robot!')
                sim.broadcastMsg(turn)
                turn_check = False
    except KeyboardInterrupt:
        print("[ END ] VocalRecognition")

    # calling this function requests that the background listener stop listening
    stop_listening(wait_for_stop=False)


if __name__ == "__main__":
    main()
