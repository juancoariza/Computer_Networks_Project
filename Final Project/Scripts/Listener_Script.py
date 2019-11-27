# Created by Yamnel S.
# Edited and Updated by Randy Hattab

import pyrebase, os

# Library for keyboard commands
from pynput.keyboard import Key, Listener

# Firebase Authentication
# Should be filled with your own Firebase Authentication info
import firebase_admin
from firebase_admin import credentials
# Firebase initialization
cred = credentials.Certificate("path/to/5b6bfdbb849f19b23cfed8c456953b5f8086680e.json")
firebase_admin.initialize_app(cred)

class Listener_Script:
    def __init__(self):
        self.ready = True

"""
        # This information should be filled with your own Firebase authentication info
        config = {
          "apiKey": "apiKey",
          "authDomain": "projectId.firebaseapp.com",
          "databaseURL": "https://databaseName.firebaseio.com",
          "storageBucket": "projectId.appspot.com",
          "serviceAccount": "path/to/serviceAccountCredentials.json"
        }pushToDB
"""

        ###### Firebase Connection #######
        firebase = pyrebase.initialize_app(config) # initiation Firebase link through Pyrebase
        self.db = firebase.database()  # calling the Database object

        # listen to the real-time stream while the ready flag is active
        while self.ready:
            my_stream = self.db.child("pilot").stream(self.stream_handler)
            my_stream.close()

    # Keyboard commands
    # w = Forward
    def on_press(key):
        if key == 'w' {
            print('w pressed'.format(
        key))
        }
        print('{w} pressed'.format(
            key))
    def on_release('w'):
        print('{w} release'.format(
            key))
        if key == Key.esc:
            # Stop listener
            return False
"""
    # a = Left
    def on_release('a'):
        print('{a} release'.format(
            key))
    def on_release('a'):
        print('{a} release'.format(
            key))
        if key == Key.esc:
            # Stop listener
            return False

    # s = Reverse
    def on_press('s'):
        print('{s} pressed'.format(
            key))
    def on_release('s'):
        print('{s} release'.format(
            key))
        if key == Key.esc:
            # Stop listener
            return False

    # d = Right
    def on_press('d'):
        print('{d} pressed'.format(
            key))
    def on_release('d'):
        print('{d} release'.format(
            key))
        if key == Key.esc:
            # Stop listener
            return False

    # i = Increase altitude
    def on_press('i'):
        print('{i} pressed'.format(
            key))

    # k = decrease altitude
    def on_press('k'):
        print('{k} pressed'.format(
            key))

    # k = decrease altitude
    def on_press('k'):
        print('{k} pressed'.format(
            key))

    # Pobably don't need. This looks like it just collects inputs
    # Collect events until released
    with Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
"""

    def stream_handler(self, message):
            self.ready = False
            coords = str(message["data"]).split(" ")
            try:
                done = os.system("python Pilot_Script.py {0} {1}".format(coords[0], coords[1]))

                if done == 0:
                    self.ready = True
                    self.db.child("ready").set(1)
                    self.db.child("pilot").set("")


            except Exception:
                print("Trouble with the coords!")

# run the listener script
listen = Listener_Script()
