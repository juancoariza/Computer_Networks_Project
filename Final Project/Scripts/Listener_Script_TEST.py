import pyrebase, os

# Created by Yamnel S.

class Listener_Script:
    def __init__(self):
        self.ready = True

        # This information should be filled with your own Firebase authentication info
        config = {
          "apiKey": "apiKey",
          "authDomain": "projectId.firebaseapp.com",
          "databaseURL": "https://databaseName.firebaseio.com",
          "storageBucket": "projectId.appspot.com",
          "serviceAccount": "path/to/serviceAccountCredentials.json"
        }

        ###### Firebase Connection #######
        firebase = pyrebase.initialize_app(config) # initiation Firebase link through Pyrebase
        self.db = firebase.database()  # calling the Database object

        # listen to the real-time stream while the ready flag is active
        while self.ready:
            my_stream = self.db.child("pilot").stream(self.stream_handler)
            my_stream.close()


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
