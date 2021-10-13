import socketio
import json
import constants


class Client():
    sio = socketio.Client()

    def __init__(self, flight_controller):
        self.flight_controller = flight_controller

    def setup(self):
        self.call_backs()
        self.sio.connect('http://192.168.132.1:8080')

    def loop(self):
        self.sio.wait()

    def emit(self, event, message):
        if type(message) is dict:
            message["id"] = constants.ID
        self.sio.emit(event, message)

    def call_backs(self):
        @self.sio.event
        def connect():
            self.sio.emit(
                'subscribe', {'type': 'drone', 'id': constants.ID})
            print('connection established')

        @self.sio.event
        def deliver(data):
            self.flight_controller.deliver(data)

        @self.sio.event
        def disconnect():
            print('disconnected from server')


if __name__ == '__main__':
    print("socket.io client class")
