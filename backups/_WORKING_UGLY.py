# WORKING

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from dronekit_sitl import SITL
import constants
from geopy.distance import geodesic


class FlightController():

    def __init__(self):
        self.state = 'idle'
        self.sitl = SITL()
        self.sitl.download('copter', '3.3', verbose=True)
        sitl_args = ['-I0', '--model', 'quad',
                     f'--home={constants.HOME_LAT},{constants.HOME_LNG},0,180']
        self.sitl.launch(sitl_args, await_ready=True, restart=True)
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)
        self.altitude = 0
        self.vehicle.add_attribute_listener('location', self.location_callback)
        print("Connected to vehicle in SITL")

    def setup(self, sio_emit):
        self.sio_emit = sio_emit
        print("Waiting for location...")
        while self.vehicle.location.global_frame.lat == 0:
            time.sleep(0.1)

        self.home_coords = [self.vehicle.location.global_frame.lat,
                            self.vehicle.location.global_frame.lon]

        print("Setting up vehicle...")
        while not self.vehicle.is_armable:
            time.sleep(1)
        print("Vehicle initialized.")
        self.sio_emit("status", {"status": "stop", "id": constants.ID})

    def arm(self):
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        print("Waiting for arming...")
        while not self.vehicle.armed:
            time.sleep(1)
        print("Vehicle Armed.")

    def deliver(self, payload):
        self.go_to_helper(payload, 'transit-to')

    def go_to_helper(self, payload, state):
        self.destination = (payload['lat'], payload['lon'])
        self.state = state
        self.sio_emit(
            'state', {"state": self.state, "id": constants.ID})
        self.arm()
        print("going to: ")
        print(payload)
        print(f"state is: {state}")
        self.sio_emit("status", {"status": "take-off", "id": constants.ID})
        self.vehicle.simple_takeoff(constants.MAX_ALTITUDE)
        print("Taking off!")
        print(self.vehicle.location.global_relative_frame.alt)
        while self.vehicle.location.global_relative_frame.alt < constants.MAX_ALTITUDE * .95:
            time.sleep(1)

        print("MAX_ALTITUDE reached, flying to coordinates...")
        self.sio_emit("status", {"status": "flying", "id": constants.ID})
        self.vehicle.airspeed = constants.AIR_SPEED
        go_to = LocationGlobalRelative(
            payload['lat'], payload['lon'], constants.MAX_ALTITUDE)

        self.vehicle.simple_goto(go_to)

    def location_callback(self, vehicle, name, location):

        if location.global_relative_frame.alt is not None:
            self.altitude = location.global_relative_frame.alt
        self.current_location = location.global_relative_frame
        distance = 0
        if hasattr(self, 'destination'):
            distance = geodesic((self.current_location.lat, self.current_location.lon),
                                self.destination).m

        if self.state == 'transit-to':
            self.sio_emit(
                'gps', {"lat": self.current_location.lat, "lng": self.current_location.lon, "id": constants.ID})

            if distance < 2:
                vehicle.mode = VehicleMode("LAND")
                print('landing...')
                self.sio_emit(
                    "status", {"status": "landing", "id": constants.ID})
                time.sleep(20)
                print('Landing sucessfull, disarming vehicle..')
                # self.sio_emit('state', 'transit-deliverd')
                self.vehicle.armed = False
                self.sio_emit("status", {"status": "stop", "id": constants.ID})
                self.state = 'transit-await'
                self.sio_emit(
                    'state', {"state": self.state, "id": constants.ID})

                time.sleep(5)

                self.state = 'transit-delivered'
                self.sio_emit(
                    'state', {"state": self.state, "id": constants.ID})

                print('Delivered go back home..')
                self.go_to_helper(
                    {'lat': constants.HOME_LAT, 'lon': constants.HOME_LNG}, 'transit-back')
        elif self.state == 'transit-back':
            if distance < 2:
                vehicle.mode = VehicleMode("LAND")
                print('landing...')
                time.sleep(10)
                print('Landing sucessfull, disarming vehicle..')
                self.state = 'idle'
                self.sio_emit(
                    'state', {"state": self.state, "id": constants.ID})

    def shutdown(self):
        self.vehicle.close()
        self.sitl.stop()


if __name__ == '__main__':
    print("flight_controller class")
