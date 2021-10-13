import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from dronekit_sitl import SITL
import constants
from geopy.distance import geodesic


class FlightController():

    def __init__(self):
        self.state = 'init'
        self.sitl = SITL()
        self.sitl.download('copter', '3.3', verbose=False)
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
        self.sio_emit("status", {"status": "stop"})
        self.change_state("idle")

    def arm(self):
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        print("Waiting for arming...")
        while not self.vehicle.armed:
            time.sleep(1)
        print("Vehicle Armed.")

    def deliver(self, payload):
        self.go_to_helper(payload, "transit-to")

    def change_state(self, state, emit=True):
        self.state = state
        print(f"[[STATE CHANGED]]-->{state}")
        if emit:
            self.sio_emit("state", {"state": self.state})

    def go_to_helper(self, payload, state):
        self.destination = (payload['lat'], payload['lon'])
        self.change_state(state)
        self.arm()
        print("going to: ")
        print(payload)
        self.sio_emit("status", {"status": "take-off"})
        self.vehicle.simple_takeoff(constants.MAX_ALTITUDE)
        print("Taking off!")
        print(self.vehicle.location.global_relative_frame.alt)
        while self.vehicle.location.global_relative_frame.alt < constants.MAX_ALTITUDE * .95:
            time.sleep(1)

        print("MAX_ALTITUDE reached, flying to coordinates...")
        self.sio_emit("status", {"status": "flying"})
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
                'gps', {"lat": self.current_location.lat, "lng": self.current_location.lon})

            if distance < 2:
                vehicle.mode = VehicleMode("LAND")
                print('landing at user-destination...')
                self.sio_emit("status", {"status": "landing"})
                time.sleep(25)
                self.sio_emit("status", {"status": "stop"})
                print('Landing sucessfull, disarming vehicle..')
                self.vehicle.armed = False
                self.change_state('transit-await')

                time.sleep(5)
                print("Waiting for pickup...")
                self.change_state('transit-delivered')
                print('Delivered go back home..')
                self.go_to_helper(
                    {'lat': constants.HOME_LAT, 'lon': constants.HOME_LNG}, 'transit-back')
        elif self.state == 'transit-back':
            if distance < 2:
                vehicle.mode = VehicleMode("LAND")
                print('Landing at home...')
                time.sleep(20)
                print('Landing sucessfull, disarming vehicle..')
                self.change_state('idle')

    def shutdown(self):
        self.vehicle.close()
        self.sitl.stop()


if __name__ == '__main__':
    print("flight_controller class")
