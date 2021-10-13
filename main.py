from socket_client import Client
from flight_controller import FlightController

flight_controller = FlightController()
client = Client(flight_controller)
client.setup()
flight_controller.setup(client.emit)
client.loop()
