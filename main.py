from socket_client import Client
from flight_controller import FlightController

flight_controller = FlightController()
client = Client(flight_controller)
try:
    flight_controller.setup(client.emit)
    client.run()
except KeyboardInterrupt:
    pass
finally:
    flight_controller.shutdown()
