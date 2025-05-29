from controller.state import State
import lib.logging as logging
import threading
import socket
import time


class ControllerClient(threading.Thread):
    SEND_INTERVAL = 0.016  # Increased send rate (approx 62.5 Hz)

    def __init__(self, server_address: tuple[str, int], state: State):
        super().__init__()
        self.server_address = server_address
        self.log = logging.getLogger(self.__class__.__name__)
        self.command_state = state
        state.reset()

    def send_state_udp(self, formatted_message):
        """Sends the state message to the server using UDP."""
        try:
            # self.log.debug(f"Sending UDP State: {formatted_message}") # Uncomment for debugging
            self.client_socket.sendto(
                formatted_message.encode(), self.server_address)
        except Exception as e:
            self.log.warning(
                f"Error sending UDP state '{formatted_message}': {e}")
            return False
        return True

    def run(self):
        try:
            self.log.info(
                f"Starting UDP connection to {self.server_address[0]}:{self.server_address[1]}...")
            self.client_socket = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM)
            self.running = True

            while self.running:
                self.command_state.update()

                # Format and send the current state
                if not self.send_state_udp(self.command_state.serialize()):
                    self.log.warning(
                        "Failed to send state, attempting to continue...")

                self.command_state.after_update()

                time.sleep(self.SEND_INTERVAL)
        except Exception as e:
            self.log.error(
                f"An error occurred in the UDP stateful controller: {e}")
        finally:
            self.command_state.reset()
            self.send_state_udp(self.command_state.serialize())
            self.client_socket.close()

    def stop(self):
        self.log.info("Attempting to stop...")
        self.running = False
