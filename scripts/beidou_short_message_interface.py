#!/usr/bin/env python3
"""
BeiDou short message interface skeleton.

This is a placeholder for future implementation when the actual BeiDou short
message protocol specification becomes available.

NOT TO BE CONFUSED WITH:
- NTRIP (RTK correction data via internet)
- 4G/LTE (commercial cellular)
- LoRa (ISM band radio)

BeiDou short message is satellite-based text messaging, similar to Iridium SBD.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class BeiDouShortMessage:
    """BeiDou short message structure.

    Fields are placeholders and will be updated when actual protocol is available.
    """
    timestamp: float
    message_id: int
    direction: str  # "SEND" or "RECEIVE"
    payload: bytes
    status: str  # "PENDING", "SENT", "RECEIVED", "FAILED"


class BeiDouShortMessageInterface:
    """Interface placeholder for BeiDou short message hardware.

    Actual implementation requires:
    - BeiDou-compatible hardware module specification
    - Message protocol/frame format
    - Serial communication parameters
    - Satellite visibility and subscription
    """

    def __init__(self, port: str, baudrate: int = 9600):
        """Initialize interface.

        Args:
            port: Serial port (e.g., "/dev/ttyUSB0")
            baudrate: Baud rate (hardware-dependent)
        """
        self.port = port
        self.baudrate = baudrate
        self.connected = False

    def connect(self) -> bool:
        """Connect to BeiDou hardware module.

        Returns:
            True if connected successfully
        """
        # TODO: Implement when protocol available
        raise NotImplementedError("BeiDou protocol not available")

    def send_message(self, payload: bytes) -> Optional[BeiDouShortMessage]:
        """Send short message via BeiDou satellite.

        Args:
            payload: Message content (size limit hardware-dependent)

        Returns:
            Message object with status, or None if failed
        """
        # TODO: Implement when protocol available
        raise NotImplementedError("BeiDou protocol not available")

    def receive_message(self, timeout: float = 1.0) -> Optional[BeiDouShortMessage]:
        """Receive short message from BeiDou satellite.

        Args:
            timeout: Timeout in seconds

        Returns:
            Message object if received, None if timeout or error
        """
        # TODO: Implement when protocol available
        raise NotImplementedError("BeiDou protocol not available")

    def get_signal_quality(self) -> dict:
        """Get BeiDou signal quality metrics.

        Returns:
            Dictionary with signal strength, satellite count, etc.
        """
        # TODO: Implement when protocol available
        raise NotImplementedError("BeiDou protocol not available")


def main():
    """Placeholder main."""
    print("BeiDou Short Message Interface")
    print("===============================")
    print("Status: NO_PROTOCOL_AVAILABLE")
    print("")
    print("This is a placeholder interface.")
    print("Implementation requires:")
    print("  1. BeiDou hardware module specification")
    print("  2. Message protocol/frame format")
    print("  3. Serial communication parameters")
    print("")
    print("NOT the same as:")
    print("  - NTRIP (RTK correction data)")
    print("  - 4G/LTE (cellular)")
    print("  - LoRa (ISM band radio)")


if __name__ == "__main__":
    main()
