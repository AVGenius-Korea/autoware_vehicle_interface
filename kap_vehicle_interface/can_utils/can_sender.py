import can


class CANSender:
    def __init__(self, channel='can0'):
        self.channel = channel
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')

    def send(self, can_id, data, extended=False):
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=extended,
        )
        try:
            self.bus.send(message)
            return True
        except can.CanError:
            return False

    def close(self):
        self.bus.shutdown()