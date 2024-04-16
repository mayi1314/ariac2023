from rclpy import Node


class Sensor(Node):
    def __init__(self, node_name: str, sensor_id):
        super().__init__(node_name)
        self.id = sensor_id
