from pprint import pprint

import roslibpy


class Platform:
    seq = 0

    def __init__(self, ip, on_ready=None):
        self.ros = roslibpy.Ros(ip, port=9090)
        self.ros.run()

        if on_ready is not None:
            self.ros.on_ready(lambda: on_ready(self.ros.is_connected))

        self.vel_topic = roslibpy.Topic(self.ros, '/cmd_vel', 'geometry_msgs/TwistStamped')

        self.scan_topic = roslibpy.Topic(self.ros, '/scan', 'sensor_msgs/LaserScan')
        self.scan_topic.subscribe(self.on_scan)

    def on_scan(self, data):
        pprint(len(data['ranges']))

    def drive(self, throttle, turn):
        self.vel_topic.publish(
            roslibpy.Message(
                {
                    'twist': {
                        'linear': {'x': throttle, 'y': turn, 'z': 0.0},
                        'angular': {'x': 0.0, 'y': 0.0, 'z': turn},
                    },
                    'header': {
                        'frame_id': 'base_link',
                        'seq': self.seq,
                        'stamp': roslibpy.Time.now()
                    }
                }
            )
        )
        self.seq += 1


if __name__ == '__main__':
    p = Platform('192.168.12.20')
    while True:
        pass
    # scan = roslibpy.Topic(p.ros, "/scan", )
