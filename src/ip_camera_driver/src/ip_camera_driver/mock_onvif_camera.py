import rospy

class MockONVIFCamera:

    def __init__(self):
        self._token = None
        self.ProfileToken = None
        self.Position = {'PanTilt': {'_x':  0.0, '_y': 0.0}, 'Zoom': {'_x': 0.0}}
        self.Speed = {'PanTilt': {'_x': 0.0, '_y': 0.0}, 'Zoom': {'_x': 0.0}}

    def create_media_service(self):
        return self

    def GetProfiles(self):
        return [self]

    def create_ptz_service(self):
        return self

    def GotoHomePosition(self, token):
        return

    def Stop(self, token):
        return

    def AbsoluteMove(self, cmd):
        self.ProfileToken = cmd.ProfileToken
        self.Position = cmd.Position
        self.Speed = cmd.Speed

    def create_type(self, move_type):
        return self

    def GetStatus(self, token):
        rospy.sleep(rospy.Duration(1.0))
        status = {'Position': self.Position, 'Speed': self.Speed}
        return status
