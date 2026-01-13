# ROS exception classes for compatibility with rospy.


class ROSException(Exception):
    pass


class ROSInterruptException(ROSException, KeyboardInterrupt):
    pass


class ROSInitException(ROSException):
    pass


class ROSSerializationException(ROSException):
    pass


class ROSTimeMovedBackwardsException(ROSException):
    def __init__(self, time):
        self.time = time
        super().__init__(f'Time moved backwards by {time}s')


class ServiceException(Exception):
    pass
