#!/usr/bin/env python3
import os
import sys


def test_scoped_startup():
    # The launcher's namespace exists before the script imports rospy.
    os.environ['ROS_NAMESPACE'] = '/test_scope'
    if os.environ['ROS_VERSION'] == '2':
        sys.argv.extend(['--ros-args', '-r', '__ns:=/test_scope'])

    import rospy
    from std_srvs.srv import SetBool, SetBoolResponse

    echo = rospy.ServiceProxy('echo', SetBool)
    rospy.init_node('scoped_startup')
    assert rospy.get_namespace() == '/test_scope/'
    assert rospy.rostime.is_wallclock()
    assert rospy.rostime.get_rostime_cond() is rospy.rostime.get_rostime_cond()

    rospy.set_param('/test_scope/settings/gain', 12)
    assert rospy.get_param('settings/gain') == 12
    rospy.set_param('~scale', 4)
    assert rospy.get_param('/test_scope/scoped_startup/scale') == 4

    rospy.Service('echo', SetBool, lambda r: SetBoolResponse(r.data, 'echo'))
    rospy.Service('~relay', SetBool, lambda r: echo(r))
    relay = rospy.ServiceProxy('~relay', SetBool)
    echo.wait_for_service(2)
    relay.wait_for_service(2)
    assert relay(True).message == 'echo'

    states = []

    def record(label):
        states.append((
            label, rospy.core.is_shutdown_requested(), rospy.is_shutdown()
        ))

    rospy.on_shutdown(lambda: record('client'))
    rospy.core.add_preshutdown_hook(lambda reason: record('pre'))
    rospy.core.add_shutdown_hook(lambda reason: record('shutdown'))
    assert not rospy.core.is_shutdown_requested()
    rospy.signal_shutdown('done')
    assert states == [
        ('client', True, False),
        ('pre', True, False),
        ('shutdown', True, True),
    ]


def main():
    test_scoped_startup()


if __name__ == '__main__':
    main()
