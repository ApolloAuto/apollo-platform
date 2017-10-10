from camera_calibration_parsers.camera_calibration_parsers_wrapper import __readCalibrationWrapper
from sensor_msgs.msg import CameraInfo

def readCalibration(file_name):
    """Read .ini or .yaml calibration file and return (camera name and cameraInfo message).
    
    @param file_name: camera calibration file name
    @type file_name: str
    @return: (camera name, cameraInfo message) or None on Error
    @rtype: tuple(str, sensor_msgs.msg.CameraInfo | None
    """
    ret, cn, ci = __readCalibrationWrapper(file_name)
    if not ret:
        return None
    c = CameraInfo()
    c.deserialize(ci)
    return cn, c
