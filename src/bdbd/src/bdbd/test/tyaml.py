# demo of yaml usage
import yaml

yy = """
    /bdbd/pantilt_camera/image_raw/compressed:
        - /bdbd/pantilt_camera
        - /bdbd/other

    /bdbd/object_detect:
        - /bdbd_docker/object_detect
"""

y = yaml.load(yy)
print(y)
