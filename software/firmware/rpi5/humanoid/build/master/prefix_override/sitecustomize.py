import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yantrigo/Dropbox/humanoid_site_1.0.0/software/firmware/rpi5_brain_firmware/brain/install/master'
