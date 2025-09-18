import sys
if sys.prefix == '/home/yantrigo/miniconda3/envs/humanoid':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yantrigo/Dropbox/humanoid_site_1.0.0/software/firmware/rpi5/humanoid/install/hands'
