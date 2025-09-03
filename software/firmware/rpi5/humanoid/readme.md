 this is in setup.cfg which solves the virtual env , module not found error "[develop]
script_dir=$base/lib/mouth
[install]
install_scripts=$base/lib/mouth
[build_scripts]
executable = /usr/bin/env python3
"

and use requirements.txt for installing modules

and use export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

