 this is in setup.cfg which solves the virtual env , module not found error "[develop]
script_dir=$base/lib/mouth
[install]
install_scripts=$base/lib/mouth
[build_scripts]
executable = /usr/bin/env python3
"

and use requirements.txt for installing modules

and use export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
orinstall this : conda install -c conda-forge libstdcxx-ng=13


pip install typing_extensions pyyaml numpy packaging protobuf sympy empy lark-parser setuptools colcon-common-extensions sounddevice

also make sure to have correct python env like 3.12.3

export OPENAI_API_KEY="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"


for using openai 


from openai import OpenAI
client = OpenAI()

response = client.responses.create(
    model="gpt-4o-mini",
    input="Write a one-sentence bedtime story about a unicorn."
)

print(response.output_text)

