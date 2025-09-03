from setuptools import find_packages, setup

package_name = 'mouth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', "sounddevice"],
    zip_safe=True,
    maintainer='yashraj-kumawat-14',
    maintainer_email='yashrajkumawat7357@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["speak=mouth.speak_with_piperTTS:main",
        ],
    },
)
