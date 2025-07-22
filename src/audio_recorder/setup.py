from setuptools import setup

package_name = 'audio_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nakamalab',
    description='ROS 2 node for recording and playing audio',
    license='Apache-2.0',
    entry_points={
    'console_scripts': [
        'recorder = audio_recorder.recorder:main',
        'player = audio_recorder.player_node:main',
        'play2 = audio_recorder.play2:main',
        'record2 = audio_recorder.record2:main',
    ],
    },
)
