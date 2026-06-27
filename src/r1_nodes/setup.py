from setuptools import find_packages, setup

package_name = 'r1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "cameras_node = r1.cameras_node:main",
            "visual_processor_node = r1.visual_processor_node:main",
            "brain_node = r1.brain_node:main",
            "audio_node = r1.audio_node:main",
            "ear_node = r1.ear_node:main",
            "action_node = r1.action_node:main",
            "vlm_node = r1.vlm_node:main",
        ],
    },
)
