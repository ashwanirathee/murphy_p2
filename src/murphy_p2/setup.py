from setuptools import find_packages, setup

package_name = 'murphy_p2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "cameras_node = murphy_p2.cameras_node:main",
            "visual_processor_node = murphy_p2.visual_processor_node:main",
            "brain_node = murphy_p2.brain_node:main",
            "audio_node = murphy_p2.audio_node:main",
            "ear_node = murphy_p2.ear_node:main",
            "action_node = murphy_p2.action_node:main",
            "vlm_node = murphy_p2.vlm_node:main",
        ],
    },
)