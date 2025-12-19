from setuptools import setup

package_name = 'my_bot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Ball perception nodes for my bot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_ball_detector = my_bot_perception.circle_ball_detector:main',
            'circle_ball_visualizer = my_bot_perception.circle_ball_visualizer:main',
            'circle_ball_node = my_bot_perception.circle_ball_node:main',
        ],
    },
)
