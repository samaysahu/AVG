from setuptools import setup
import os
from glob import glob

package_name = 'car_rqt_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.xml')), # Modified line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samay', # Replace with your name
    maintainer_email='samaysahu700@gmail.com', # Replace with your email
    description='Custom RQT GUI for car control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
        'rqt_gui_py_plugins': [ # Add this entry point
            'car_control = car_rqt_gui.car_rqt_gui_plugin:CarRqtGuiPlugin',
        ],
    },
)