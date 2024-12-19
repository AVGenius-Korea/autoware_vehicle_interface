from setuptools import find_packages, setup
import os
from glob import glob

# Other imports ...


package_name = 'avg_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('kap_vehicle_interface/launch', '*launch.[py]*'))),
        ('lib/' + package_name + '/can_utils', ['kap_vehicle_interface/can_utils/can_sender.py']),
        ('lib/' + package_name + '/kap_dataclass', ['avg_vehicle_interface/vehicle_dataclass/BrakeStaFb.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/data_utils.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/DriveCtrl.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/DriveStaFb.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/PowerStaFb.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/SteerCtrl.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/SteerStaFb.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/VehicleCtrl.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/VehicleFltSta.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/VehicleStaFb.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/VehicleWorkStaFb.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/BrakeCtrl.py',
                                                    'avg_vehicle_interface/vehicle_dataclass/WheelRpmFb.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JEON YANG HO',
    maintainer_email='yhjeon@avgenius.kr',
    description='kap vehicle interface',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface_report = avg_vehicle_interface.command_node:main',
            'interface_command = avg_vehicle_interface.report_node:main',
            'interface_test = avg_vehicle_interface.test.interface_test:main',
        ],
    },
)
