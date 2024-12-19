from setuptools import find_packages, setup
import os
from glob import glob

# Other imports ...


package_name = 'kap_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('kap_vehicle_interface/launch', '*launch.[py]*'))),
        ('lib/' + package_name + '/can_utils', ['kap_vehicle_interface/can_utils/can_sender.py']),
        ('lib/' + package_name + '/kap_dataclass', ['kap_vehicle_interface/kap_dataclass/BrakeStaFb.py',
                                                    'kap_vehicle_interface/kap_dataclass/data_utils.py',
                                                    'kap_vehicle_interface/kap_dataclass/DriveCtrl.py',
                                                    'kap_vehicle_interface/kap_dataclass/DriveStaFb.py',
                                                    'kap_vehicle_interface/kap_dataclass/PowerStaFb.py',
                                                    'kap_vehicle_interface/kap_dataclass/SteerCtrl.py',
                                                    'kap_vehicle_interface/kap_dataclass/SteerStaFb.py',
                                                    'kap_vehicle_interface/kap_dataclass/VehicleCtrl.py',
                                                    'kap_vehicle_interface/kap_dataclass/VehicleFltSta.py',
                                                    'kap_vehicle_interface/kap_dataclass/VehicleStaFb.py',
                                                    'kap_vehicle_interface/kap_dataclass/VehicleWorkStaFb.py',
                                                    'kap_vehicle_interface/kap_dataclass/BrakeCtrl.py',
                                                    'kap_vehicle_interface/kap_dataclass/WheelRpmFb.py']),

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
            'kap_interface_rpt = kap_vehicle_interface.kap_interface_rpt_node:main',
            'kap_interface_cmd = kap_vehicle_interface.kap_interface_cmd_node:main',
            'kap_interface_test = kap_vehicle_interface.test_cmd:main',
        ],
    },
)
