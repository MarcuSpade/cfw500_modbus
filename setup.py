from setuptools import setup

package_name = 'cfw500_modbus'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seuemail@example.com',
    description='Pacote ROS 2 para comunicação Modbus com inversor WEG CFW500',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cfw500_modbus_node = cfw500_modbus.cfw500_modbus_node:main'
        ],
    },
)

