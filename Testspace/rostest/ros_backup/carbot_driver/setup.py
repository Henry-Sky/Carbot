from setuptools import find_packages, setup

package_name = 'carbot_driver'

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
    maintainer='henry',
    maintainer_email='henry@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'car_driver = carbot_driver.car_driver:main',
            'key_ctrl = carbot_driver.key_ctrl:main',
        ],
    },
)
