from setuptools import find_packages, setup

package_name = 'carbot_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henry',
    maintainer_email='2191976138@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pub = carbot_camera.image_pub:main',
            'scan_qrcode = carbot_camera.scan_qrcode:main',
            'camera_aim = carbot_camera.camera_aim:main',
        ],
    },
)
