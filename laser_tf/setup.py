from setuptools import setup

package_name = 'laser_tf'

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
    maintainer='clobot',
    maintainer_email='qkrwk0921@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_tf = laser_tf.laser_tf:main',
            'laser_tms = laser_tf.laser_tms:main'
        ],
    },
)
