from setuptools import find_packages, setup

package_name = 'ur3e_rl'

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
    maintainer='vinayak',
    maintainer_email='vinayakvsamant@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur3e_motion = ur3e_rl.ur3e_motion:main',
            'train_rl = ur3e_rl.train:main',
            'test_rl = ur3e_rl.test:main',
        ],
    },
)
