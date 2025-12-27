from setuptools import setup

package_name = 'ur3_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'gym', 'numpy', 'torch', 'stable-baselines3'],
    zip_safe=True,
    maintainer='vinayak',
    maintainer_email='vinayakvsamant@gmail.com',
    description='Reinforcement Learning for UR3e Cartesian Goal Reaching',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train = ur3_rl.train:main',
            'evaluate = ur3_rl.evaluate:main',
        ],
    },
)
