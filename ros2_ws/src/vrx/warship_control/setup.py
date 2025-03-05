from setuptools import find_packages, setup

package_name = 'warship_control'

setup(
    name='warship_control',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duy12i1i7',
    maintainer_email='udy.duyy@gmail.com',
    description='control canon',
    license='duy',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_listener = warship_control.fire_listener:main',
        ],
    },
)
