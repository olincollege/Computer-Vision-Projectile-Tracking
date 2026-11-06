from setuptools import find_packages, setup

package_name = 'projectile_tracking'

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
    maintainer='austin',
    maintainer_email='awoolf07@gmail.com',
    description='Tracks the 3D position of a ball using images',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ball_detector = projectile_tracking.ball_detector:main',
            'position_calculator = projectile_tracking.position_calculator:main',
            'image_publisher = projectile_tracking.image_publisher:main'
        ],
    },
)
