from setuptools import find_packages, setup

package_name = 'wall_following_alpha'

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
    maintainer='vekkaz',
    maintainer_email='santiagov3lasquez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'dist_finder_alpha = wall_following_alpha.dist_finder_alpha:main',
             'control_alpha = wall_following_alpha.control_alpha:main',
        ],
    },
)
