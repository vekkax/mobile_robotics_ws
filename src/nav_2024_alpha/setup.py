from setuptools import find_packages, setup

package_name = 'nav_2024_alpha'

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
            'nav_2024_alpha_node = nav_2024_alpha.nav_2024_alpha_node:main',
            'nav_control_alpha = nav_2024_alpha.nav_control_alpha:main',
            'nav_rozo = nav_2024_alpha.nav_rozo:main',
        ],
    },
)
