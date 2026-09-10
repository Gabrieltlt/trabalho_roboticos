from setuptools import find_packages, setup

package_name = 'trabalho_roboticos'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Torres',
    maintainer_email='gabrieltlt721@gmail.com.com',
    description='Navegação autônoma do TurtleBot3 Burger (Trabalho Final - Sistemas Robóticos).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = trabalho_roboticos.navigator:main',
        ],
    },
)