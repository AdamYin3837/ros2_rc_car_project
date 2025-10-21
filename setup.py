from setuptools import find_packages, setup

package_name = 'my_onboarding_project2'

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
    maintainer='adamyin',
    maintainer_email='adamyin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "joystick_interpreter = my_onboarding_project2.joystick_interpreter:main",
            "canbus_node = my_onboarding_project2.canbus_node:main"

        ],
    },
)
