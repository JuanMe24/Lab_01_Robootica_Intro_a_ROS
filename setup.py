from setuptools import find_packages, setup

package_name = 'my_turtle_controller'

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
    maintainer='robotica-movil',
    maintainer_email='robotica-movil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
   entry_points={                       #creacion de nodos
    'console_scripts': [
        'move_turtle = my_turtle_controller.move_turtle:main',
        'move_turtlelab = my_turtle_controller.move_turtlelab:main',
    ],
},
)
