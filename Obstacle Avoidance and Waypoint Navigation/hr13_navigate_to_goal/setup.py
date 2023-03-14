from setuptools import setup

package_name = 'hr13_navigate_to_goal'

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
    maintainer='himanshu',
    maintainer_email='hvairagade3@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "get_object_range = " + package_name + ".get_object_range:main",
            "bot_behaviour = " + package_name + ".bot_behaviour:main",
            "controller = " + package_name + ".controller_node:main",
            "debug = " + package_name + ".debug_node:main"
        ],
    },
)
