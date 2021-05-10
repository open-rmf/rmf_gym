from setuptools import setup

package_name = 'rmf_gym_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    author='cnboonhan',
    author_email='cnboonhan@openrobotics.org',
    zip_safe=True,
    maintainer='cnboonhan',
    maintainer_email='cnboonhan@openrobotics.org',
    description='A package containing scripts fort tests',
    license='Apache License Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'check_tasks_complete = rmf_gym_tools.check_tasks_complete:main',
          'extract_waypoints = rmf_gym_tools.extract_waypoints:main',
          'random_tasks = rmf_gym_tools.random_tasks:main',
          'check_environment = rmf_gym_tools.check_environment:main',
        ],
    },
)
