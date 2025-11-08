from setuptools import find_packages, setup

package_name = 'alert_utils'

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
    maintainer='max1',
    maintainer_email='alert@fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stamped_twist_converter = alert_utils.stamped_twist_converter:main',
            'pcl_mapfilter = alert_utils.pcl_mapfilter:main',
        ],
    },
)
