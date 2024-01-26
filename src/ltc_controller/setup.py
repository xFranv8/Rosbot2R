from setuptools import find_packages, setup

package_name = 'ltc_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/ServerConnection.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xfranv8',
    maintainer_email='fv8vazquez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ltc_conotroller_main = ltc_controller.ltc_controller_main:main'
        ],
    },
)