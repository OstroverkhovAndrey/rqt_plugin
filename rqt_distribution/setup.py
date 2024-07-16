from setuptools import setup

package_name = 'rqt_distribution'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #package_dir={"", "src"},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource', ['resource/distribution.ui']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrey Ostroverkhov',
    maintainer_email='andrey26052001@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
