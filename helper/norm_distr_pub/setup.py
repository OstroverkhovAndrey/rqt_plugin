from setuptools import find_packages, setup

package_name = 'norm_distr_pub'

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
    maintainer='Andrey Ostroverkhov',
    maintainer_email='andrey26052001@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'norm_distr_pub = norm_distr_pub.norm_distr_pub:main',
        ],
    },
)
