from setuptools import setup

package_name = 'pipebot_genetic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pipebot_GA.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas Cambier',
    maintainer_email='n.p.cambier@leeds.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'GA_client = pipebot_genetic.GA_client:main',
        ],
    },
)
