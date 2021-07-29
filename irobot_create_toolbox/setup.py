from setuptools import setup

package_name = 'irobot_create_toolbox'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Rodrigo Causarano',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Helper tools for the create 3 robot.',
    maintainer='Ekumen',
    maintainer_email='ekumen@irbt.onmicrosoft.com',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_republisher ='
            ' irobot_create_toolbox.topic_republisher:main',
        ],
    },
)