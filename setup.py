from glob import glob

from setuptools import setup

package_name = 'robcoop_lab'

setup(
    name=package_name,
    version='1.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raúl',
    description='Robótica Cooperativa: paquete base de las prácticas',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'consensus_node = robcoop_lab.consensus_node:main',
            'experiment_node = robcoop_lab.experiment_node:main',
            'anchor_node = robcoop_lab.anchor_node:main',
            'parar = robcoop_lab.stop_all:main',
            'plot_p1 = robcoop_lab.plot_p1:main',
            'check_p1 = robcoop_lab.check_p1:main',
            'kinsim = robcoop_lab.kinsim:main',
            'graph_info = robcoop_lab.graphs:main',
            'auction_node = robcoop_lab.auction_node:main',
            'auctioneer_node = robcoop_lab.auctioneer_node:main',
            'plot_p2 = robcoop_lab.plot_p2:main',
            'compare_p2 = robcoop_lab.compare_p2:main',
            'check_p2 = robcoop_lab.check_p2:main',
            'kinsim_p2 = robcoop_lab.kinsim_p2:main',
            'coverage_node = robcoop_lab.coverage_node:main',
            'coverage_experiment = robcoop_lab.coverage_experiment_node:main',
            'plot_p3 = robcoop_lab.plot_p3:main',
            'compare_p3 = robcoop_lab.compare_p3:main',
            'check_p3 = robcoop_lab.check_p3:main',
            'kinsim_p3 = robcoop_lab.kinsim_p3:main',
            'agent_node = robcoop_lab.agent_node:main',
            'mission_node = robcoop_lab.mission_node:main',
            'averia = robcoop_lab.fail_robot:main',
            'plot_p4 = robcoop_lab.plot_p4:main',
            'check_p4 = robcoop_lab.check_p4:main',
            'kinsim_p4 = robcoop_lab.kinsim_p4:main',
        ],
    },
)
