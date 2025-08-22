from setuptools import setup

package_name = "brover_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ali",
    maintainer_email="ali@example.com",
    description="B-ROVER control package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = brover_control.bridge:main",
            "encoder_mapping = brover_control.encoder_mapping:main",
            "px4_control = brover_control.px4_control:main",
            "odom_pub = brover_control.odom_pub:main",
        ],
    },
)
