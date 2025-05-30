from setuptools import find_packages, setup

package_name = "jk"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="northee",
    maintainer_email="prponkshe173@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm = jk.arm:main",
            "nav = jk.nav:main",
            "dock = jk.dock:main",
        ],
    },
)
