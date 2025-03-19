from setuptools import find_packages, setup

package_name = "ff"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=["ff", "ff.*"], exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "lib/" + package_name,
            ["ff/arm.py", "ff/nav.py", "ff/globals.py", "ff/dock.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="northee",
    maintainer_email="prponkshe173@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["go = ff.run:main"],
    },
)
