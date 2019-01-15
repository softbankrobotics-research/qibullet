import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="qibullet",
    version="0.0.2",
    author="Maxime Busy, Maxime Caniot",
    author_email="",
    description="Bullet simulation for SBR robots",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ProtolabSBRE/qibullet",
    packages=setuptools.find_packages(),
    install_requires=['numpy', 'pybullet'],
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
        "Operating System :: Microsoft"
    ]
)
