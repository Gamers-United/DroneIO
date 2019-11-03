import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="DroneIO", # Replace with your own username
    version="0.2",
    author="Mac",
    author_email="xmachd@gmail.com",
    description="Drone Control Python Library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Mac898/DroneIO",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
)