"""Install parameters for CLI and python import."""
from setuptools import setup

with open('README.md') as in_file:
    long_description = in_file.read()

setup(
    name="alicat",
    version="0.6.3",
    description="Python driver for Alicat mass flow controllers.",
    long_description=long_description,
    long_description_content_type='text/markdown',
    url="https://github.com/alexrudd2/alicat/",
    author="Patrick Fuller",
    author_email="pat@numat-tech.com",
    maintainer="Alex Ruddick",
    maintainer_email="alex@ruddick.tech",
    packages=["alicat"],
    package_data={"alicat": ["py.typed"]},
    install_requires=["pyserial"],
    extras_require={
            'test': [
                'pytest>=8,<9',
                'pytest-cov>=5,<6',
                'pytest-asyncio>=0.23.5',
                'pytest-xdist==3.*',
                'ruff==0.5.2',
                'mypy==1.11.1',
                'types-pyserial',
            ],
        },
    entry_points={
        "console_scripts": [("alicat = alicat:command_line")]
    },
    license="GPLv2",
    classifiers=[
        "License :: OSI Approved :: GNU General Public License v2 (GPLv2)",
        "Development Status :: 4 - Beta",
        "Natural Language :: English",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering :: Human Machine Interfaces",
    ]
)
