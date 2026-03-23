[build-system]
requires = ["setuptools", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "moteus_gui"
version = {{VERSION}}
description = "moteus brushless controller graphical user interfaces"
readme = "README.md"
license = {text = "Apache-2.0"}
requires-python = ">=3.7, <4"
authors = [{name = "mjbots Robotic Systems", email = "info@mjbots.com"}]
keywords = ["moteus"]
classifiers = [
    "Development Status :: 3 - Alpha",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: Apache Software License",
    "Programming Language :: Python :: 3",
]
dependencies = [
    "asyncqt>=0.8",
    "moteus~=0.3.92",
    "matplotlib>=3.9",
    # For some reason, matplotlib can barf without this, but
    # doesn't actually list it as a dependency on Windows.
    "msvc-runtime; platform_system == 'Windows' and python_version < '3.12'",
    "PySide6>=6.6",
    "qtconsole>=5.6",
    "qtpy>=2.0.1",
    "scipy>=1.14",
]

[project.urls]
Homepage = "https://github.com/mjbots/moteus"

[project.scripts]
tview = "moteus_gui.tview:main"

[tool.setuptools]
packages = ["moteus_gui"]

[tool.setuptools.package-data]
moteus_gui = ["tview_main_window.ui"]
