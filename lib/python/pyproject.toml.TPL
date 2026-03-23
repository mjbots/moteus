[build-system]
requires = ["setuptools", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "moteus"
version = {{VERSION}}
description = "moteus brushless controller library and tools"
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
    "pyserial>=3.5",
    "python-can>=3.3",
    "pyelftools>=0.26",
    "scipy>=1.8.0",
    "importlib_metadata>=3.6",
    "pywin32; platform_system == 'Windows'",
    "packaging",
]

[project.urls]
Homepage = "https://github.com/mjbots/moteus"

[project.scripts]
moteus_tool = "moteus.moteus_tool:main"

[tool.setuptools]
packages = ["moteus"]

[tool.mypy]
python_version = "3.11"
warn_unused_configs = true
ignore_missing_imports = true

# Start with basic checking, not strict
# These are disabled to allow gradual adoption
disallow_untyped_defs = false
check_untyped_defs = false
warn_return_any = false

# Exclude test files from checks initially
[[tool.mypy.overrides]]
module = ["moteus.test.*"]
ignore_errors = true
