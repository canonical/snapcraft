from setuptools import setup, find_packages

setup(
	name = "test",
	version = "0.1",
	packages = find_packages(),
        entry_points = {
		'console_scripts': [
			'python3_test = python3_test_package.__main__:main'
		]
	}
)
