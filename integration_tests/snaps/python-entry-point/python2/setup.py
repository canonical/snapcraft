from setuptools import setup, find_packages

setup(
	name = "test",
	version = "0.1",
	packages = find_packages(),
        entry_points = {
		'console_scripts': [
			'python2_test = python2_test_package.__main__:main'
		]
	}
)
