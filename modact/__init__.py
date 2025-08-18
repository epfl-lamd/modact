from importlib.metadata import version, PackageNotFoundError

try:
    __version__ = version("modact")
except PackageNotFoundError:
    # package is not installed
    pass
