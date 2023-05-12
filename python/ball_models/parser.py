import pathlib
import pam_configuration

_CONFIG_FILE_SUFFIX = pathlib.Path("ball_models") / "config.toml"


def get_default_config_file() -> pathlib.Path:
    """
    Returns the absolute path to the default configuration, as it has been
    installed by the pam_configuration package, i.e.
    ~/.mpi-is/pam//tennicam_client/config.toml (if it exists) or
    /opt/mpi-is/pam/tennicam_client/config.toml (otherwise)
    """

    return pathlib.Path(pam_configuration.get_path()) / _CONFIG_FILE_SUFFIX
